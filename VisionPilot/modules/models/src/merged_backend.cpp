#include "models/merged_backend.hpp"

#include <onnxruntime_run_options_config_keys.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace visionpilot::models {

namespace {

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::duration<double, std::milli>;

// Copy a 64-element tensor into a fixed 64-float array.
void copy_64(const Ort::Value& v, std::array<float, 64>& dst,
             const std::string& name)
{
    const auto info = v.GetTensorTypeAndShapeInfo();
    if (info.GetElementCount() < dst.size()) {
        throw std::runtime_error(
            "[MergedBackend] output '" + name + "' has " +
            std::to_string(info.GetElementCount()) +
            " elements, expected at least 64");
    }
    std::memcpy(dst.data(), v.GetTensorData<float>(),
                dst.size() * sizeof(float));
}

}  // namespace

MergedBackend::MergedBackend(engine::OnnxEngine& engine,
                             const std::string&  model_path,
                             const std::string&  contract_path)
    : session_(engine.create_session(model_path, "merged_"))
    , mem_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
    , frame_shape_{1, 3, NET_H, NET_W}
    , arena_shrink_(engine.config().provider == "cpu" ||
                    engine.config().provider == "renesas"
                        ? "cpu:0" : "cpu:0;gpu:0")
    , provider_(engine.config().provider)
    , require_npu_nodes_(engine.config().require_npu_nodes)
{
    Ort::AllocatorWithDefaultOptions alloc;

    const size_t n_in = session_->GetInputCount();
    in_name_strs_.resize(n_in);
    in_names_.resize(n_in);
    for (size_t i = 0; i < n_in; ++i) {
        in_name_strs_[i] = session_->GetInputNameAllocated(i, alloc).get();
        in_names_[i]     = in_name_strs_[i].c_str();
        printf("[MergedBackend] input[%zu]  = %s\n", i, in_names_[i]);
    }

    const size_t n_out = session_->GetOutputCount();
    out_name_strs_.resize(n_out);
    out_names_.resize(n_out);
    for (size_t i = 0; i < n_out; ++i) {
        out_name_strs_[i] = session_->GetOutputNameAllocated(i, alloc).get();
        out_names_[i]     = out_name_strs_[i].c_str();
        out_index_[out_name_strs_[i]] = i;
        printf("[MergedBackend] output[%zu] = %s\n", i, out_names_[i]);
    }

    if (!contract_path.empty()) {
        contract_ = MergedContract::from_file(contract_path);
        printf("[MergedBackend] contract=%s  attn_mode=%s  lane=%s\n",
               contract_path.c_str(),
               contract_->attn_mode.empty() ? "(unset)"
                                            : contract_->attn_mode.c_str(),
               contract_->steer_xp ? "host soft-argmax (v7 R8)"
                                   : "graph output (v6)");
        if (!contract_->attn_mode.empty() && contract_->attn_mode != "keep") {
            printf("[MergedBackend] WARNING: attn_mode='%s' — the ego-path "
                   "output (AutoSteer xp) is degraded. openadkit measured lane "
                   "correlation 0.653-0.954 against the CPU reference for "
                   "'frozen'. Do not treat xp as reference-quality.\n",
                   contract_->attn_mode.c_str());
        }
    } else if (has_rewrite_signature_output(out_name_strs_)) {
        throw std::runtime_error(
            "[MergedBackend] this model exposes rewrite outputs "
            "(drive_head_raw / speed_l*_box) but no contract was found. "
            "Running it without the contract would silently produce an "
            "unscaled drive head, undecoded speed boxes, and no ego path. "
            "Place contract.json in the artifacts directory, or set "
            "model.contract.");
    } else {
        printf("[MergedBackend] no contract — plain-merged mode\n");
    }

    // Three inputs are mandatory in both modes.
    for (const char* required : {"input", "drive_image_prev", "drive_image_curr"}) {
        bool found = false;
        for (const auto& n : in_name_strs_) if (n == required) found = true;
        if (!found) {
            throw std::runtime_error(
                std::string("[MergedBackend] merged model is missing required "
                            "input '") + required + "'");
        }
    }

    validate_contract();

    printf("[MergedBackend] Ready — %zu inputs, %zu outputs\n", n_in, n_out);
}

const Ort::Value* MergedBackend::find_output(const std::string& name) const
{
    const auto it = out_index_.find(name);
    if (it == out_index_.end()) return nullptr;
    return &results_[it->second];
}

void MergedBackend::validate_contract() const
{
    if (!contract_) return;

    // 1. Every name the contract mentions must exist in the session. This is
    //    also what catches a v6 contract paired with a v7 model (which does not
    //    output steer_lane_value) and the reverse (no steer_silu_41).
    std::vector<std::string> missing;
    const auto require = [&](const std::string& name) {
        if (out_index_.find(name) == out_index_.end()) missing.push_back(name);
    };

    for (const auto& n : contract_->passthrough) require(n);
    if (contract_->head)     require(contract_->head->output);
    if (contract_->steer_xp) require(contract_->steer_xp->logits);
    if (contract_->speed) {
        for (const auto& l : contract_->speed->levels) {
            require(l.box);
            require(l.cls);
        }
    }

    if (!missing.empty()) {
        std::string msg =
            "[MergedBackend] the contract names outputs this session does not "
            "expose.\n  Missing:";
        for (const auto& m : missing) msg += "\n    " + m;
        msg += "\n  Session outputs:";
        for (const auto& n : out_name_strs_) msg += "\n    " + n;
        throw std::runtime_error(msg);
    }

    // 2. The ego path must be supplied exactly once: either as a v6
    //    steer_lane_value passthrough, or by the v7 steer_xp rule. With
    //    neither, xp would stay zero and the vehicle would steer on an empty
    //    path with no error anywhere.
    bool lane_passthrough = false;
    for (const auto& n : contract_->passthrough) {
        if (n == "steer_lane_value") lane_passthrough = true;
    }
    if (!lane_passthrough && !contract_->steer_xp) {
        throw std::runtime_error(
            "[MergedBackend] the contract supplies no ego path: it has neither "
            "a 'steer_lane_value' passthrough (v6) nor a 'steer_xp' rule "
            "(v7). AutoSteer xp would stay zero and lateral fusion would "
            "steer on an empty path.");
    }
    if (lane_passthrough && contract_->steer_xp) {
        throw std::runtime_error(
            "[MergedBackend] the contract supplies the ego path twice: both a "
            "'steer_lane_value' passthrough and a 'steer_xp' rule. Exactly "
            "one is expected.");
    }

    // 3. Each speed level's box/cls outputs must have the element counts the
    //    contract's declared hw implies, read from the session's static output
    //    shapes (no run required). assemble_speed() re-checks this per frame
    //    against the real tensors (box_count/cls_count) as a defence-in-depth
    //    guard, but a contract that mis-describes the graph's geometry must
    //    fail here, at startup, rather than there, on the first frame.
    if (contract_->speed) {
        const auto shape_of = [&](const std::string& name) {
            return session_->GetOutputTypeInfo(out_index_.at(name))
                .GetTensorTypeAndShapeInfo()
                .GetShape();
        };
        const auto element_count =
            [&](const std::vector<int64_t>& shape,
                const std::string& name) -> int64_t {
            int64_t count = 1;
            for (const auto d : shape) {
                if (d <= 0) {
                    throw std::runtime_error(
                        "[MergedBackend] speed output '" + name + "' has a "
                        "dynamic or unknown dimension in its declared shape; "
                        "its geometry cannot be verified against the "
                        "contract at startup");
                }
                count *= d;
            }
            return count;
        };

        for (const auto& l : contract_->speed->levels) {
            const auto box_shape = shape_of(l.box);
            const auto cls_shape = shape_of(l.cls);

            const int64_t box_count = element_count(box_shape, l.box);
            const int64_t expected_box =
                4LL * static_cast<int64_t>(l.h) * static_cast<int64_t>(l.w);
            if (box_count != expected_box) {
                throw std::runtime_error(
                    "[MergedBackend] speed box output '" + l.box + "' has " +
                    std::to_string(box_count) + " elements but the "
                    "contract's hw=[" + std::to_string(l.h) + "," +
                    std::to_string(l.w) + "] requires 4*h*w=" +
                    std::to_string(expected_box));
            }

            if (cls_shape.size() < 2) {
                throw std::runtime_error(
                    "[MergedBackend] speed cls output '" + l.cls +
                    "' has rank " + std::to_string(cls_shape.size()) +
                    ", expected at least 2 to read a class dimension");
            }
            const int64_t cls_count = element_count(cls_shape, l.cls);
            const int64_t num_classes = cls_shape[1];
            const int64_t expected_cls = num_classes *
                                         static_cast<int64_t>(l.h) *
                                         static_cast<int64_t>(l.w);
            if (cls_count != expected_cls) {
                throw std::runtime_error(
                    "[MergedBackend] speed cls output '" + l.cls + "' has " +
                    std::to_string(cls_count) + " elements but num_classes(" +
                    std::to_string(num_classes) + ")*h*w=" +
                    std::to_string(expected_cls) + " is required by the "
                    "contract's hw=[" + std::to_string(l.h) + "," +
                    std::to_string(l.w) + "]");
            }
        }
    }
}

BackendOutputs MergedBackend::run(const float* prev_imn,
                                  const float* curr_imn,
                                  const float* curr_01)
{
    BackendOutputs out;

    // Feed in the session's declared input order.
    std::vector<Ort::Value> inputs;
    inputs.reserve(in_name_strs_.size());
    for (const auto& name : in_name_strs_) {
        const float* src = nullptr;
        if (name == "input")                 src = curr_01;
        else if (name == "drive_image_prev") src = prev_imn;
        else if (name == "drive_image_curr") src = curr_imn;
        else {
            printf("[MergedBackend] Unexpected input '%s'\n", name.c_str());
            return out;
        }
        inputs.push_back(Ort::Value::CreateTensor<float>(
            mem_info_, const_cast<float*>(src), CHW_SIZE,
            frame_shape_.data(), frame_shape_.size()));
    }

    const auto t = Clock::now();
    try {
        Ort::RunOptions run_options;
        run_options.AddConfigEntry(
            kOrtRunOptionsConfigEnableMemoryArenaShrinkage,
            arena_shrink_.c_str());
        results_ = session_->Run(run_options,
                                 in_names_.data(), inputs.data(), inputs.size(),
                                 out_names_.data(), out_names_.size());
    } catch (const Ort::Exception& e) {
        printf("[MergedBackend] Inference error: %s\n", e.what());
        return out;
    }
    out.ad_ms = Ms(Clock::now() - t).count();

    try {
        if (contract_) {
            for (const auto& name : contract_->passthrough) {
                const Ort::Value* v = find_output(name);
                if (v == nullptr) {
                    throw std::runtime_error(
                        "[MergedBackend] contract passthrough names '" + name +
                        "' but the session has no such output");
                }
                if (name == "steer_lane_value")  copy_64(*v, out.steer.xp, name);
                else if (name == "steer_height") copy_64(*v, out.steer.h_vector, name);
                else {
                    throw std::runtime_error(
                        "[MergedBackend] unknown passthrough output '" + name +
                        "'. Expected steer_lane_value or steer_height");
                }
            }

            // v7: recover the ego path from pre-softmax logits on the host.
            if (contract_->steer_xp) {
                const Ort::Value* v = find_output(contract_->steer_xp->logits);
                if (v == nullptr) {
                    throw std::runtime_error(
                        "[MergedBackend] contract steer_xp names '" +
                        contract_->steer_xp->logits +
                        "' but the session has no such output");
                }
                apply_steer_xp(*contract_->steer_xp, v->GetTensorData<float>(),
                               v->GetTensorTypeAndShapeInfo().GetElementCount(),
                               out.steer);
            } else {
                // v6: xp arrived through passthrough, validated at startup.
                out.steer.valid = true;
            }

            if (contract_->head) {
                const Ort::Value* v = find_output(contract_->head->output);
                if (v == nullptr) {
                    throw std::runtime_error(
                        "[MergedBackend] contract head names '" +
                        contract_->head->output +
                        "' but the session has no such output");
                }
                apply_head(*contract_->head, v->GetTensorData<float>(),
                           v->GetTensorTypeAndShapeInfo().GetElementCount(),
                           out.drive);
            }

            if (contract_->speed) {
                std::vector<SpeedLevelTensors> levels;
                levels.reserve(contract_->speed->levels.size());
                for (const auto& l : contract_->speed->levels) {
                    const Ort::Value* b = find_output(l.box);
                    const Ort::Value* c = find_output(l.cls);
                    if (b == nullptr || c == nullptr) {
                        throw std::runtime_error(
                            "[MergedBackend] contract speed level names '" +
                            l.box + "' / '" + l.cls +
                            "' but the session lacks one of them");
                    }
                    const auto cls_shape =
                        c->GetTensorTypeAndShapeInfo().GetShape();
                    if (cls_shape.size() < 2) {
                        throw std::runtime_error(
                            "[MergedBackend] speed cls output '" + l.cls +
                            "' has rank < 2");
                    }
                    SpeedLevelTensors t_l;
                    t_l.box         = b->GetTensorData<float>();
                    t_l.cls         = c->GetTensorData<float>();
                    t_l.h           = l.h;
                    t_l.w           = l.w;
                    t_l.stride      = l.stride;
                    t_l.num_classes = static_cast<int>(cls_shape[1]);
                    t_l.box_count   = b->GetTensorTypeAndShapeInfo().GetElementCount();
                    t_l.cls_count   = c->GetTensorTypeAndShapeInfo().GetElementCount();
                    levels.push_back(t_l);
                }
                const auto a = assemble_speed(levels);
                // R7 emits cls already sigmoided.
                out.speed = decode_detections(a.data.data(), a.channels,
                                              a.anchors,
                                              /*cls_is_probability=*/true,
                                              conf_thres_, iou_thres_);
            }
        } else {
            // Plain merged: prefixed copies of the original outputs.
            if (const Ort::Value* v = find_output("steer_xp")) {
                copy_64(*v, out.steer.xp, "steer_xp");
            }
            if (const Ort::Value* v = find_output("steer_h_vector")) {
                copy_64(*v, out.steer.h_vector, "steer_h_vector");
                out.steer.valid = true;
            }
            if (const Ort::Value* v = find_output("speed_output")) {
                const auto shape = v->GetTensorTypeAndShapeInfo().GetShape();
                if (shape.size() >= 3) {
                    // The standalone graph emits class logits.
                    out.speed = decode_detections(v->GetTensorData<float>(),
                                                  shape[1], shape[2],
                                                  /*cls_is_probability=*/false,
                                                  conf_thres_, iou_thres_);
                }
            }
            const Ort::Value* d0 = find_output("drive_distance");
            const Ort::Value* d1 = find_output("drive_curvature");
            const Ort::Value* d2 = find_output("drive_flag_logit");
            if (d0 && d1 && d2) {
                out.drive.dist_normalized = d0->GetTensorData<float>()[0];
                out.drive.curvature_raw   = d1->GetTensorData<float>()[0];
                out.drive.flag_prob =
                    1.f / (1.f + std::exp(-d2->GetTensorData<float>()[0]));
                out.drive.valid = true;
            }
        }
    } catch (const std::runtime_error& e) {
        // Structural contract errors are caught at startup by
        // validate_contract(); a shape surprise can still only surface on the
        // first real run. Report and mark the frame invalid rather than
        // aborting mid-drive.
        printf("[MergedBackend] Postprocessing error: %s\n", e.what());
        return BackendOutputs{};
    }

    return out;
}

}  // namespace visionpilot::models
