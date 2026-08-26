#pragma once

#include <engine/onnx_engine.hpp>
#include <models/backend.hpp>
#include <models/merged_contract.hpp>
#include <onnxruntime_cxx_api.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace visionpilot::models {

// All three networks in one ONNX Runtime session. Required for the Renesas
// execution provider, which permits only one NPU session per process.
//
// Inputs, fixed by the merged graph:
//   input             [1,3,512,1024]  /255 resized, shared by steer and speed
//   drive_image_prev  [1,3,512,1024]  ImageNet-normalised warped BEV, t-1
//   drive_image_curr  [1,3,512,1024]  ImageNet-normalised warped BEV, t
class MergedBackend : public ModelBackend {
public:
    static constexpr int NET_H    = 512;
    static constexpr int NET_W    = 1024;
    static constexpr int CHW_SIZE = 3 * NET_H * NET_W;

    // model_path   : the .onnx to load. For the renesas provider this is the
    //                artifacts directory; OnnxEngine resolves it.
    // contract_path: "" means plain-merged mode. A rewrite signature output
    //                with no contract is a startup error.
    MergedBackend(engine::OnnxEngine& engine,
                  const std::string&  model_path,
                  const std::string&  contract_path);

    BackendOutputs run(const float* prev_imn,
                       const float* curr_imn,
                       const float* curr_01) override;

    // Run one warm-up frame and assert the NPU actually executed nodes.
    // Only meaningful for the renesas provider; a no-op otherwise.
    // Throws std::runtime_error when the NPU did no work, or when fewer than
    // engine.require_npu_nodes nodes were placed on it.
    void verify_offload(const float* prev_imn,
                        const float* curr_imn,
                        const float* curr_01);

private:
    const Ort::Value* find_output(const std::string& name) const;
    void validate_contract() const;

    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo               mem_info_;

    std::vector<std::string> in_name_strs_;
    std::vector<const char*> in_names_;
    std::vector<std::string> out_name_strs_;
    std::vector<const char*> out_names_;
    std::unordered_map<std::string, size_t> out_index_;

    std::optional<MergedContract> contract_;
    std::vector<int64_t>          frame_shape_;
    std::string                   arena_shrink_;
    std::string                   provider_;
    int                           require_npu_nodes_ = 0;
    float                         conf_thres_ = 0.6f;
    float                         iou_thres_  = 0.45f;

    // Filled per run so find_output() can resolve by name.
    std::vector<Ort::Value> results_;
};

}  // namespace visionpilot::models
