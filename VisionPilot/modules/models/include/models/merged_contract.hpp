#pragma once

#include <models/auto_drive.hpp>
#include <models/auto_speed.hpp>
#include <models/auto_steer.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace visionpilot::models {

// One row of the head rule: which logical output this row produces, and the
// activation to apply after dividing by the row's alpha.
struct ContractHeadMap {
    std::string output;      // "drive_distance" | "drive_curvature" | "drive_flag_logit"
    std::string activation;  // "relu" | "tanh" | "none"
};

// The fused drive head (R6). Three parallel 1-channel head convolutions are
// folded into one 3-channel conv with per-row int8 range-balancing factors,
// and Relu/Tanh move to the host.
struct ContractHead {
    std::string                  output;  // graph output holding [1,3,1,1]
    std::vector<float>           alpha;   // per-row balance factors
    std::vector<ContractHeadMap> map;     // same length as alpha
};

// The lane soft-argmax, moved to the host by R8 (v7 only). int8 softmax
// quantisation caused frame-to-frame trajectory wobble, so the graph now emits
// pre-softmax logits and the host decodes them in floating point.
struct ContractSteerXp {
    std::string logits;         // graph output, [1,1,rows,positions]
    int         positions = 0;  // row width; 256 for v7
    float       div       = 1.f;  // divisor; 256.0 for v7
};

// One FPN level of the speed DFL tail (R7), decoded before any cross-level
// concatenation.
struct ContractSpeedLevel {
    std::string box;         // graph output, [1,4,H,W], grid units
    std::string cls;         // graph output, [1,K,H,W], already sigmoided
    int         stride = 0;  // grid units -> pixels
    int         h      = 0;
    int         w      = 0;
};

struct ContractSpeed {
    std::vector<ContractSpeedLevel> levels;
};

// Host-side postprocessing description emitted alongside an NPU-legal merged
// model. Absent contract means plain-merged mode: raw prefixed outputs.
struct MergedContract {
    std::string              attn_mode;    // "frozen" | "keep" | "uniform" | "identity"
    std::vector<std::string> passthrough;  // outputs copied verbatim
    std::optional<ContractHead>    head;
    std::optional<ContractSteerXp> steer_xp;
    std::optional<ContractSpeed>   speed;

    // Throws std::runtime_error on malformed input or an internally
    // inconsistent contract (alpha/map length mismatch, empty level list,
    // non-positive positions, zero divisor).
    static MergedContract from_json_string(const std::string& text);
    static MergedContract from_file(const std::string& path);
};

// Apply the head rule to a [1,3,1,1] raw head tensor: divide element i by
// alpha[i], apply map[i].activation, and store into the field named by
// map[i].output. The "drive_flag_logit" row is passed through sigmoid, matching
// what the split path does in auto_drive.cpp.
// Throws std::runtime_error when raw_count is smaller than the contract's row
// count or when a row names an unknown output.
void apply_head(const ContractHead& head, const float* raw, size_t raw_count,
                AutoDriveOutput& out);

// Decode the lane soft-argmax on the host (R8, v7). logits is a row-major
// buffer of rows * rule.positions floats; rows is inferred from count.
//
//   xp[r] = softmax(logits[r]) . arange(positions) / div
//
// Computed in double precision with row-max subtraction before exponentiating,
// matching openadkit's `npu-final-check.py` reference decoder. Sets out.valid.
//
// Throws std::runtime_error when rule.positions is not positive, when
// rule.div is zero, when count is not a positive multiple of rule.positions,
// or when the row count is not AutoSteerOutput::xp's size.
void apply_steer_xp(const ContractSteerXp& rule, const float* logits,
                    size_t count, AutoSteerOutput& out);

// Resolve a contract path. Checks "<model_path>.contract.json" first, then
// "<artifacts_dir>/contract.json". Returns "" when neither exists.
// Either argument may be empty.
std::string resolve_contract_path(const std::string& model_path,
                                  const std::string& artifacts_dir);

// True when the session exposes an output that only exists after the NPU-legal
// rewrite and therefore cannot be interpreted without a contract. Loading such
// a model without its contract would yield an unscaled drive head, undecoded
// speed boxes, and under v7 no ego path at all, with no error. Both v6 and v7
// expose drive_head_raw, so one check covers both.
bool has_rewrite_signature_output(const std::vector<std::string>& output_names);

}  // namespace visionpilot::models
