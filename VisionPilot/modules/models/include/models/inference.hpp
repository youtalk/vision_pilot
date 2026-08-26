#pragma once

#include <fusion/lateral_fusion.hpp>
#include <fusion/longitudinal_fusion.hpp>
#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>
#include <models/auto_speed.hpp>
#include <models/backend.hpp>
#include <opencv2/core.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace visionpilot::engine {
class OnnxEngine;
}

namespace visionpilot::models {

struct Config {
    std::string precision    = "fp32";
    bool        fusion_debug = false;
    float       cte_bias_m   = 0.0f;  // camera mounting offset [m] — subtracted from raw CTE before filter

    // Run all three networks in one session over a merged graph. Required for
    // the renesas provider, which permits only one NPU session per process.
    bool        merged = false;
    // Merged .onnx path. Ignored under the renesas provider, which uses
    // engine.artifacts_dir instead.
    std::string merged_path;
    // "auto" resolves <merged_path>.contract.json or
    // <artifacts_dir>/contract.json. "none" forces plain-merged mode.
    // Anything else is treated as an explicit contract path.
    std::string contract = "auto";
};

struct LatencyStats {
    double pre{0}, ad{0}, as{0}, asp{0}, wall{0};

    void update(double pre_, double ad_, double as_, double asp_, double wall_);
    void print() const;
    void reset();
};

struct InferenceFrameResult {
    uint64_t    frame_id = 0;
    double      wall_ms  = 0;
    double      pre_ms   = 0;
    double      ad_ms    = 0;
    double      as_ms    = 0;
    double      asp_ms   = 0;

    AutoDriveOutput              auto_drive;
    AutoSteerOutput              auto_steer;
    AutoSpeedOutput              auto_speed;
    fusion::CIPOFusionEstimate   cipo;
    fusion::LateralFusionEstimate  lateral;
};

// Two-frame buffer → parallel ONNX → longitudinal + lateral fusion.
class InferencePipeline {
public:
    InferencePipeline(engine::OnnxEngine& engine, const Config& cfg);

    // nullopt until two frames collected (AutoDrive needs t-1 and t).
    // warped  : BEV 1024×512 image → AutoDrive only.
    // resized : plain-resized 1024×512 image → AutoSteer + AutoSpeed.
    //           If empty, falls back to warped for all networks (legacy behaviour).
    std::optional<InferenceFrameResult> process(const cv::Mat& warped,
                                                const cv::Mat& resized = {});

    // Compute and apply H_resized to both fusion modules so that AutoSteer /
    // AutoSpeed outputs are projected correctly when they run on a resized
    // (non-BEV) image.  Call once after the first preprocess() when using
    // the resized routing.
    //   C        : raw-camera → warped-BEV homography (from ImagePreprocessor)
    //   raw_size : original frame dimensions before top-crop + resize
    void set_H_resized(const cv::Mat& H, cv::Size raw_size);

    // H that maps resized-image pixel → world (set after set_H_resized()).
    // Empty until set_H_resized() is called.
    const cv::Mat& H_resized() const { return H_resized_; }

    // Inverse of H_resized: world → resized-image pixel.
    // Used by both debug and production visualizers for path projection.
    const cv::Mat& H_world2resized() const { return H_world2resized_; }

    void reset();
    const LatencyStats& latency() const { return stats_; }

private:
    cv::Mat H_resized_;
    cv::Mat H_world2resized_;
    std::unique_ptr<ModelBackend> backend_;
    fusion::LongitudinalFusion long_fusion_;
    fusion::LateralFusion      lat_fusion_;
    LatencyStats       stats_;
    uint64_t           frame_count_ = 0;

    cv::Mat prev_frame_;
    cv::Mat curr_frame_;
    int     frame_buf_count_ = 0;

    bool offload_verified_ = false;
};

}  // namespace visionpilot::models
