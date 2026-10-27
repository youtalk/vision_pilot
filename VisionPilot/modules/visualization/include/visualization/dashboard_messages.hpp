#pragma once

#include <common/types.hpp>
#include <models/inference.hpp>
#include <opencv2/core.hpp>
#include <visualization/object_tracker.hpp>

#include <string>
#include <vector>

// JSON text for the ADAS dashboard's `stack` role, schema 1.
namespace visualization::dashboard
{

std::string hello(double t_ms, const std::string & device);

// objects, trajectory and alerts for one inference frame, in that order. The dashboard's ego frame
// starts at the rear axle, rear_axle_m behind the H origin, so every x gets rear_axle_m added.
std::vector<std::string> frame(
  double t_ms, const visionpilot::models::InferenceFrameResult & r, const Plan & plan,
  const cv::Mat & H_resized, ObjectTracker & tracker, double rear_axle_m);

std::string status(
  double t_ms, const visionpilot::models::InferenceFrameResult & r, const std::string & device,
  double speed_limit_mps);

}  // namespace visualization::dashboard
