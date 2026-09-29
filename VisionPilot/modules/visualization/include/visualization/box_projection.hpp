#pragma once

#include <opencv2/core.hpp>

namespace visualization
{

// One AutoSpeed box placed on the ground in the ego frame (x forward, y left) [m].
struct GroundBox
{
  float x_rear;  // bottom-center of the box, which is the visible rear face
  float y;
  float width_m;  // clamped to [0.9, 3.2]
  bool truck;
};

// Projects the bottom edge of a box, given in resized-image pixels, through H (pixel to ground).
// Returns false for an empty H, and for a bottom corner that lands outside [-2, max_x_m] or at a
// non-finite point. A box at or above the horizon therefore never reaches the caller.
bool project_box(
  const cv::Mat & H, float x1, float y1, float x2, float y2, float max_x_m, GroundBox & out);

}  // namespace visualization
