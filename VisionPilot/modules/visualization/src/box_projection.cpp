#include <visualization/box_projection.hpp>

#include <algorithm>
#include <cmath>

namespace visualization
{
namespace
{

bool px_to_ground(const cv::Mat & H, float u, float v, float max_x_m, float & x, float & y)
{
  // perspectiveTransform writes (0, 0) instead of inf when w is about 0, so divide here.
  cv::Mat Hd;
  H.convertTo(Hd, CV_64F);
  const double w = Hd.at<double>(2, 0) * u + Hd.at<double>(2, 1) * v + Hd.at<double>(2, 2);
  if (std::abs(w) < 1e-6) return false;
  x = static_cast<float>(
    (Hd.at<double>(0, 0) * u + Hd.at<double>(0, 1) * v + Hd.at<double>(0, 2)) / w);
  y = static_cast<float>(
    (Hd.at<double>(1, 0) * u + Hd.at<double>(1, 1) * v + Hd.at<double>(1, 2)) / w);
  return std::isfinite(x) && std::isfinite(y) && x >= -2.f && x <= max_x_m;
}

bool looks_like_truck(float width_m, float bbox_w, float bbox_h)
{
  if (width_m >= 2.25f) return true;
  if (width_m >= 1.95f && bbox_h > bbox_w * 0.85f) return true;
  if (bbox_h > 140.f && bbox_w > 90.f && width_m >= 1.7f) return true;
  return false;
}

}  // namespace

bool project_box(
  const cv::Mat & H, float x1, float y1, float x2, float y2, float max_x_m, GroundBox & out)
{
  if (H.empty()) return false;
  float x_bl, y_bl, x_br, y_br, x_bc, y_bc;
  if (!px_to_ground(H, x1, y2, max_x_m, x_bl, y_bl)) return false;
  if (!px_to_ground(H, x2, y2, max_x_m, x_br, y_br)) return false;
  if (!px_to_ground(H, 0.5f * (x1 + x2), y2, max_x_m, x_bc, y_bc)) return false;
  out.x_rear = x_bc;
  out.y = y_bc;
  out.width_m = std::clamp(std::abs(y_bl - y_br), 0.9f, 3.2f);
  out.truck = looks_like_truck(out.width_m, std::max(1.f, x2 - x1), std::max(1.f, y2 - y1));
  return true;
}

}  // namespace visualization
