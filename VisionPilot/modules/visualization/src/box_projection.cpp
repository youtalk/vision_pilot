#include <visualization/box_projection.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace visualization
{
namespace
{

bool px_to_ground(const cv::Mat & H, float u, float v, float max_x_m, float & x, float & y)
{
  std::vector<cv::Point2f> src = {cv::Point2f(u, v)}, dst;
  cv::perspectiveTransform(src, dst, H);
  x = dst[0].x;
  y = dst[0].y;
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
