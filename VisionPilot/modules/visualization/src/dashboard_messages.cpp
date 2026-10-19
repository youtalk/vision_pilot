#include <nlohmann/json.hpp>
#include <visualization/box_projection.hpp>
#include <visualization/dashboard_messages.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace visualization::dashboard
{
namespace
{

using nlohmann::json;
namespace vm = visionpilot::models;
namespace vf = visionpilot::fusion;

constexpr char kSource[] = "visionpilot";
constexpr char kName[] = "VisionPilot";
constexpr float kMaxX = 158.f;     // 150 m of path plus a truck length
constexpr float kLeadPathM = 2.f;  // a lead candidate lies this close to the path
constexpr float kLeadMaxScore = 25.f;
constexpr double kTrajectoryWidthM = 2.3;

double r1(double v)
{
  return std::round(v * 10.0) / 10.0;
}
double r2(double v)
{
  return std::round(v * 100.0) / 100.0;
}

json envelope(const char * type, double t_ms)
{
  return {{"source", kSource}, {"type", type}, {"t_ms", r1(t_ms)}};
}

float path_y(const vf::LateralFusionEstimate & l, float x)
{
  return l.path_valid ? l.path_a * x * x + l.path_b * x + l.path_c : 0.f;
}

json objects(
  double t_ms, const vm::InferenceFrameResult & r, const cv::Mat & H, ObjectTracker & tracker)
{
  std::vector<GroundBox> boxes;
  std::vector<int> levels;
  std::vector<ObjectTracker::Point> pts;
  for (const auto & d : r.auto_speed.detections) {
    GroundBox g;
    if (!project_box(H, d.x1, d.y1, d.x2, d.y2, kMaxX, g)) continue;
    boxes.push_back(g);
    levels.push_back(d.class_id);
    pts.push_back({g.x_rear, g.y});
  }
  const std::vector<int> ids = tracker.update(pts);

  // The lead is the box the Occupancy view paints red: close to the path, nearest the fused CIPO
  // distance, with AutoSpeed's in-path level (class 1) preferred.
  int lead = -1;
  float best = kLeadMaxScore;
  const bool has_cipo = r.cipo.valid && r.cipo.distance_m > 0.5f && r.cipo.distance_m < 150.f;
  for (size_t i = 0; has_cipo && i < boxes.size(); ++i) {
    if (std::abs(boxes[i].y - path_y(r.lateral, boxes[i].x_rear)) > kLeadPathM) continue;
    const float score =
      std::abs(boxes[i].x_rear - r.cipo.distance_m) + (levels[i] == 1 ? 0.f : 2.f);
    if (score < best) {
      best = score;
      lead = static_cast<int>(i);
    }
  }

  json list = json::array();
  for (size_t i = 0; i < boxes.size(); ++i) {
    const GroundBox & g = boxes[i];
    const double length = g.truck ? 10.0 : 4.5;
    const bool is_lead = static_cast<int>(i) == lead;
    json o = {
      {"id", ids[i]},
      {"class", g.truck ? "truck" : "car"},
      {"x", r2(g.x_rear + length / 2)},
      {"y", r2(g.y)},
      {"yaw", 0.0},
      {"length", length},
      {"width", r2(g.width_m)},
      {"height", g.truck ? 3.2 : 1.5},
      {"lead", is_lead}};
    if (is_lead) o["distance_m"] = r2(r.cipo.distance_m);
    list.push_back(o);
  }
  json m = envelope("objects", t_ms);
  m["frame"] = "ego";
  m["objects"] = list;
  return m;
}

json trajectory(double t_ms, const vf::LateralFusionEstimate & l)
{
  // A failed fit sends no points, so the page clears the ribbon instead of keeping the last one.
  json pts = json::array();
  for (int x = 1; l.path_valid && x <= 150; ++x) {
    const double y = path_y(l, static_cast<float>(x));
    if (std::isfinite(y)) pts.push_back({x, r2(y)});
  }
  json m = envelope("trajectory", t_ms);
  m["frame"] = "ego";
  m["width_m"] = kTrajectoryWidthM;
  m["pts"] = pts;
  return m;
}

json alert(const char * code, const char * severity, const char * text, const char * target)
{
  return {{"code", code}, {"severity", severity}, {"text", text}, {"target", target}};
}

json alerts(double t_ms, const Plan & plan)
{
  json list = json::array();
  for (const Warning w : plan.warnings) {
    switch (w) {
      case Warning::FCW:
        list.push_back(alert("FCW", "critical", "Brake: vehicle ahead", "lead"));
        break;
      case Warning::AEB:
        list.push_back(alert("AEB", "critical", "Emergency braking", "lead"));
        break;
      // Lane departure is info: bench oscillation would otherwise keep the page in Alert.
      case Warning::LLDW:
        list.push_back(alert("LLDW", "info", "Drifting left", "trajectory"));
        break;
      case Warning::RLDW:
        list.push_back(alert("RLDW", "info", "Drifting right", "trajectory"));
        break;
      case Warning::None:
        break;
    }
  }
  json m = envelope("alerts", t_ms);
  m["alerts"] = list;
  return m;
}

}  // namespace

std::string hello(double t_ms, const std::string & device)
{
  json m = envelope("hello", t_ms);
  m["schema"] = 1;
  m["roles"] = json::array({"stack"});
  m["name"] = kName;
  if (!device.empty()) m["device"] = device;
  return m.dump();
}

std::vector<std::string> frame(
  double t_ms, const vm::InferenceFrameResult & r, const Plan & plan, const cv::Mat & H_resized,
  ObjectTracker & tracker)
{
  return {
    objects(t_ms, r, H_resized, tracker).dump(), trajectory(t_ms, r.lateral).dump(),
    alerts(t_ms, plan).dump()};
}

std::string status(
  double t_ms, const vm::InferenceFrameResult & r, const std::string & device,
  double speed_limit_mps)
{
  json m = envelope("status", t_ms);
  m["name"] = kName;
  if (!device.empty()) m["device"] = device;
  m["mode"] = "active";
  m["latency_ms"] = r1(r.wall_ms);
  m["stages"] = {
    {"pre", r1(r.pre_ms)},
    {"AutoDrive", r1(r.ad_ms)},
    {"AutoSteer", r1(r.as_ms)},
    {"AutoSpeed", r1(r.asp_ms)}};
  m["speed_limit_mps"] = speed_limit_mps;
  return m.dump();
}

}  // namespace visualization::dashboard
