// Tests for the dashboard `stack` endpoint.
// Run: test_dashboard [OUT.jsonl]   or   test_dashboard --serve PORT SECONDS
#include <common/types.hpp>
#include <models/inference.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <visualization/box_projection.hpp>
#include <visualization/dashboard_messages.hpp>
#include <visualization/dashboard_server.hpp>
#include <visualization/object_tracker.hpp>

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <future>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// assert() vanishes under NDEBUG, so the checks use their own macro.
#define CHECK(c)                                                                 \
  do {                                                                           \
    if (!(c)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); \
      std::exit(1);                                                              \
    }                                                                            \
  } while (0)

namespace
{

namespace vz = visualization;
namespace vm = visionpilot::models;
using nlohmann::json;

// x = 1000 / (v - 256) and y = (512 - u) / (v - 256): the horizon is image row 256.
cv::Mat test_H()
{
  return (cv::Mat_<double>(3, 3) << 0, 0, 1000, -1, 0, 512, 0, 1, -256);
}

void test_project_box()
{
  vz::GroundBox g;
  CHECK(vz::project_box(test_H(), 494, 250, 530, 276, 158.f, g));
  CHECK(std::abs(g.x_rear - 50.f) < 1e-3f && std::abs(g.y) < 1e-3f);
  CHECK(std::abs(g.width_m - 1.8f) < 1e-3f && !g.truck);
  CHECK(vz::project_box(test_H(), 412, 150, 612, 286, 158.f, g));
  CHECK(g.truck && g.width_m == 3.2f);
  CHECK(!vz::project_box(test_H(), 494, 200, 530, 257, 158.f, g));  // at the horizon: x = 1000 m
  CHECK(!vz::project_box(test_H(), 494, 200, 530, 256, 158.f, g));  // on the horizon: w = 0
  CHECK(!vz::project_box(test_H(), 494, 200, 530, 250, 158.f, g));  // above it: behind the ego
  CHECK(!vz::project_box(cv::Mat(), 494, 250, 530, 276, 158.f, g));
}

void test_tracker()
{
  vz::ObjectTracker t;
  CHECK((t.update({{10, 0}, {30, 3}}) == std::vector<int>{0, 1}));
  CHECK((t.update({{31, 3}, {11, 0}}) == std::vector<int>{1, 0}));   // by position, not order
  CHECK((t.update({{11, 0}, {50, -3}}) == std::vector<int>{0, 2}));  // beyond the 3 m gate
  for (int i = 0; i < 5; ++i) t.update({});
  CHECK((t.update({{11, 0}}) == std::vector<int>{0}));  // kept through 5 missed frames
  for (int i = 0; i < 6; ++i) t.update({});
  CHECK((t.update({{11, 0}}) == std::vector<int>{3}));  // dropped at the 6th
}

json by_type(const std::vector<std::string> & msgs, const std::string & type)
{
  for (const auto & s : msgs) {
    json j = json::parse(s);
    if (j["type"] == type) return j;
  }
  CHECK(false);
  return {};
}

// A car in the path at 50 m, a truck at 33 m, and a box at the horizon.
vm::InferenceFrameResult sample_frame()
{
  vm::InferenceFrameResult r;
  r.wall_ms = 24.7;
  r.pre_ms = 1.8;
  r.ad_ms = 8.1;
  r.as_ms = 7.2;
  r.asp_ms = 6.9;
  r.auto_speed.detections = {
    {494, 250, 530, 276, 0.9f, 1}, {412, 150, 612, 286, 0.8f, 2}, {494, 200, 530, 257, 0.7f, 1}};
  r.cipo.valid = true;
  r.cipo.distance_m = 50.f;
  r.lateral.path_valid = true;
  r.lateral.path_a = 0.0002f;  // y = 0.0002 x^2: 0.5 m left at 50 m, 2 m at 100 m
  return r;
}

void test_frame()
{
  vz::ObjectTracker tracker;
  vm::InferenceFrameResult r = sample_frame();
  const auto msgs = vz::dashboard::frame(
    1000.0, r, Plan{-4.0, {0.0, 0.0}, {Warning::FCW, Warning::LLDW}}, test_H(), tracker);
  CHECK(msgs.size() == 3);

  const json objs = by_type(msgs, "objects")["objects"];
  CHECK(objs.size() == 2);  // the horizon box is dropped
  CHECK(objs[0]["class"] == "car" && objs[0]["lead"] == true && objs[0]["distance_m"] == 50.0);
  CHECK(std::abs(objs[0]["x"].get<double>() - 52.25) < 0.01);  // rear face + half of 4.5 m
  CHECK(objs[1]["class"] == "truck" && objs[1]["lead"] == false);
  CHECK(!objs[1].contains("distance_m") && objs[0]["id"] != objs[1]["id"]);

  const json traj = by_type(msgs, "trajectory");
  CHECK(traj["pts"].size() == 150 && traj["pts"][0][0] == 1 && traj["pts"][149][0] == 150);
  CHECK(std::abs(traj["pts"][99][1].get<double>() - 2.0) < 0.01);

  const json alerts = by_type(msgs, "alerts")["alerts"];
  CHECK(alerts.size() == 2);
  CHECK(alerts[0]["code"] == "FCW" && alerts[0]["severity"] == "critical");
  CHECK(alerts[0]["target"] == "lead");
  CHECK(alerts[1]["code"] == "LLDW" && alerts[1]["severity"] == "info");
  CHECK(alerts[1]["target"] == "trajectory");

  const auto more = vz::dashboard::frame(
    1100.0, r, Plan{-6.0, {}, {Warning::AEB, Warning::RLDW}}, test_H(), tracker);
  const json a2 = by_type(more, "alerts")["alerts"];
  CHECK(a2[0]["code"] == "AEB" && a2[0]["severity"] == "critical" && a2[0]["target"] == "lead");
  CHECK(a2[1]["code"] == "RLDW" && a2[1]["severity"] == "info");
  CHECK(a2[1]["target"] == "trajectory");

  r.lateral.path_valid = false;  // a failed fit clears the ribbon on the page
  const auto lost = vz::dashboard::frame(1200.0, r, Plan{0.0, {}, {}}, test_H(), tracker);
  CHECK(by_type(lost, "trajectory")["pts"].empty());
  CHECK(by_type(lost, "alerts")["alerts"].empty());
}

void test_hello_status()
{
  const json h = json::parse(vz::dashboard::hello(0.0, "RTX 4070"));
  CHECK(h["type"] == "hello" && h["schema"] == 1 && h["roles"] == json::array({"stack"}));
  CHECK(h["name"] == "VisionPilot" && h["device"] == "RTX 4070");
  CHECK(!json::parse(vz::dashboard::hello(0.0, "")).contains("device"));
  const json s = json::parse(vz::dashboard::status(5.0, sample_frame(), "", 12.0));
  CHECK(s["mode"] == "active" && s["latency_ms"] == 24.7 && s["speed_limit_mps"] == 12.0);
  CHECK(s["stages"]["AutoSteer"] == 7.2 && !s.contains("device"));
}

// A client that connects after the last publish (a page reload while the loop is stalled) gets
// `hello` and then the current frame, without waiting for another publish.
void test_late_client_gets_current_frame()
{
  vz::DashboardServer server(18093, "test");
  server.publish({"X"});
  std::promise<std::string> got;
  auto fut = got.get_future();
  std::thread([&got] {
    try {
      boost::asio::io_context ioc;
      boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws(ioc);
      ws.next_layer().connect({boost::asio::ip::make_address("127.0.0.1"), 18093});
      ws.handshake("127.0.0.1", "/");
      boost::beast::flat_buffer b;
      ws.read(b);  // hello
      b.clear();
      ws.read(b);
      got.set_value(boost::beast::buffers_to_string(b.data()));
    } catch (const std::exception & e) {
      got.set_value(e.what());
    }
  }).detach();
  CHECK(fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
  CHECK(fut.get() == "X");
}

// One message of every type, for `python3 -m tools.schema` in adas-dashboard.
void write_jsonl(const char * path)
{
  vz::ObjectTracker tracker;
  std::ofstream out(path);
  out << vz::dashboard::hello(0.0, "test") << "\n";
  for (const auto & m : vz::dashboard::frame(
         1.0, sample_frame(), Plan{-4.0, {}, {Warning::FCW, Warning::RLDW}}, test_H(), tracker)) {
    out << m << "\n";
  }
  out << vz::dashboard::status(1.0, sample_frame(), "test", 12.0) << "\n";
}

// Publishes the sample frame at 100 Hz, ten times the bench rate, for `seconds`. Fails when one
// publish() took 5 ms or more, which means a client was able to stall the inference loop.
int serve(uint16_t port, int seconds)
{
  vz::DashboardServer server(port, "test");
  vz::ObjectTracker tracker;
  const vm::InferenceFrameResult r = sample_frame();
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  double worst_ms = 0.0;
  int n = 0;
  while (std::chrono::steady_clock::now() < end) {
    const double t = vz::now_ms();
    auto msgs = vz::dashboard::frame(t, r, Plan{0.0, {}, {}}, test_H(), tracker);
    if (n % 100 == 0) msgs.push_back(vz::dashboard::status(t, r, "test", 12.0));
    const auto p0 = std::chrono::steady_clock::now();
    server.publish(std::move(msgs));
    const std::chrono::duration<double, std::milli> took = std::chrono::steady_clock::now() - p0;
    worst_ms = std::max(worst_ms, took.count());
    ++n;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::printf("SERVE_DONE frames=%d worst_publish_ms=%.3f\n", n, worst_ms);
  return worst_ms < 5.0 ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 4 && std::string(argv[1]) == "--serve") {
    return serve(static_cast<uint16_t>(std::atoi(argv[2])), std::atoi(argv[3]));
  }
  test_project_box();
  test_tracker();
  test_frame();
  test_hello_status();
  test_late_client_gets_current_frame();
  if (argc == 2) write_jsonl(argv[1]);
  std::printf("TEST_PASS test_dashboard\n");
  return 0;
}
