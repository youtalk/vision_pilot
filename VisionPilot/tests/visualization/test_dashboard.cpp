// Tests for the dashboard `stack` endpoint.
// Run: test_dashboard [OUT.jsonl]   or   test_dashboard --serve PORT SECONDS
#include <opencv2/core.hpp>
#include <visualization/box_projection.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

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
  CHECK(!vz::project_box(test_H(), 494, 200, 530, 250, 158.f, g));  // above it: behind the ego
  CHECK(!vz::project_box(cv::Mat(), 494, 250, 530, 276, 158.f, g));
}

}  // namespace

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;
  test_project_box();
  std::printf("TEST_PASS test_dashboard\n");
  return 0;
}
