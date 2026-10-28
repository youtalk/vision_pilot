// The KMS sink decides once whether to show the HUD, and never restarts its
// pipeline. KmsGate holds that decision. KmsDisplay makes it at construction,
// before the frame loop, because the pipeline start takes about 0.5 s on the
// X5H and the S-CORE hook can already time the first frame. The KmsDisplay
// cases stop before GStreamer, because the connector read fails first.
#include <gtest/gtest.h>
#include <visualization/kms_display.hpp>
#include <visualization/kms_gate.hpp>

#include <filesystem>
#include <fstream>
#include <string>

#include <unistd.h>

namespace fs = std::filesystem;
using visualization::KmsAction;
using visualization::KmsDisplay;
using visualization::KmsGate;

static KmsGate running_gate()
{
  KmsGate g;
  EXPECT_EQ(g.on_frame(false, 1024, 512), KmsAction::Start);
  g.start_result(true);
  return g;
}

TEST(KmsGate, EmptyFramesDoNotUseTheAttempt)
{
  KmsGate g;
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(g.on_frame(true, 0, 0), KmsAction::Skip);
    EXPECT_TRUE(g.wants_frames());
  }
  EXPECT_EQ(g.on_frame(false, 1024, 512), KmsAction::Start);
}

TEST(KmsGate, FailedStartMakesInert)
{
  KmsGate g;
  ASSERT_EQ(g.on_frame(false, 1024, 512), KmsAction::Start);
  g.start_result(false);
  for (int i = 0; i < 10; ++i) EXPECT_EQ(g.on_frame(false, 1024, 512), KmsAction::Skip);
  EXPECT_FALSE(g.wants_frames());
}

TEST(KmsGate, RunningPushes)
{
  KmsGate g = running_gate();
  EXPECT_EQ(g.on_frame(false, 1024, 512), KmsAction::Push);
  EXPECT_TRUE(g.wants_frames());
}

TEST(KmsGate, ErrorMakesInert)
{
  KmsGate g = running_gate();
  g.pipeline_error();
  EXPECT_EQ(g.on_frame(false, 1024, 512), KmsAction::Skip);
  EXPECT_FALSE(g.wants_frames());
}

TEST(KmsGate, SizeChangeMakesInert)
{
  KmsGate g = running_gate();
  EXPECT_EQ(g.on_frame(false, 1280, 720), KmsAction::Skip);
  EXPECT_EQ(g.on_frame(false, 1024, 512), KmsAction::Skip);
  EXPECT_EQ(g.on_frame(false, 1280, 720), KmsAction::Skip);
  EXPECT_FALSE(g.wants_frames());
}

TEST(KmsDisplay, NoDrmDirMakesInertAtConstruction)
{
  KmsDisplay d("rcar-vcon", 1024, 512, "/nonexistent");
  EXPECT_FALSE(d.wants_frames());
  EXPECT_TRUE(d.render_frame(cv::Mat(512, 1024, CV_8UC3)));
  EXPECT_FALSE(d.wants_frames());
}

TEST(KmsDisplay, NoStartWhenConnectorDisabled)
{
  const fs::path drm = fs::temp_directory_path() / ("kms_gate_test_" + std::to_string(::getpid()));
  const fs::path c = drm / "card0-DP-1";
  fs::create_directories(c);
  std::ofstream(c / "status") << "connected\n";
  std::ofstream(c / "enabled") << "disabled\n";
  std::ofstream(c / "modes") << "3440x1440\n";

  KmsDisplay d("rcar-vcon", 1024, 512, drm.string());
  EXPECT_FALSE(d.wants_frames());
  // The console lights the connector later: the sink does not try again.
  std::ofstream(c / "enabled") << "enabled\n";
  EXPECT_TRUE(d.render_frame(cv::Mat(512, 1024, CV_8UC3)));
  EXPECT_FALSE(d.wants_frames());
  fs::remove_all(drm);
}
