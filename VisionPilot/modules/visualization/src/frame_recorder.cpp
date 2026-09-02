#include <opencv2/imgcodecs.hpp>
#include <visualization/frame_recorder.hpp>

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <utility>
#include <vector>

namespace visualization
{

namespace fs = std::filesystem;

// Lossless, and level 1 rather than the default 3: the pipeline is the thing
// being measured, so the sink must not become part of the per-frame cost.
static const std::vector<int> kPngParams = {cv::IMWRITE_PNG_COMPRESSION, 1};

FrameRecorder::FrameRecorder(std::string dir) : dir_(std::move(dir))
{
  std::error_code ec;
  fs::create_directories(dir_, ec);
  if (ec) {
    std::cerr << "[Recorder] cannot create " << dir_ << ": " << ec.message() << "\n";
    failed_ = true;
  }
}

bool FrameRecorder::render_frame(const cv::Mat & display_frame)
{
  if (failed_) return false;

  if (display_frame.empty()) {
    std::cerr << "[Recorder] refusing to write an empty frame at index " << index_ << "\n";
    failed_ = true;
    return false;
  }

  char name[32];
  std::snprintf(name, sizeof(name), "frame_%06zu.png", index_);
  const std::string path = (fs::path(dir_) / name).string();

  if (!cv::imwrite(path, display_frame, kPngParams)) {
    std::cerr << "[Recorder] write failed: " << path << "\n";
    failed_ = true;
    return false;
  }

  ++index_;
  return true;
}

bool FrameRecorder::stop()
{
  // The frame count is the run's own record of how much of the clip was
  // rendered, so print it even when the run has already failed.
  std::cout << "[Recorder] " << index_ << " frames written to " << dir_ << "\n";
  return !failed_;
}

}  // namespace visualization
