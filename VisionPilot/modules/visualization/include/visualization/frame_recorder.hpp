#ifndef VISIONPILOT_VISUALIZATION_FRAME_RECORDER_HPP
#define VISIONPILOT_VISUALIZATION_FRAME_RECORDER_HPP

#include <opencv2/core.hpp>

#include <cstddef>
#include <string>
#include <visualization/visual_interface.hpp>

namespace visualization
{

// Sink that writes every composed display frame to a numbered PNG in a
// directory, for assembling a video offline.
//
// LocalDisplay needs a window and WebRTCStreamer paces and re-encodes frames to
// VP8 in real time, so both are free to drop a frame under load. This sink
// never does: one call to render_frame() produces exactly one lossless file, so
// the resulting sequence is a faithful record of what the pipeline rendered and
// is comparable frame-for-frame across runs.
class FrameRecorder : public VisualInterface
{
public:
  explicit FrameRecorder(std::string dir);

  FrameRecorder(const FrameRecorder &) = delete;
  FrameRecorder & operator=(const FrameRecorder &) = delete;

  bool render_frame(const cv::Mat & display_frame) override;
  bool stop() override;

private:
  std::string dir_;
  std::size_t index_ = 0;
  bool failed_ = false;
};

}  // namespace visualization

#endif  // VISIONPILOT_VISUALIZATION_FRAME_RECORDER_HPP
