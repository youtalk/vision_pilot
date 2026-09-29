#ifndef VISUALIZATION__KMS_DISPLAY_HPP_
#define VISUALIZATION__KMS_DISPLAY_HPP_

#include <opencv2/core/mat.hpp>
#include <visualization/visual_interface.hpp>

#include <chrono>
#include <string>

typedef struct _GstElement GstElement;

namespace visualization
{

// Sink that shows every composed display frame on a DisplayPort monitor
// through the kernel's DRM driver: appsrc ! videoconvert ! videoscale !
// kmssink. The HUD is scaled to fit the connector's preferred mode and drawn
// on a plane over whatever the CRTC already shows (the frame buffer console).
//
// It never switches the display mode. On the X5H BSP kernel, releasing a mode
// that kmssink set panics the board through a NULL callback in vsp1 at the
// next frame-end interrupt.
//
// The display is never allowed to stop VisionPilot. With no connected monitor,
// or after any pipeline error, render_frame() drops the frame and returns true,
// and the sink tries again at most once every kRetry. A monitor that is
// unplugged when VisionPilot starts therefore shows the HUD a few seconds after
// it is plugged in. A monitor that sleeps or is re-plugged while the sink runs
// does not come back: its wake is a long HPD, the sink holds DRM master, and
// the kernel console defers the hotplug modeset that would train the link
// again. appsrc keeps at most two frames and drops the oldest, so a slow
// display drops frames instead of stalling the inference loop.
class KmsDisplay : public VisualInterface
{
public:
  explicit KmsDisplay(std::string driver, std::string drm_dir = "/sys/class/drm");
  ~KmsDisplay() override;

  KmsDisplay(const KmsDisplay &) = delete;
  KmsDisplay & operator=(const KmsDisplay &) = delete;

  bool render_frame(const cv::Mat & display_frame) override;
  bool stop() override;

private:
  static constexpr std::chrono::seconds kRetry{5};

  bool start(int frame_w, int frame_h);
  bool pipeline_failed();
  void teardown();

  std::string driver_;
  std::string drm_dir_;
  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  int width_ = 0;
  int height_ = 0;
  std::chrono::steady_clock::time_point next_try_{};
  bool warned_ = false;
};

}  // namespace visualization

#endif  // VISUALIZATION__KMS_DISPLAY_HPP_
