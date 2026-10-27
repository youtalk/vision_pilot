#ifndef VISUALIZATION__KMS_DISPLAY_HPP_
#define VISUALIZATION__KMS_DISPLAY_HPP_

#include <opencv2/core/mat.hpp>
#include <visualization/kms_gate.hpp>
#include <visualization/visual_interface.hpp>

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
// The display is never allowed to stop VisionPilot, and it never costs a timed
// frame. The sink makes one attempt, on the first non-empty frame, which the
// S-CORE hook does not time (KmsGate). With no connected monitor then, or after
// a failed start, a pipeline error or a change of frame size, the sink is inert
// until the process ends: render_frame() drops every frame and returns true,
// and wants_frames() turns false so VisionPilot stops composing the HUD. The
// pipeline is torn down only in stop() and the destructor, so the vsp1 release
// path never runs in a running system. A monitor plugged in later shows the
// HUD after the next VisionPilot restart. A monitor that sleeps or is
// re-plugged while the sink runs does not come back: its wake is a long HPD,
// the sink holds DRM master, and the kernel console defers the hotplug modeset
// that would train the link again. appsrc keeps at most two frames and drops
// the oldest, so a slow display drops frames instead of stalling the inference
// loop.
class KmsDisplay : public VisualInterface
{
public:
  explicit KmsDisplay(std::string driver, std::string drm_dir = "/sys/class/drm");
  ~KmsDisplay() override;

  KmsDisplay(const KmsDisplay &) = delete;
  KmsDisplay & operator=(const KmsDisplay &) = delete;

  bool render_frame(const cv::Mat & display_frame) override;
  bool stop() override;
  bool wants_frames() const override { return gate_.wants_frames(); }

private:
  bool start(int frame_w, int frame_h);
  bool pipeline_failed();
  void teardown();

  std::string driver_;
  std::string drm_dir_;
  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  KmsGate gate_;
};

}  // namespace visualization

#endif  // VISUALIZATION__KMS_DISPLAY_HPP_
