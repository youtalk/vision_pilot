#ifndef VISUALIZATION__KMS_LAYOUT_HPP_
#define VISUALIZATION__KMS_LAYOUT_HPP_

#include <string>

namespace visualization
{

struct KmsRect
{
  int x = 0;
  int y = 0;
  int w = 0;
  int h = 0;
};

// The frame at the largest whole-number scale that fits the screen, centred.
// A frame larger than the screen is scaled down to the largest rectangle with
// its aspect ratio instead. Whole numbers only, because the CPU scaler's cost
// grows with the output pixels and the HUD shares the CPU with a timed frame
// loop. Width and height are rounded down to even. Any non-positive input
// gives an empty rectangle.
KmsRect fit_centered(int screen_w, int screen_h, int frame_w, int frame_h);

// The preferred mode of the first connected DisplayPort connector under
// drm_dir (normally /sys/class/drm): a "card*-DP-*" entry whose status reads
// "connected" and whose enabled reads "enabled", first line of its modes
// file, "<w>x<h>". False when no such connector exists or the line does not
// parse; w and h are then unchanged. "enabled" means a CRTC already drives
// the connector (the kernel console lit it). Without one, kmssink sets a
// mode itself, and releasing that mode panics the X5H BSP kernel.
bool read_preferred_mode(const std::string & drm_dir, int & w, int & h);

}  // namespace visualization

#endif  // VISUALIZATION__KMS_LAYOUT_HPP_
