#include <visualization/kms_display.hpp>
#include <visualization/kms_layout.hpp>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <cstdio>
#include <iostream>
#include <utility>

namespace visualization
{

KmsDisplay::KmsDisplay(std::string driver, std::string drm_dir)
: driver_(std::move(driver)), drm_dir_(std::move(drm_dir))
{
}

KmsDisplay::~KmsDisplay()
{
  teardown();
}

bool KmsDisplay::start(int frame_w, int frame_h)
{
  int mode_w = 0, mode_h = 0;
  if (!read_preferred_mode(drm_dir_, mode_w, mode_h)) {
    if (!warned_)
      std::cerr << "[KmsDisplay] no connected DisplayPort monitor under " << drm_dir_
                << "; retrying\n";
    warned_ = true;
    return false;
  }
  const KmsRect r = fit_centered(mode_w, mode_h, frame_w, frame_h);
  if (r.w <= 0 || r.h <= 0) return false;

  if (!gst_is_initialized()) gst_init(nullptr, nullptr);
  // ponytail: videoscale scales on the CPU; move it to the VSP (kmssink
  // can-scale with a larger render rectangle) if phase 2 finds the cost.
  char desc[768];
  std::snprintf(
    desc, sizeof(desc),
    "appsrc name=src is-live=true do-timestamp=true format=time max-buffers=2 "
    "leaky-type=downstream "
    "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=0/1 "
    "! videoconvert ! videoscale ! video/x-raw,width=%d,height=%d "
    "! kmssink driver-name=%s sync=false render-rectangle=\"<%d,%d,%d,%d>\"",
    frame_w, frame_h, r.w, r.h, driver_.c_str(), r.x, r.y, r.w, r.h);
  GError * err = nullptr;
  pipeline_ = gst_parse_launch(desc, &err);
  if (err != nullptr) {
    std::cerr << "[KmsDisplay] pipeline rejected: " << err->message << "\n";
    g_error_free(err);
    teardown();
    return false;
  }
  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
  if (
    appsrc_ == nullptr ||
    gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    std::cerr << "[KmsDisplay] kmssink did not start (driver " << driver_ << "); retrying\n";
    teardown();
    return false;
  }
  width_ = frame_w;
  height_ = frame_h;
  warned_ = false;
  std::cout << "[KmsDisplay] " << frame_w << "x" << frame_h << " shown at " << r.w << "x" << r.h
            << "+" << r.x << "+" << r.y << " on a " << mode_w << "x" << mode_h << " mode\n";
  return true;
}

bool KmsDisplay::pipeline_failed()
{
  GstBus * bus = gst_element_get_bus(pipeline_);
  GstMessage * msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
  gst_object_unref(bus);
  if (msg == nullptr) return false;
  GError * err = nullptr;
  gchar * dbg = nullptr;
  gst_message_parse_error(msg, &err, &dbg);
  std::cerr << "[KmsDisplay] pipeline error: " << (err ? err->message : "unknown")
            << "; retrying\n";
  if (err) g_error_free(err);
  g_free(dbg);
  gst_message_unref(msg);
  return true;
}

bool KmsDisplay::render_frame(const cv::Mat & display_frame)
{
  if (display_frame.empty() || display_frame.type() != CV_8UC3) return true;
  if (
    pipeline_ != nullptr &&
    (display_frame.cols != width_ || display_frame.rows != height_ || pipeline_failed())) {
    teardown();
  }
  if (pipeline_ == nullptr) {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_try_) return true;
    next_try_ = now + kRetry;
    if (!start(display_frame.cols, display_frame.rows)) return true;
  }
  const cv::Mat frame = display_frame.isContinuous() ? display_frame : display_frame.clone();
  const gsize bytes = frame.total() * frame.elemSize();
  GstBuffer * buf = gst_buffer_new_allocate(nullptr, bytes, nullptr);
  gst_buffer_fill(buf, 0, frame.data, bytes);
  gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buf);  // takes ownership
  return true;
}

void KmsDisplay::teardown()
{
  if (pipeline_ != nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  if (appsrc_ != nullptr) {
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
  }
}

bool KmsDisplay::stop()
{
  teardown();
  return true;
}

}  // namespace visualization
