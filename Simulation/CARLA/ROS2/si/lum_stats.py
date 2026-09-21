"""Luminance of a camera frame, no ROS. Run: python3 -m pytest test_lum_stats.py

Rec.601 luma of an RGB uint8 frame. The bench host's blown-out HUD is judged
against the workstation's 166-196/255 band (memory: carla-ue58-visionpilot-
closed-loop.md), so mean luminance on that 0-255 scale is the number.
"""
import numpy as np


def luminance(rgb):
    """rgb: uint8 array (h, w, 3). Returns (mean, p5, p95) on a 0-255 scale."""
    y = 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
    return float(y.mean()), float(np.percentile(y, 5)), float(np.percentile(y, 95))


def from_image(data, width, height, encoding):
    """sensor_msgs/Image bytes -> rgb (h, w, 3). CARLA's native ROS 2 camera is bgra8."""
    a = np.frombuffer(data, dtype=np.uint8)
    if encoding == "bgra8":
        return a.reshape(height, width, 4)[..., [2, 1, 0]]
    if encoding == "rgba8":
        return a.reshape(height, width, 4)[..., :3]
    if encoding == "rgb8":
        return a.reshape(height, width, 3)
    if encoding == "bgr8":
        return a.reshape(height, width, 3)[..., ::-1]
    raise ValueError("unsupported encoding %s" % encoding)


def write_ppm(path, rgb):
    """Binary PPM: no image library needed inside the container."""
    with open(path, "wb") as f:
        f.write(b"P6 %d %d 255\n" % (rgb.shape[1], rgb.shape[0]))
        f.write(np.ascontiguousarray(rgb).tobytes())
