#!/usr/bin/env python3
"""Mean luminance of /carla/hero/main_cam/image over N frames, plus one saved frame.

Runs inside visionpilot:si (rclpy, numpy) on the bench DDS configuration.
  luminance_probe.py [--frames 50] [--ppm /tmp/frame.ppm] [--timeout 120]
Prints LUMINANCE mean=<f> p5=<f> p95=<f> frames=<n> encoding=<enc> ppm=<path>
or LUMINANCE_FAIL reason=<slug>. p5/p95 are the extremes over all frames.
"""
import argparse
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lum_stats import from_image, luminance, write_ppm


class Probe(Node):
    def __init__(self):
        super().__init__("luminance_probe")
        self.stats = []; self.last = None
        self.create_subscription(Image, "/carla/hero/main_cam/image", self.on_image, 10)

    def on_image(self, m):
        rgb = from_image(bytes(m.data), m.width, m.height, m.encoding)
        self.stats.append(luminance(rgb)); self.last = (rgb, m.encoding)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--frames", type=int, default=50); p.add_argument("--ppm", default="/tmp/frame.ppm")
    p.add_argument("--timeout", type=float, default=120.0)
    a = p.parse_args()
    rclpy.init(); n = Probe(); end = time.time() + a.timeout
    while len(n.stats) < a.frames and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.stats:
        print("LUMINANCE_FAIL reason=no_frames"); return 1
    write_ppm(a.ppm, n.last[0])
    mean = sum(s[0] for s in n.stats) / len(n.stats)
    print(f"LUMINANCE mean={mean:.1f} p5={min(s[1] for s in n.stats):.1f} "
          f"p95={max(s[2] for s in n.stats):.1f} frames={len(n.stats)} encoding={n.last[1]} ppm={a.ppm}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
