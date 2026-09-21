#!/usr/bin/env python3
"""1 Hz snapshots of /carla/hero/main_cam/image as PPM, evidence for gate D6.

Adopted from openadkit-e2e's carla_bridge.py 1 Hz JPEG snapshots; PPM here
because the container has numpy and no image library.
  snap.py <out-dir> [--seconds 60]
Prints SNAP n=<frames written>.
"""
import argparse
import os
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lum_stats import from_image, write_ppm


class Snap(Node):
    def __init__(self, out):
        super().__init__("snap"); self.out = out; self.n = 0; self.last_t = 0.0
        self.create_subscription(Image, "/carla/hero/main_cam/image", self.on_image, 1)

    def on_image(self, m):
        now = time.time()
        if now - self.last_t < 1.0:
            return
        self.last_t = now
        write_ppm(os.path.join(self.out, f"{now:.1f}.ppm"), from_image(bytes(m.data), m.width, m.height, m.encoding))
        self.n += 1


def main():
    p = argparse.ArgumentParser(); p.add_argument("out"); p.add_argument("--seconds", type=float, default=60.0)
    a = p.parse_args(); os.makedirs(a.out, exist_ok=True)
    rclpy.init(); s = Snap(a.out); end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(s, timeout_sec=0.1)
    print(f"SNAP n={s.n}"); return 0


if __name__ == "__main__":
    sys.exit(main())
