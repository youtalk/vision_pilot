#!/usr/bin/env python3
"""Compress the CARLA camera before it crosses the bench LAN (gate D5).

  jpeg_bridge.py [--in /carla/hero/main_cam/image] [--out .../image/compressed]
                 [--quality 85]

CARLA publishes 1280x720 bgra8, which is 3.69 MB per sample. At 10 Hz that is
297 Mbit/s, and each sample fragments into about 2800 datagrams. Published
RELIABLE with KEEP_LAST(1), a sample that loses one fragment is retransmitted
while the next sample is already superseding it, so the X5H board reassembled
none of them and VisionPilot sat at zero frames (board 2, rx_drop=39454 with
rx_errs=0, 2026-09-18). Raising net.core.rmem_max to 16 MiB and adding
SocketReceiveBufferSize both failed to help, because the wall is the byte
count, not the socket.

This runs on the bench host, where the raw topic never leaves the machine.
The board subscribes to the compressed topic and turns it back into
sensor_msgs/Image with image_transport's own republish, so nothing on the
board needs new code.

Prints JPEG_BRIDGE ready in=<topic> out=<topic> quality=<q> then
JPEG_BRIDGE n=<frames> mean_kib=<size> every hundred frames.
"""
import argparse
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image

from lum_stats import from_image


def to_bgr(data, height, width, encoding):
    """The encodings CARLA's RGB camera actually emits, as BGR for cv2.

    from_image owns the encoding table and raises on an unknown encoding
    rather than reinterpreting it: reshaping bgra8 bytes as three channels
    produces a plausible-looking garbled frame, which is far worse to debug
    than a refusal. cv2 wants contiguous BGR, so copy the reversed view.
    """
    return np.ascontiguousarray(from_image(data, width, height, encoding)[..., ::-1])


class JpegBridge(Node):
    def __init__(self, topic_in, topic_out, quality):
        super().__init__("jpeg_bridge")
        self.quality = quality
        self.n = 0
        self.total_bytes = 0
        # Match the CARLA publisher: RELIABLE, KEEP_LAST(1). The subscription
        # is on the loopback side, where the raw size costs nothing.
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(CompressedImage, topic_out, qos)
        self.create_subscription(Image, topic_in, self.on_image, qos)

    def on_image(self, m):
        try:
            frame = to_bgr(bytes(m.data), m.height, m.width, m.encoding)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        ok, buf = cv2.imencode(".jpg", frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            self.get_logger().error("cv2.imencode failed")
            return
        out = CompressedImage()
        out.header = m.header
        out.format = "jpeg"
        out.data = buf.tobytes()
        self.pub.publish(out)
        self.n += 1
        self.total_bytes += len(out.data)
        if self.n % 100 == 0:
            print(f"JPEG_BRIDGE n={self.n} "
                  f"mean_kib={self.total_bytes / self.n / 1024:.1f}", flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--in", dest="topic_in", default="/carla/hero/main_cam/image")
    p.add_argument("--out", dest="topic_out",
                   default="/carla/hero/main_cam/image/compressed")
    p.add_argument("--quality", type=int, default=85)
    a = p.parse_args()
    if not 1 <= a.quality <= 100:
        print("JPEG_BRIDGE_FAIL reason=bad_quality", flush=True)
        return 1
    rclpy.init()
    node = JpegBridge(a.topic_in, a.topic_out, a.quality)
    print(f"JPEG_BRIDGE ready in={a.topic_in} out={a.topic_out} "
          f"quality={a.quality}", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
