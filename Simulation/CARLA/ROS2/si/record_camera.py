#!/usr/bin/env python3
"""Record the compressed camera the board receives, stream 2 of the demo recording.

  record_camera.py <out-dir> [--topic /carla/hero/main_cam/image/compressed]
                   [--seconds 120]

Writes every sample of jpeg_bridge.py's output topic to disk byte for byte. The
pane answers "what was VisionPilot on the board looking at", and the board
already subscribes to this exact topic, so recording it needs no decode and no
re-encode: about 160 KB a sample instead of the 3.69 MB raw frame that the whole
bridge exists to keep off the bench LAN. Decoding here would also make the pane
a picture of this host's OpenCV rather than of what crossed the wire.

jpeg_bridge.py must already be running on this host. Nothing here starts it,
and nothing here can tell "the bridge is not running" apart from "the camera is
not publishing", so record-demo.sh checks the frame count instead.

bench_time in index.csv is the sample's header stamp when that stamp is a wall
clock, and the arrival time on this host otherwise. This bench runs the CARLA
bridge under use_sim_time, so in practice it is the arrival time, and the run
says so with a REC_CAM clock=arrival line.

Prints REC_CAM ready, then REC_CAM n=<frames> dir=<dir>, or
REC_CAM_FAIL reason=<slug>.
"""
import argparse
import os
import signal
import sys
import time

import rclpy
from demo_streams import open_index, stamp_seconds, write_row
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def _on_sigterm(signum, frame):
    """record-demo.sh stops this container with docker stop, which is a SIGTERM
    to the process the image's command exec'd. Turning it into SystemExit is
    what lets the final REC_CAM line out; without it the frame count of a
    perfectly good recording is never printed."""
    raise SystemExit(0)


class RecordCamera(Node):
    def __init__(self, out, topic):
        super().__init__("record_camera")
        self.out = out
        self.n = 0
        self.sim_time = False
        self.index = open_index(os.path.join(out, "index.csv"))
        # Reliability must match jpeg_bridge.py's publisher, which is RELIABLE
        # with KEEP_LAST(1). This is the one QoS field that silently costs the
        # whole stream if it is guessed: an incompatible pair matches nothing
        # and neither side reports anything, the recorder just sits at n=0.
        # Depth is not part of that compatibility check, so keep a deeper queue
        # than the publisher's one sample: a disk write that stalls for a couple
        # of frame intervals must not punch a hole in the reel.
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=30,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(CompressedImage, topic, self.on_image, qos)

    def on_image(self, m):
        # Bench-measured 2026-09-18: this bench runs the CARLA bridge under
        # use_sim_time, so the header stamp is simulation time, a few thousand
        # seconds from zero, and not a clock any other stream shares. Refusing
        # it recorded n=0 and cost a recording run. The arrival time on this
        # host is the bench clock the reel aligns on. It differs from the
        # board's own receive time by one LAN hop, which is far inside the one
        # rendered frame the reel claims. The header still wins when it IS an
        # epoch, because then it is the better of the two.
        try:
            t = stamp_seconds(m.header.stamp.sec, m.header.stamp.nanosec)
        except ValueError:
            t = time.time()
            self.sim_time = True
        name = f"{self.n:06d}.jpg"
        with open(os.path.join(self.out, name), "wb") as f:
            f.write(bytes(m.data))       # unchanged: the JPEG the board received
        write_row(self.index, name, t)
        self.n += 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("out")
    p.add_argument("--topic", default="/carla/hero/main_cam/image/compressed")
    p.add_argument("--seconds", type=float, default=120.0)
    a = p.parse_args()
    # Same reason as demo_cam.py: a non-positive --seconds would exit 0 with an
    # empty directory, which reads as a broken camera rather than a bad argument.
    if a.seconds <= 0:
        print("REC_CAM_FAIL reason=bad_seconds", flush=True)
        return 1
    try:
        os.makedirs(a.out, exist_ok=True)
    except OSError:
        print("REC_CAM_FAIL reason=bad_out", flush=True)
        return 1
    rclpy.init()
    node = RecordCamera(a.out, a.topic)
    signal.signal(signal.SIGTERM, _on_sigterm)
    print("REC_CAM ready", flush=True)
    end = time.time() + a.seconds
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.index.close()
    if node.sim_time:
        # Not a failure, but the reader of this run deserves to know which
        # clock the index carries.
        print("REC_CAM clock=arrival reason=sim_time_header", flush=True)
    print(f"REC_CAM n={node.n} dir={a.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
