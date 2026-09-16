#!/usr/bin/env python3
"""Gate D6, rog-amd half. Records when the first CR52-authored control_cmd
arrives after the fault and how far the ego travels until it stops.

  si_stop_gate.py --fault-at <epoch-seconds> [--window 30] [--max-latency-ms 200]

control_cmd_raw is the bridge's copy of the CR52's sample on domain 1: the
bridge is its only publisher, so a sample there was made on the CR52 (the
X5H_STACK_PASS argument in openadkit/component-stack.md). Ego position and
speed come from the ego-state publisher's /localization/kinematic_state.

The latency budget depends on which si_fault.sh route triggered the fault,
so --max-latency-ms defaults to 200 but the caller must override it for the
kill route:
  channel  200 ms. The rpmsg-si channel latches the fault directly, so the
           controller needs no staleness detection.
  kill     700 ms. Stopping x5h-vp.service is noticed only through heartbeat
           staleness: the firmware trips 0.5 s after the last heartbeat and
           then adds one 0.15 s control cycle. A kill-route run must be
           gated with --max-latency-ms 700, or a correct run reads as a
           failure.
"""
import argparse
import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_control_msgs.msg import Control
from stop_metrics import first_after, stop_distance


class Gate(Node):
    def __init__(self, t_fault):
        super().__init__("si_stop_gate")
        self.t_fault = t_fault; self.raw_stamps = []; self.rows = []
        self.create_subscription(Control, "/control/trajectory_follower/control_cmd_raw", self.on_raw, 10)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 10)

    def on_raw(self, _msg):
        self.raw_stamps.append(time.time())

    def on_odom(self, m):
        self.rows.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x))


def main():
    p = argparse.ArgumentParser(); p.add_argument("--fault-at", type=float, required=True)
    p.add_argument("--window", type=float, default=30.0); p.add_argument("--max-latency-ms", type=float, default=200.0)
    a = p.parse_args()
    rclpy.init(); g = Gate(a.fault_at)
    end = a.fault_at + a.window
    while time.time() < end:
        rclpy.spin_once(g, timeout_sec=0.1)
    t_first = first_after(a.fault_at, g.raw_stamps)
    if t_first is None:
        print("SI_STOP_FAIL reason=no_cr52_cmd"); return 1
    latency_ms = (t_first - a.fault_at) * 1000.0
    if latency_ms > a.max_latency_ms:
        print(f"SI_STOP_FAIL reason=cmd_late first_cr52_cmd_ms={latency_ms:.0f}"); return 1
    d, t_stop = stop_distance([r for r in g.rows if r[0] >= a.fault_at], 0.05)
    if d is None:
        print(f"SI_STOP_FAIL reason=no_stop first_cr52_cmd_ms={latency_ms:.0f}"); return 1
    print(f"SI_STOP_PASS first_cr52_cmd_ms={latency_ms:.0f} stop_distance_m={d:.2f} stop_s={t_stop - a.fault_at:.2f}"); return 0


if __name__ == "__main__":
    sys.exit(main())
