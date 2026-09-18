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

first_si_ack_ms is the first /carla/hero/ackermann_control_cmd at or below
-2.5 m/s^2 after the fault, which is the bench arbiter forwarding the CR52
ramp instead of VisionPilot's pair (arbiter.py choose_source).

The decision itself is stop_metrics.verdict(), which is pure and tested; this
module only collects the samples and prints what it returns.
"""
import argparse
import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_control_msgs.msg import Control
from ackermann_msgs.msg import AckermannDriveStamped
from stop_metrics import verdict


class Gate(Node):
    def __init__(self, t_fault):
        super().__init__("si_stop_gate")
        self.t_fault = t_fault; self.raw_stamps = []; self.rows = []; self.ack = []
        self.create_subscription(Control, "/control/trajectory_follower/control_cmd_raw", self.on_raw, 10)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 10)
        self.create_subscription(AckermannDriveStamped, "/carla/hero/ackermann_control_cmd", self.on_ack, 10)

    def on_raw(self, _msg):
        self.raw_stamps.append(time.time())

    def on_odom(self, m):
        self.rows.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x))

    def on_ack(self, m):
        self.ack.append((time.time(), float(m.drive.acceleration)))


def main():
    p = argparse.ArgumentParser(); p.add_argument("--fault-at", type=float, required=True)
    p.add_argument("--window", type=float, default=30.0); p.add_argument("--max-latency-ms", type=float, default=200.0)
    # The marker line carries a mean deceleration, which cannot tell a vehicle
    # that brakes weakly from the first instant apart from one whose braking
    # builds up over seconds. The two have different causes, so write the
    # samples out and let the shape of the speed trace answer it.
    p.add_argument("--trace", help="write the odometry and command samples to this CSV")
    a = p.parse_args()
    rclpy.init(); g = Gate(a.fault_at)
    end = a.fault_at + a.window
    while time.time() < end:
        rclpy.spin_once(g, timeout_sec=0.1)
    line = verdict(a.fault_at, g.raw_stamps, g.ack, g.rows, a.max_latency_ms)
    print(line)
    # After the marker, never before it: an unwritable --trace path must not
    # turn a measured verdict into run-d6.sh's gate_no_verdict.
    if a.trace:
        with open(a.trace, "w") as f:
            f.write("kind,t_rel_s,x,y,value\n")
            for t, x, y, v in g.rows:
                f.write(f"odom,{t - a.fault_at:.3f},{x:.3f},{y:.3f},{v:.3f}\n")
            for t, acc in g.ack:
                f.write(f"ack_accel,{t - a.fault_at:.3f},,,{acc:.3f}\n")
            for t in g.raw_stamps:
                f.write(f"cr52_cmd,{t - a.fault_at:.3f},,,\n")
    return 0 if line.startswith("SI_STOP_PASS") else 1


if __name__ == "__main__":
    sys.exit(main())
