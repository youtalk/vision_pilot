#!/usr/bin/env python3
"""Stand-in for the CR52 during the bench rehearsal of gate D6 (no board).

Publishes the same -3 m/s^2 ramp the firmware's StopProfile produces, on the
two topics the board's bridge and restamp would carry it on, every 150 ms.
  si_standin.py --v0 <m/s> [--start-at <epoch>] [--period 0.15] [--decel 3.0] [--hold 3.0]
Prints SI_STANDIN start=<epoch> v0=<v0> then SI_STANDIN done n=<samples>.

--start-at is si_fault.sh's idiom: come up, join the domain, and then wait for
a FUTURE instant before the first sample. Without it the caller has to start
this process at the fault instant, and a container cold start plus DDS
discovery puts about 750 ms of harness in front of the ramp. At 12 m/s that
is 9 m of the stop distance the gate measures, which is harness overhead
reported as if the Safety Island were slow.
"""
import argparse
import sys
import time

TOPICS = ("/control/trajectory_follower/control_cmd_raw", "/control/trajectory_follower/control_cmd")


def ramp(v0, t, decel=3.0):
    return max(0.0, v0 - decel * t)


def main():
    # Deferred so `import si_standin` (test_si_standin.py, testing ramp() alone)
    # works on a bench PC with no ROS install; the CLI still needs rclpy at run time.
    import rclpy
    from rclpy.node import Node
    from autoware_control_msgs.msg import Control

    p = argparse.ArgumentParser(); p.add_argument("--v0", type=float, required=True)
    p.add_argument("--period", type=float, default=0.15); p.add_argument("--decel", type=float, default=3.0)
    p.add_argument("--hold", type=float, default=3.0); p.add_argument("--start-at", type=float)
    a = p.parse_args()
    rclpy.init(); node = Node("si_standin")
    pubs = [node.create_publisher(Control, t, 1) for t in TOPICS]
    # Publishers exist, so discovery runs during this wait rather than inside
    # the measurement. Not publishing before start_at also matters: the gate
    # fails a run with reason=cmd_before_fault.
    if a.start_at is not None:
        wait = a.start_at - time.time()
        if wait > 0:
            time.sleep(wait)
    t0 = a.start_at if a.start_at is not None else time.time(); n = 0
    print(f"SI_STANDIN start={t0:.3f} v0={a.v0}", flush=True)
    while True:
        t = time.time() - t0; v = ramp(a.v0, t, a.decel)
        m = Control(); m.stamp = node.get_clock().now().to_msg()
        m.longitudinal.velocity = v; m.longitudinal.acceleration = -a.decel
        m.lateral.steering_tire_angle = 0.0
        for pub in pubs: pub.publish(m)
        n += 1
        if v == 0.0 and t >= a.v0 / a.decel + a.hold:
            break
        time.sleep(a.period)
    print(f"SI_STANDIN done n={n}", flush=True); return 0


if __name__ == "__main__":
    sys.exit(main())
