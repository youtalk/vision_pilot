#!/usr/bin/env python3
"""Gate D4: VisionPilot drives one closed lap unaided with |cte| under the limit.

Samples the hero at 10 Hz through the client API (no ROS), like mpc_trace.py.
  lap_gate.py <seconds> [--max-cte 1.75] [--min-lap-m 500] [--stall-s 20] [--csv out.csv]
Prints one D4_LAP_PASS or D4_LAP_FAIL line and exits 0/1.
"""
import argparse
import csv
import sys
import time
import carla
from lap_geom import LapTracker, signed_cte

# Gate D4 is "one closed lap at 12 m/s", but the 12.0 lives only in
# visionpilot.carla.conf, which run-d4.sh delivers by bind mount. If that mount
# misses, VisionPilot drives the old 4.0 m/s, 3046 m still fits inside
# VP_SECONDS=900, the lap closes, no |cte| exceeds the limit, and the run used
# to print a clean D4_LAP_PASS with no speed field at all. The mean is over the
# gate's own samples rather than the wall clock, so the standstill before
# VisionPilot engages costs about 1 % on a 254 s lap.
MIN_MEAN_SPEED = 10.0


def find_hero(world):
    # config_carla.py holds the world in synchronous mode and drives the clock
    # itself (config_carla.py:237). A passive client gets no world snapshot until
    # it waits for one, and until then get_actors() returns an empty list rather
    # than raising, so the caller's "while hero is None" loop never ends.
    world.wait_for_tick(seconds=30.0)
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") == "hero":
            return a
    return None


def main():
    p = argparse.ArgumentParser()
    # 1.75 is the gate's own limit; the old 1.0 default meant a hand-run gate
    # was stricter than the one run-d4.sh runs, and nothing said so.
    p.add_argument("seconds", type=float); p.add_argument("--max-cte", type=float, default=1.75)
    p.add_argument("--min-lap-m", type=float, default=500.0); p.add_argument("--stall-s", type=float, default=20.0)
    p.add_argument("--csv", default="/tmp/d4-lap.csv")
    a = p.parse_args()
    client = carla.Client("localhost", 2000); client.set_timeout(30.0)
    world = client.get_world(); cmap = world.get_map()
    hero = None
    while hero is None:
        hero = find_hero(world); time.sleep(0.5)
    loc0 = hero.get_transform().location
    lap = LapTracker(loc0.x, loc0.y, a.min_lap_m, 8.0)
    max_cte = 0.0; last_move = time.time(); t_end = time.time() + a.seconds; n = 0; speed_sum = 0.0
    with open(a.csv, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "x", "y", "speed", "cte", "travelled"])
        while time.time() < t_end:
            tf = hero.get_transform(); v = hero.get_velocity(); speed = (v.x ** 2 + v.y ** 2) ** 0.5
            wp = cmap.get_waypoint(tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            # Fail closed. A missing waypoint used to make cte NaN, and NaN is
            # silent in both tests that follow: abs(nan) > limit is False, so
            # the sample counted as in-lane, and max(max_cte, nan) leaves the
            # maximum untouched, so the PASS line said nothing about it.
            if wp is None:
                print(f"D4_LAP_FAIL reason=no_waypoint at_m={lap.travelled_m:.1f} max_cte_m={max_cte:.2f}"); return 1
            cte = signed_cte(wp.transform.location.x, wp.transform.location.y, wp.transform.rotation.yaw, tf.location.x, tf.location.y)
            n += 1; speed_sum += speed
            w.writerow([f"{time.time():.3f}", f"{tf.location.x:.3f}", f"{tf.location.y:.3f}", f"{speed:.3f}", f"{cte:.3f}", f"{lap.travelled_m:.1f}"])
            # |cte| is judged at every speed: gating it on speed > 0.5 left an
            # off-lane crawl unjudged for as long as it crawled.
            max_cte = max(max_cte, abs(cte))
            if abs(cte) > a.max_cte:
                print(f"D4_LAP_FAIL reason=lane_departure at_m={lap.travelled_m:.1f} max_cte_m={max_cte:.2f}"); return 1
            if speed > 0.5:
                last_move = time.time()
            elif time.time() - last_move > a.stall_s and lap.travelled_m > 5.0:
                print(f"D4_LAP_FAIL reason=stalled at_m={lap.travelled_m:.1f} max_cte_m={max_cte:.2f}"); return 1
            if lap.update(tf.location.x, tf.location.y):
                mean = speed_sum / n
                if mean < MIN_MEAN_SPEED:
                    print(f"D4_LAP_FAIL reason=too_slow mean_speed_mps={mean:.2f} lap_m={lap.travelled_m:.1f}"); return 1
                print(f"D4_LAP_PASS lap_m={lap.travelled_m:.1f} max_cte_m={max_cte:.2f} samples={n} mean_speed_mps={mean:.2f}"); return 0
            # One sample per simulation step, which is the 10 Hz the docstring
            # promises at fixed_delta_seconds 0.1. time.sleep() would resample the
            # same stale snapshot, because a passive client only updates on a tick.
            world.wait_for_tick(seconds=30.0)
    print(f"D4_LAP_FAIL reason=timeout at_m={lap.travelled_m:.1f} max_cte_m={max_cte:.2f}"); return 1


if __name__ == "__main__":
    sys.exit(main())
