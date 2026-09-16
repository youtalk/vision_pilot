#!/usr/bin/env python3
"""Gate D4: VisionPilot drives one closed lap unaided with |cte| under the limit.

Samples the hero at 10 Hz through the client API (no ROS), like mpc_trace.py.
  lap_gate.py <seconds> [--max-cte 1.0] [--min-lap-m 500] [--stall-s 20] [--csv out.csv]
Prints one D4_LAP_PASS or D4_LAP_FAIL line and exits 0/1.
"""
import argparse
import csv
import sys
import time
import carla
from lap_geom import LapTracker, signed_cte


def find_hero(world):
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") == "hero":
            return a
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("seconds", type=float); p.add_argument("--max-cte", type=float, default=1.0)
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
    max_cte = 0.0; last_move = time.time(); t_end = time.time() + a.seconds; n = 0
    with open(a.csv, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "x", "y", "speed", "cte", "travelled"])
        while time.time() < t_end:
            tf = hero.get_transform(); v = hero.get_velocity(); speed = (v.x ** 2 + v.y ** 2) ** 0.5
            wp = cmap.get_waypoint(tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            cte = signed_cte(wp.transform.location.x, wp.transform.location.y, wp.transform.rotation.yaw, tf.location.x, tf.location.y) if wp else float("nan")
            n += 1; w.writerow([f"{time.time():.3f}", f"{tf.location.x:.3f}", f"{tf.location.y:.3f}", f"{speed:.3f}", f"{cte:.3f}", f"{lap.travelled_m:.1f}"])
            if speed > 0.5:
                last_move = time.time()
                max_cte = max(max_cte, abs(cte))
                if abs(cte) > a.max_cte:
                    print(f"D4_LAP_FAIL reason=lane_departure at_m={lap.travelled_m:.1f} max_cte_m={max_cte:.2f}"); return 1
            elif time.time() - last_move > a.stall_s and lap.travelled_m > 5.0:
                print(f"D4_LAP_FAIL reason=stalled at_m={lap.travelled_m:.1f} max_cte_m={max_cte:.2f}"); return 1
            if lap.update(tf.location.x, tf.location.y):
                print(f"D4_LAP_PASS lap_m={lap.travelled_m:.1f} max_cte_m={max_cte:.2f} samples={n}"); return 0
            time.sleep(0.1)
    print(f"D4_LAP_FAIL reason=timeout at_m={lap.travelled_m:.1f} max_cte_m={max_cte:.2f}"); return 1


if __name__ == "__main__":
    sys.exit(main())
