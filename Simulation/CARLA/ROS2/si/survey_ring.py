#!/usr/bin/env python3
"""Rank Town04_Opt spawn points for a junction-free closed lap.

From each spawn, walk the road graph with the straightest continuation in 2 m
steps. Stop at the first waypoint inside a junction (that is where VisionPilot
departs, see carla-visionpilot-mpc-demo/README.md), or when the walk returns
within 10 m of the start after at least 500 m (a closed lap). Needs a running
server on localhost:2000 with the map loaded; host venv, no ROS.
"""
import argparse
import math
import carla


def straightest(cands, heading_deg):
    best, best_d = None, 1e9
    for c in cands:
        d = abs((c.transform.rotation.yaw - heading_deg + 180.0) % 360.0 - 180.0)
        if d < best_d:
            best, best_d = c, d
    return best


def walk(carla_map, spawn, distance, step=2.0):
    wp = carla_map.get_waypoint(spawn.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    sx, sy = wp.transform.location.x, wp.transform.location.y
    travelled = 0.0
    while travelled < distance:
        nxt = wp.next(step)
        if not nxt:
            return travelled, None, travelled
        wp = straightest(nxt, wp.transform.rotation.yaw)
        travelled += step
        if wp.is_junction:
            return None, travelled, travelled
        if travelled >= 500.0 and math.hypot(wp.transform.location.x - sx, wp.transform.location.y - sy) <= 10.0:
            return travelled, None, travelled
    return None, None, travelled


def main():
    p = argparse.ArgumentParser(); p.add_argument("--distance", type=float, default=6000.0); a = p.parse_args()
    client = carla.Client("localhost", 2000); client.set_timeout(30.0)
    carla_map = client.get_world().get_map()
    rows = []
    for i, sp in enumerate(carla_map.get_spawn_points()):
        loop_m, junction_at, clean_m = walk(carla_map, sp, a.distance)
        rows.append((i, loop_m or 0.0, clean_m, junction_at or 0.0))
    rows.sort(key=lambda r: (r[1], r[2]), reverse=True)
    print("idx   loop_m  clean_m  junction_at_m")
    for i, loop_m, clean_m, j in rows[:20]:
        print(f"{i:4d} {loop_m:8.0f} {clean_m:8.0f} {j:8.0f}")


if __name__ == "__main__":
    main()
