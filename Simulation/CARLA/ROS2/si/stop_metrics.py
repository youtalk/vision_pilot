"""Arithmetic behind gate D6; no ROS, no CARLA."""
import math


def first_after(t0, stamps):
    for t in stamps:
        if t >= t0:
            return t
    return None


def stop_distance(rows, stop_speed):
    """rows: (t, x, y, speed) from the fault instant on. Returns (metres, t_stop)."""
    d, prev = 0.0, None
    for t, x, y, v in rows:
        if prev is not None:
            d += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)
        if v <= stop_speed:
            return d, t
    return None, None
