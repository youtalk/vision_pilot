"""Lane-relative geometry for the D4 lap gate. No CARLA import here.

signed_cte matches mpc_trace.py in carla-visionpilot-mpc-demo: positive means
the ego is left of the lane centre in CARLA's left-handed frame.
"""
import math


def signed_cte(center_x, center_y, center_yaw_deg, x, y):
    yaw = math.radians(center_yaw_deg)
    rx, ry = -math.sin(yaw), math.cos(yaw)
    return (x - center_x) * rx + (y - center_y) * ry


class LapTracker:
    def __init__(self, start_x, start_y, min_travel_m, close_radius_m):
        self.sx, self.sy = start_x, start_y
        self.min_travel_m, self.close_radius_m = min_travel_m, close_radius_m
        self.travelled_m = 0.0
        self.closed = False
        self._last = None

    def update(self, x, y):
        if self._last is not None:
            self.travelled_m += math.hypot(x - self._last[0], y - self._last[1])
        self._last = (x, y)
        if not self.closed and self.travelled_m >= self.min_travel_m \
                and math.hypot(x - self.sx, y - self.sy) <= self.close_radius_m:
            self.closed = True
        return self.closed
