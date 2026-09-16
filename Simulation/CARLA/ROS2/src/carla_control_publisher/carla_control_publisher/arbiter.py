"""Decisions of carla_control_publisher, kept free of rclpy so they are testable.

VisionPilot's steering/throttle pair drives CARLA. When the Safety Island on
the X5H CR52 publishes a fresh control_cmd (bridged from domain 2 and
restamped on the board), that command wins: it is the stop ramp, and the
demo's claim is that the CR52 made that decision.
"""
A_DES_MIN = -8.0   # a phantom CIPO once produced -50 m/s^2
A_DES_MAX = 1.5
SI_FRESH_SEC = 0.5  # 3 CR52 control cycles at 0.15 s


def clamp_accel(a: float) -> float:
    return max(A_DES_MIN, min(A_DES_MAX, a))


def choose_source(now: float, si_rx_time):
    if si_rx_time is not None and now - si_rx_time <= SI_FRESH_SEC:
        return "si"
    return "vp"
