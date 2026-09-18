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


def first_ramp_after(t0, rows, decel):
    """rows: (t, accel) of /carla/hero/ackermann_control_cmd. The first command at or
    below -decel (with 0.5 m/s^2 slack) after t0 is the arbiter forwarding the ramp."""
    for t, a in rows:
        if t >= t0 and a <= -decel + 0.5:
            return t, a
    return None


# Gate D6's decision ladder, kept here so every branch is testable: si_stop_gate
# imports rclpy at module level and nothing in it could be exercised on a bench PC.
MIN_PRE_FAULT_SPEED = 5.0   # m/s
# A stop from the gate D4 cruise speed of 12 m/s at the firmware's 3 m/s^2 ramp
# covers v^2 / 2a = 24 m. 40 m leaves headroom for the reaction time and a
# softer real ramp while still rejecting a coast-down that happens to reach
# zero inside the 30 s window.
MAX_STOP_M = 40.0


def verdict(fault_at, raw_stamps, ack, rows, max_latency_ms,
            min_pre_fault_speed=MIN_PRE_FAULT_SPEED, max_stop_m=MAX_STOP_M,
            stop_speed=0.05, decel=3.0):
    """The one SI_STOP_ marker line for a run.

    rows are (t, x, y, speed) from the gate's own start, which is about DRIVE_S
    before the fault, so the pre-fault samples are there to be judged.
    """
    # A car that never moved satisfies "stopped" at sample zero, so without
    # this it passes the gate with stop_distance_m=0.00, which a person reads
    # as "the Safety Island stopped the car in 8 ms".
    # Having no pre-fault odometry at all is a plumbing failure, not a parked
    # car, and the two used to share one reason slug. Keep them apart, and
    # carry the speed never_moving was judged on so the number is arguable.
    pre = [r for r in rows if r[0] < fault_at]
    if not pre:
        return "SI_STOP_FAIL reason=no_pre_fault_odom"
    max_pre = max(r[3] for r in pre)
    if max_pre < min_pre_fault_speed:
        return f"SI_STOP_FAIL reason=never_moving max_pre_speed_mps={max_pre:.2f}"
    # Commands already flowing before the fault mean first_cr52_cmd_ms measures
    # the next periodic sample, not a reaction to anything.
    if any(t < fault_at for t in raw_stamps):
        return "SI_STOP_FAIL reason=cmd_before_fault"
    t_first = first_after(fault_at, raw_stamps)
    if t_first is None:
        return "SI_STOP_FAIL reason=no_cr52_cmd"
    latency_ms = (t_first - fault_at) * 1000.0
    if latency_ms > max_latency_ms:
        return f"SI_STOP_FAIL reason=cmd_late first_cr52_cmd_ms={latency_ms:.0f}"
    sw = first_ramp_after(fault_at, ack, decel)
    if sw is None:
        return f"SI_STOP_FAIL reason=arbiter_never_switched first_cr52_cmd_ms={latency_ms:.0f}"
    ack_ms = (sw[0] - fault_at) * 1000.0
    d, t_stop = stop_distance([r for r in rows if r[0] >= fault_at], stop_speed)
    if d is None:
        return f"SI_STOP_FAIL reason=no_stop first_cr52_cmd_ms={latency_ms:.0f}"
    if d > max_stop_m:
        # Every sibling FAIL line carries the latency, and the PASS line
        # carries stop_s. Without them this line cannot distinguish a slow
        # reaction from weak braking, which is the first question a reader
        # asks. Board 2's standin rehearsal on 2026-09-18 failed here at
        # 67.14 m and the answer was not in the record.
        return (f"SI_STOP_FAIL reason=stop_too_far stop_distance_m={d:.2f} "
                f"first_cr52_cmd_ms={latency_ms:.0f} first_si_ack_ms={ack_ms:.0f} "
                f"stop_s={t_stop - fault_at:.2f}")
    return (f"SI_STOP_PASS first_cr52_cmd_ms={latency_ms:.0f} first_si_ack_ms={ack_ms:.0f} "
            f"stop_distance_m={d:.2f} stop_s={t_stop - fault_at:.2f}")
