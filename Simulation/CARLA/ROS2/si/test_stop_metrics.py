"""Pure D6 arithmetic. Run: python3 -m pytest test_stop_metrics.py"""
import stop_metrics as s


def test_first_after_picks_first_sample_at_or_after_t0():
    assert s.first_after(10.0, [9.0, 9.9, 10.1, 10.4]) == 10.1
    assert s.first_after(10.0, [9.0]) is None
    assert s.first_after(10.0, []) is None


def test_stop_distance_is_path_length_until_speed_is_zero():
    rows = [(10.0, 0.0, 0.0, 5.0), (10.5, 2.0, 0.0, 3.0), (11.0, 3.0, 0.0, 0.4), (11.5, 3.2, 0.0, 0.0), (12.0, 3.2, 0.0, 0.0)]
    d, t_stop = s.stop_distance(rows, stop_speed=0.05)
    assert abs(d - 3.2) < 1e-9
    assert t_stop == 11.5


def test_stop_distance_none_when_never_stopped():
    assert s.stop_distance([(10.0, 0.0, 0.0, 5.0), (11.0, 4.0, 0.0, 4.0)], 0.05) == (None, None)


def test_first_ramp_after_finds_the_first_braking_ackermann():
    rows = [(9.0, -0.1), (10.2, 0.4), (10.5, -3.0), (10.7, -3.0)]
    assert s.first_ramp_after(10.0, rows, 3.0) == (10.5, -3.0)
    assert s.first_ramp_after(11.0, rows, 3.0) is None


# --- the decision ladder -------------------------------------------------
FAULT = 1000.0


def _rows(v0=12.0, decel=3.0, pre_s=20.0, post_s=10.0, dt=0.05):
    """Ego cruising at v0 until the fault, then braking at decel to a stop."""
    rows = []
    x = 0.0
    for i in range(int((pre_s + post_s) / dt)):
        t = FAULT - pre_s + i * dt
        v = v0 if t < FAULT else max(0.0, v0 - decel * (t - FAULT))
        rows.append((t, x, 0.0, v))
        x += v * dt
    return rows


def _stamps(first_at, n=100, period=0.15):
    return [first_at + period * i for i in range(n)]


def _ack(first_at, n=100, accel=-3.0):
    return [(t, accel) for t in _stamps(first_at, n)]


def test_verdict_passes_a_healthy_run():
    v = s.verdict(FAULT, _stamps(FAULT + 0.05), _ack(FAULT + 0.05), _rows(), 200.0)
    assert v.startswith("SI_STOP_PASS first_cr52_cmd_ms=50 first_si_ack_ms=50 ")
    assert "stop_distance_m=24" in v and "stop_s=4.0" in v


def test_verdict_fails_a_car_that_never_moved():
    # The B3 case: parked ego, Safety Island already braking. The old ladder
    # called this SI_STOP_PASS ... stop_distance_m=0.00 stop_s=0.00.
    parked = [(FAULT - 20 + 0.05 * i, 0.0, 0.0, 0.0) for i in range(600)]
    assert s.verdict(FAULT, _stamps(FAULT + 0.05), _ack(FAULT + 0.05), parked, 200.0) \
        == "SI_STOP_FAIL reason=never_moving max_pre_speed_mps=0.00"


def test_verdict_fails_a_crawl_before_the_fault():
    # The speed it was judged on rides along, so a reader can argue with it
    # instead of taking "never_moving" on trust.
    assert s.verdict(FAULT, _stamps(FAULT + 0.05), _ack(FAULT + 0.05), _rows(v0=2.0), 200.0) \
        == "SI_STOP_FAIL reason=never_moving max_pre_speed_mps=2.00"


def test_verdict_fails_when_there_are_no_pre_fault_samples():
    # A distinct slug from never_moving: no odometry at all is a plumbing
    # failure, and blaming the vehicle for it sends the reader the wrong way.
    late = [r for r in _rows() if r[0] >= FAULT]
    assert s.verdict(FAULT, _stamps(FAULT + 0.05), _ack(FAULT + 0.05), late, 200.0) \
        == "SI_STOP_FAIL reason=no_pre_fault_odom"


def test_verdict_fails_when_commands_were_already_flowing():
    # first_cr52_cmd_ms would measure the next periodic sample, not a reaction.
    assert s.verdict(FAULT, _stamps(FAULT - 10.0), _ack(FAULT + 0.05), _rows(), 200.0) \
        == "SI_STOP_FAIL reason=cmd_before_fault"


def test_verdict_fails_a_coast_down_that_stops_too_far_away():
    v = s.verdict(FAULT, _stamps(FAULT + 0.05), _ack(FAULT + 0.05),
                  _rows(decel=1.0, post_s=20.0), 200.0)
    assert v.startswith("SI_STOP_FAIL reason=stop_too_far stop_distance_m=7")
    # The fields that tell a slow reaction apart from weak braking.
    assert "first_cr52_cmd_ms=" in v and "first_si_ack_ms=" in v and "stop_s=" in v


def test_verdict_names_the_remaining_rungs():
    rows = _rows()
    assert s.verdict(FAULT, [], _ack(FAULT + 0.05), rows, 200.0) \
        == "SI_STOP_FAIL reason=no_cr52_cmd"
    assert s.verdict(FAULT, _stamps(FAULT + 0.98), _ack(FAULT + 0.05), rows, 200.0) \
        == "SI_STOP_FAIL reason=cmd_late first_cr52_cmd_ms=980"
    assert s.verdict(FAULT, _stamps(FAULT + 0.05), _ack(FAULT + 0.05, accel=-0.2), rows, 200.0) \
        == "SI_STOP_FAIL reason=arbiter_never_switched first_cr52_cmd_ms=50"
    never = [r for r in rows if r[0] < FAULT + 1.0]   # window ends while still rolling
    assert s.verdict(FAULT, _stamps(FAULT + 0.05), _ack(FAULT + 0.05), never, 200.0) \
        == "SI_STOP_FAIL reason=no_stop first_cr52_cmd_ms=50"
