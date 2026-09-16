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
