"""The stand-in's ramp, no ROS. Run: python3 -m pytest test_si_standin.py"""
import si_standin as s


def test_ramp_is_linear_and_floors_at_zero():
    assert s.ramp(12.0, 0.0) == 12.0
    assert s.ramp(12.0, 1.0) == 9.0
    assert s.ramp(12.0, 4.0) == 0.0
    assert s.ramp(12.0, 9.0) == 0.0
