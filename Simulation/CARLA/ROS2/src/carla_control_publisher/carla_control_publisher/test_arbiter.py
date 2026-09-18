"""Pure decision logic of carla_control_publisher. Run: python3 -m pytest test_arbiter.py"""
import arbiter as a


def test_clamp_rejects_phantom_cipo_spike():
    assert a.clamp_accel(-50.0) == a.A_DES_MIN
    assert a.clamp_accel(-8.0) == -8.0
    assert a.clamp_accel(-2.5) == -2.5
    assert a.clamp_accel(9.0) == a.A_DES_MAX
    assert a.clamp_accel(0.0) == 0.0


def test_choose_source_prefers_fresh_safety_island_command():
    assert a.choose_source(10.0, None) == "vp"
    assert a.choose_source(10.0, 9.7) == "si"
    assert a.choose_source(10.0, 9.5) == "si"
    assert a.choose_source(10.0, 9.4) == "vp"
