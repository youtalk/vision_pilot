"""Pure lap geometry. Run: python3 -m pytest test_lap_geom.py"""
import math
import lap_geom as g


def test_cte_sign_and_magnitude():
    # lane heading +x (yaw 0): a point at y=+0.5 is 0.5 m to the left in CARLA's left-handed frame
    assert math.isclose(g.signed_cte(0.0, 0.0, 0.0, 3.0, 0.5), 0.5)
    assert math.isclose(g.signed_cte(0.0, 0.0, 0.0, 3.0, -0.5), -0.5)
    # heading +y (yaw 90): offset along -x is left
    assert math.isclose(g.signed_cte(0.0, 0.0, 90.0, -0.7, 5.0), 0.7)


def test_lap_closes_only_after_min_travel():
    t = g.LapTracker(0.0, 0.0, min_travel_m=100.0, close_radius_m=5.0)
    assert not t.update(0.0, 0.0)
    for x in range(1, 61):
        assert not t.update(float(x), 0.0)   # 60 m out, never near the start
    assert t.update(2.0, 0.0)                # back within 5 m after 118 m: closed
    assert t.closed
    assert t.travelled_m > 100.0


def test_lap_does_not_close_early():
    t = g.LapTracker(0.0, 0.0, min_travel_m=100.0, close_radius_m=5.0)
    t.update(10.0, 0.0); assert not t.update(1.0, 0.0)
    assert not t.closed
