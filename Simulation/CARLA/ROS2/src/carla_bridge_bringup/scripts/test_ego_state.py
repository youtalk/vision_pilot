"""Frame mapping CARLA (left-handed) -> ROS (right-handed). Run: python3 -m pytest test_ego_state.py"""
import math
import ego_state as e


def test_y_and_yaw_flip_sign():
    x, y, yaw, v = e.carla_to_ros(10.0, 5.0, 90.0, 3.0, 4.0, 0.0)
    assert (x, y) == (10.0, -5.0)
    assert math.isclose(yaw, -math.pi / 2)
    assert math.isclose(v, 5.0)


def test_quaternion_from_yaw():
    qx, qy, qz, qw = e.yaw_to_quaternion(math.pi)
    assert (qx, qy) == (0.0, 0.0)
    assert math.isclose(qz, 1.0) and math.isclose(qw, 0.0, abs_tol=1e-12)
