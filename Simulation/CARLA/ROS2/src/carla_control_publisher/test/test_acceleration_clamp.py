"""Tests for the planner-acceleration clamp in carla_control_publisher.

The longitudinal planner can emit a large transient deceleration when it
discovers a curve speed limit late (a measured -20.6 m/s^2 on CARLA Town04).
The bridge turns an acceleration into a speed target, so an unclamped spike
becomes a full-stop request mid-curve. These tests pin the clamp that keeps
the demand inside what the vehicle can actually do.
"""

from carla_control_publisher.carla_control_publisher_node import (
    A_DES_MAX,
    A_DES_MIN,
    clamp_acceleration,
)


def test_passes_through_an_in_range_deceleration():
    assert clamp_acceleration(-2.5) == -2.5


def test_clamps_a_planner_spike_to_the_deceleration_floor():
    # The -20.613 m/s^2 spike measured on Town04 at 29.18 m/s.
    assert clamp_acceleration(-20.613) == A_DES_MIN


def test_clamps_an_excessive_acceleration_to_the_ceiling():
    assert clamp_acceleration(12.0) == A_DES_MAX


def test_keeps_the_range_endpoints_unchanged():
    assert clamp_acceleration(A_DES_MIN) == A_DES_MIN
    assert clamp_acceleration(A_DES_MAX) == A_DES_MAX


def test_reports_whether_the_clamp_bit():
    assert clamp_acceleration(-2.5, report=True) == (-2.5, False)
    assert clamp_acceleration(-20.613, report=True) == (A_DES_MIN, True)


def _node_without_ros(monkeypatch):
    """Build a CarlaControlPublisher with rclpy's Node.__init__ neutralised.

    throttle_callback only touches plain attributes and the logger, so the
    callback can be exercised without a running ROS graph.
    """
    import rclpy.node

    from carla_control_publisher import carla_control_publisher_node as mod

    logged = []

    class _Logger:
        def warn(self, message, **kwargs):
            logged.append(message)

    monkeypatch.setattr(rclpy.node.Node, "__init__", lambda self, *a, **k: None)
    monkeypatch.setattr(mod.CarlaControlPublisher, "__init__", lambda self: None)

    node = mod.CarlaControlPublisher()
    node.acceleration = 0.0
    node.have_throttle = False
    node.get_logger = lambda: _Logger()
    node.try_publish = lambda: None
    return node, logged


class _Msg:
    def __init__(self, data):
        self.data = data


def test_throttle_callback_stores_an_in_range_demand_verbatim(monkeypatch):
    node, logged = _node_without_ros(monkeypatch)

    node.throttle_callback(_Msg(-2.5))

    assert node.acceleration == -2.5
    assert logged == []


def test_throttle_callback_clamps_a_spike_and_warns(monkeypatch):
    node, logged = _node_without_ros(monkeypatch)

    node.throttle_callback(_Msg(-20.613))

    assert node.acceleration == A_DES_MIN
    assert len(logged) == 1
    assert "-20.613" in logged[0]
