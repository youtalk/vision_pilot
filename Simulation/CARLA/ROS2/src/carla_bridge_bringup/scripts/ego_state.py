"""CARLA world frame is left-handed; ROS is right-handed: y_ros = -y, yaw_ros = -yaw.
Same mapping carla_mpc_bridge.py uses in carla-visionpilot-mpc-demo."""
import math


def carla_to_ros(x, y, yaw_deg, vx, vy, vz):
    return x, -y, -math.radians(yaw_deg), math.sqrt(vx * vx + vy * vy + vz * vz)


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
