#!/usr/bin/env python3
"""Publish the ego's kinematic state and steering report for the Safety Island.

CARLA's native ROS 2 emits no odometry for a vehicle that is not under
Autoware control, so the state is read from the Python client API at 20 Hz.
Both topics are bridged 1 -> 2 to the CR52 by the board's domain_bridge.
"""
import math
import os
import carla
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_vehicle_msgs.msg import SteeringReport
from ego_state import carla_to_ros, yaw_to_quaternion

MAX_STEER_RAD = math.radians(70.0)  # vehicle.lincoln.mkz max wheel angle in CARLA 0.10


class EgoStatePublisher(Node):
    def __init__(self):
        super().__init__('carla_ego_state_publisher')
        host = os.environ.get('CARLA_HOST', 'localhost'); port = int(os.environ.get('CARLA_PORT', '2000'))
        self.client = carla.Client(host, port); self.client.set_timeout(30.0)
        self.world = self.client.get_world()
        self.ego = None
        self.odom_pub = self.create_publisher(Odometry, '/localization/kinematic_state', 1)
        self.steer_pub = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 1)
        self.create_timer(0.05, self.tick)

    def find_ego(self):
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == 'hero':
                return actor
        return None

    def tick(self):
        if self.ego is None or not self.ego.is_alive:
            self.ego = self.find_ego()
            if self.ego is None:
                return
        tf = self.ego.get_transform(); vel = self.ego.get_velocity()
        x, y, yaw, speed = carla_to_ros(tf.location.x, tf.location.y, tf.rotation.yaw, vel.x, vel.y, vel.z)
        stamp = self.get_clock().now().to_msg()
        od = Odometry(); od.header.stamp = stamp; od.header.frame_id = 'map'; od.child_frame_id = 'base_link'
        od.pose.pose.position.x = x; od.pose.pose.position.y = y; od.pose.pose.position.z = tf.location.z
        od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z, od.pose.pose.orientation.w = yaw_to_quaternion(yaw)
        od.twist.twist.linear.x = speed
        self.odom_pub.publish(od)
        sr = SteeringReport(); sr.stamp = stamp
        sr.steering_tire_angle = -float(self.ego.get_control().steer) * MAX_STEER_RAD
        self.steer_pub.publish(sr)


def main(args=None):
    rclpy.init(args=args)
    node = EgoStatePublisher()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
