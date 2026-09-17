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
        # config_carla.py holds the world in synchronous mode and drives the clock
        # itself, so this passive client receives no snapshot until it waits for
        # one. Until then get_actors() returns an empty list rather than raising,
        # and the ego reads as if it was never spawned. One seeding wait is enough:
        # snapshots then keep arriving on their own at the simulation rate, which
        # is why the timer below can stay a plain timer. Measured on Town04_Opt at
        # fixed_delta_seconds 0.1: 0 actors before the wait, 126 after, and the
        # frame number then advances without any further wait.
        # The wait is short and only happens while there is no ego to publish, so
        # it cannot stall the executor during normal operation.
        try:
            self.world.wait_for_tick(seconds=1.0)
        except RuntimeError:
            return None  # nothing is ticking yet; the next timer call retries
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == 'hero':
                return actor
        return None

    def tick(self):
        if self.ego is None or not self.ego.is_alive:
            self.ego = self.find_ego()
            if self.ego is None:
                return
        try:
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
        except Exception as exc:
            # The hero actor can be destroyed between the is_alive check above
            # and these calls (episode reset, actor teardown), and CARLA's
            # client raises for a dead actor rather than returning stale data.
            # Drop this tick and re-resolve the actor on the next one instead
            # of letting the exception kill the rclpy executor.
            self.get_logger().warning('ego state read failed, will re-resolve actor: %s', exc)
            self.ego = None


def main(args=None):
    rclpy.init(args=args)
    node = EgoStatePublisher()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
