import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from autoware_control_msgs.msg import Control
from std_msgs.msg import Float64

from .arbiter import clamp_accel, choose_source


class CarlaControlPublisher(Node):
    def __init__(self):
        super().__init__('carla_control_publisher')

        # VisionPilot reports +cte as "ego right of path" and emits the tyre angle
        # in that same sense, so a positive angle steers right. CARLA's native
        # ROS2 ackermann path uses the same convention rather than REP-103:
        # AckermannControlConversion.h assigns steering_angle straight to
        # AckermannControl.steer, AckermannController.cpp only rescales it by
        # VehicleMaxSteering, and python_api.md documents that field as
        # "Desired steer (rad). Positive value is to the right."
        # The two conventions already agree — negating here turns lane keeping
        # into positive feedback. Expose it for a differently-signed source.
        self.declare_parameter('steering_sign', 1.0)
        self.steering_sign = self.get_parameter('steering_sign').value

        self.steering_sub_ = self.create_subscription(Float64, '/vehicle/steering_cmd', self.steering_callback, 1)
        self.throttle_sub_ = self.create_subscription(Float64, '/vehicle/throttle_cmd', self.throttle_callback, 1)
        self.speed_sub_ = self.create_subscription(Float64, '/vehicle/speed', self.speed_callback, 1)
        self.si_sub_ = self.create_subscription(Control, '/control/trajectory_follower/control_cmd', self.si_callback, 1)
        self.ackerman_control_pub_ = self.create_publisher(AckermannDriveStamped, "/carla/hero/ackermann_control_cmd", 1)

        self.speed = 0.0
        self.v_ref = 30.0
        self.steering_angle_cmd = 0.0
        self.acceleration = 0.0
        self.si_rx_time = None

        # Publish only once BOTH a fresh steering and a fresh throttle have
        # arrived. Each planning cycle in VisionPilot emits one steering + one
        # throttle together, so this fires once per cycle, at the planner's own
        # rate, with no zero-order-hold delay.
        self.have_steering = False
        self.have_throttle = False

    def publish_ackermann(self, steering_angle, speed, acceleration):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'hero'
        cmd.drive.steering_angle = steering_angle  # radians
        cmd.drive.steering_angle_velocity = 0.0
        cmd.drive.speed = speed
        cmd.drive.acceleration = acceleration
        cmd.drive.jerk = 10.0
        self.ackerman_control_pub_.publish(cmd)

    def publish_control(self):
        self.publish_ackermann(float(self.steering_angle_cmd),
                               max(0.0, self.speed + self.acceleration * 2.0),
                               self.acceleration)

    def try_publish(self):
        if self.have_steering and self.have_throttle:
            now = self.get_clock().now().nanoseconds * 1e-9
            if choose_source(now, self.si_rx_time) == "vp":
                self.publish_control()
            self.have_steering = False
            self.have_throttle = False

    def steering_callback(self, msg):
        # self.get_logger().info(f'Steering command received: {msg.data}')
        self.steering_angle_cmd = self.steering_sign * msg.data
        self.have_steering = True
        self.try_publish()

    def throttle_callback(self, msg):
        # self.get_logger().info(f'Throttle command received: {msg.data}')
        self.acceleration = clamp_accel(msg.data)
        self.have_throttle = True
        self.try_publish()

    def speed_callback(self, msg):
        self.speed = msg.data

    def si_callback(self, msg):
        self.si_rx_time = self.get_clock().now().nanoseconds * 1e-9
        self.publish_ackermann(self.steering_sign * float(msg.lateral.steering_tire_angle),
                               max(0.0, float(msg.longitudinal.velocity)),
                               float(msg.longitudinal.acceleration))


def main(args=None):
    rclpy.init(args=args)
    node = CarlaControlPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
