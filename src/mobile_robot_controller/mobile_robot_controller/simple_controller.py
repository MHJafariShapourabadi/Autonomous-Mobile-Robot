#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

import numpy as np
import math

from quaternion import Quaternion

class SimpleController(Node):
    def __init__(self):
        super().__init__(node_name="simple_controller")

        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_sepaparion', 0.35)

        self._wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self._wheel_separation_ = self.get_parameter('wheel_sepaparion').get_parameter_value().double_value

        self.get_logger().info(f"wheel radius: {self._wheel_radius_ :0.2f}")
        self.get_logger().info(f"wheel separation: {self._wheel_separation_ :0.2f}")

        self._wheel_vel_to_cmd_vel_matrix_ = np.array([
            [self._wheel_radius_ / 2.0, self._wheel_radius_ / 2.0],
            [self._wheel_radius_ / self._wheel_separation_, - self._wheel_radius_ / self._wheel_separation_]
        ])

        self._cmd_vel_to_wheel_vel_matrix_ = np.linalg.inv(self._wheel_vel_to_cmd_vel_matrix_)

        wheel_vel_to_cmd_vel_matrix_str = np.array2string(self._wheel_vel_to_cmd_vel_matrix_, precision=2, suppress_small=True)
        cmd_vel_to_wheel_vel_matrix_str = np.array2string(self._cmd_vel_to_wheel_vel_matrix_, precision=2, suppress_small=True)
        self.get_logger().info(f"conversion matrix: {wheel_vel_to_cmd_vel_matrix_str}")
        self.get_logger().info(f"conversion matrix inverse: {cmd_vel_to_wheel_vel_matrix_str}")

        self._wheel_vel_pub_ = self.create_publisher(
            msg_type=Float64MultiArray,
            topic='simple_velocity_controller/commands',
            qos_profile=10,
        )

        self._cmd_vel_sub_ = self.create_subscription(
            msg_type=TwistStamped,
            topic="cmd_vel",
            callback=self._cmd_vel_callback,
            qos_profile=10,
        )


    def _cmd_vel_callback(self, msg: TwistStamped):
        v = msg.twist.linear.x
        w = msg.twist.angular.z

        robot_speed = np.array([
            [v],
            [w]
        ])

        wheel_speed = np.matmul(self._cmd_vel_to_wheel_vel_matrix_, robot_speed)

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]] # [left_wheel_speed, right_wheel_speed]

        self._wheel_vel_pub_.publish(wheel_speed_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = SimpleController()
        rclpy.spin(node=node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()