#!/usr/bin/env python3

import rclpy
from rclpy import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped

import numpy as np

class SimpleController(Node):
    def __init__(self):
        super().__init__(node_name="simple_controller")

        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_sepaparion', 0.45)

        self._wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self._wheel_separation = self.get_parameter('wheel_sepaparion').get_parameter_value().double_value

        self.get_logger().info(f"wheel radius: {self._wheel_radius :0.2f}")
        self.get_logger().info(f"wheel separation: {self._wheel_separation :0.2f}")

        self._wheel_vel_to_cmd_vel_matrix = np.array([
            [self._wheel_radius / 2.0, self._wheel_radius / 2.0],
            [self._wheel_radius / self._wheel_separation, - self._wheel_radius / self._wheel_separation]
        ])

        self._cmd_vel_to_wheel_vel_matrix = np.linalg.inv(self._wheel_vel_to_cmd_vel_matrix)

        self.get_logger().info(f"conversion matrix: {self._wheel_vel_to_cmd_vel_matrix :0.2f}")
        self.get_logger().info(f"conversion matrix inverse: {self._cmd_vel_to_wheel_vel_matrix :0.2f}")

        self.wheel_vel_pub = self.create_publisher(
            msg_type=Float64MultiArray,
            topic='simple_velocity_controller/commands',
            qos_profile=10
        )

        self.cmd_vel_sub = self.create_subscription(
            msg_type=TwistStamped,
            topic="cmd_vel",
            callback=self.cmd_vel_callback,
            qos_profile=10
        )

    def cmd_vel_callback(self, msg: TwistStamped):
        v = msg.twist.linear.x
        w = msg.twist.angular.z

        robot_speed = np.array([
            [v],
            [w]
        ])

        wheel_speed = np.matmul(self._cmd_vel_to_wheel_vel_matrix, robot_speed)

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]] # [left_wheel_speed, right_wheel_speed]

        self.wheel_vel_pub.publish(wheel_speed_msg)

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