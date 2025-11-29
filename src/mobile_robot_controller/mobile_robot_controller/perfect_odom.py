#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

import numpy as np
import math

from quaternion import Quaternion

class PerfectOdom(Node):
    def __init__(self):
        super().__init__(node_name="perfect_odom")

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

        left_wheel_last_pos_ = 0.0
        right_wheel_last_pos_ = 0.0
        self._last_wheel_pos_ = np.array([
            [right_wheel_last_pos_],
            [left_wheel_last_pos_],
        ])
        self._robot_pos_ = np.zeros((2, 1))
        self._robot_orien_ = 0.0
        self._robot_speed_ = np.zeros((2, 1))
        self._last_time_ = self.get_clock().now()

        wheel_vel_to_cmd_vel_matrix_str = np.array2string(self._wheel_vel_to_cmd_vel_matrix_, precision=2, suppress_small=True)
        cmd_vel_to_wheel_vel_matrix_str = np.array2string(self._cmd_vel_to_wheel_vel_matrix_, precision=2, suppress_small=True)
        self.get_logger().info(f"conversion matrix: {wheel_vel_to_cmd_vel_matrix_str}")
        self.get_logger().info(f"conversion matrix inverse: {cmd_vel_to_wheel_vel_matrix_str}")


        self._joint_states_sub_ = self.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self._joint_states_callback,
            qos_profile=10,
        )

        self._odom_pub_ = self.create_publisher(
            msg_type=Odometry,
            topic="odom_perfect",
            qos_profile=10,
        )

        self._odom_msg_ = Odometry()
        self._odom_msg_.header.frame_id = "odom"
        self._odom_msg_.child_frame_id = "base_footprint_perfect"

        self._transform_broadcaster_ = TransformBroadcaster(
            node=self
        )

        self._odom_base_tf_ = TransformStamped()
        self._odom_base_tf_.header.frame_id = "odom"
        self._odom_base_tf_.child_frame_id = "base_footprint_perfect"

    def _joint_states_callback(self, msg: JointState):
        current_time = Time.from_msg(msg.header.stamp)
        current_wheel_pos_left = msg.position[0]
        current_wheel_pos_right = msg.position[1]
        current_wheel_pos = np.array([
            [current_wheel_pos_right],
            [current_wheel_pos_left],
        ])

        dt = (current_time - self._last_time_).nanoseconds / S_TO_NS
        # dp_left = current_wheel_pos_left - self._left_wheel_last_pos_
        # dp_right = current_wheel_pos_right - self._right_wheel_last_pos_
        wheel_dp = current_wheel_pos - self._last_wheel_pos_

        # self._left_wheel_last_pos_ = current_wheel_pos_left
        # self._right_wheel_last_pos_ = current_wheel_pos_right
        self._last_time_ = current_time
        self._last_wheel_pos_ = current_wheel_pos

        # omega_left = dp_left / dt
        # omega_right = dp_right / dt
        # wheel_speed = np.array([
        #     [omega_right],
        #     [omega_left]
        # ])

        wheel_speed = wheel_dp / (dt + 1e-10)
        robot_speed = self._robot_speed_ = np.matmul(self._wheel_vel_to_cmd_vel_matrix_, wheel_speed)
        v, w = robot_speed[0, 0], robot_speed[1, 0]

        robot_dtf = np.matmul(self._wheel_vel_to_cmd_vel_matrix_, wheel_dp)
        ds, dtheta = robot_dtf[0, 0], robot_dtf[1, 0]

        theta = self._robot_orien_ = self._robot_orien_ + dtheta
        if self._robot_orien_ >= 2 * math.pi:
            theta = self._robot_orien_ = self._robot_orien_ - 2 * math.pi
        elif self._robot_orien_ <= -2 * math.pi:
            theta = self._robot_orien_ = self._robot_orien_ + 2 * math.pi

        dx, dy = math.cos(theta) * ds, math.sin(theta) * ds
        robot_dp = np.array([
            [dx],
            [dy],
        ])
        
        self._robot_pos_ += robot_dp

        x, y = self._robot_pos_[0, 0], self._robot_pos_[1, 0]
        q = Quaternion.from_euler(roll=0, pitch=0, yaw=theta)
        q.normalize()

        # self.get_logger().info(f"linear velocity: {v :0.3f} angular velocity: {w:0.3f}")
        # self.get_logger().info(f"x: {x :0.3f} y: {y:0.3f} theta: {theta}")

        self._odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self._odom_msg_.pose.pose.position.x = x
        self._odom_msg_.pose.pose.position.y = y
        self._odom_msg_.pose.pose.orientation.x = q.x
        self._odom_msg_.pose.pose.orientation.y = q.y
        self._odom_msg_.pose.pose.orientation.z = q.z
        self._odom_msg_.pose.pose.orientation.w = q.w
        self._odom_msg_.twist.twist.linear.x = v
        self._odom_msg_.twist.twist.angular.z = w

        self._odom_pub_.publish(self._odom_msg_)

        self._odom_base_tf_.header.stamp = self.get_clock().now().to_msg()
        self._odom_base_tf_.transform.translation.x = x
        self._odom_base_tf_.transform.translation.y = y
        self._odom_base_tf_.transform.rotation.x = q.x
        self._odom_base_tf_.transform.rotation.y = q.y
        self._odom_base_tf_.transform.rotation.z = q.z
        self._odom_base_tf_.transform.rotation.w = q.w

        self._transform_broadcaster_.sendTransform(self._odom_base_tf_)



def main(args=None):
    try:
        rclpy.init(args=args)
        node = PerfectOdom()
        rclpy.spin(node=node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()