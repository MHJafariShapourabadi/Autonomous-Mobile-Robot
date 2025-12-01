#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

class KalmanFilter(Node):
    def __init__(self):
        super().__init__(node_name="kalman_filter")

        self._odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self._odom_callback,
            qos_profile=10,
        )
        self._imu_sub = self.create_subscription(
            msg_type=Imu,
            topic="imu/data_raw",
            callback=self._imu_callback,
            qos_profile=10,
        )
        self._odom_pub = self.create_publisher(
            msg_type=Odometry,
            topic="odom_kalman",
            qos_profile=10,
        )

        self._angular_velocity_z_mean = 0.0
        self._angular_velocity_z_variance = 10.0

        self._imu_angular_velocity_z_mean = 0.0
        self._imu_angular_velocity_z_variance = 0.5

        self._is_first_ododm = True

        self._odom_angular_velocity_z_last = 0.0
        self._angular_velocity_z_motion_mean = 0.0
        self._angular_velocity_z_motion_variance = 4.0
        
        self._kalman_odom = Odometry()

        self.get_logger().info(f"{self.get_name()} node initialized!")

    def _state_prediction(self):
        self._angular_velocity_z_mean = self._angular_velocity_z_mean + self._angular_velocity_z_motion_mean
        self._angular_velocity_z_variance = self._angular_velocity_z_variance + self._angular_velocity_z_motion_variance

    def _measurement_update(self):
        self._angular_velocity_z_mean = (self._imu_angular_velocity_z_variance * self._angular_velocity_z_mean + self._angular_velocity_z_variance * self._imu_angular_velocity_z_mean) / (self._angular_velocity_z_variance + self._imu_angular_velocity_z_variance)
        self._angular_velocity_z_variance = (self._angular_velocity_z_variance * self._imu_angular_velocity_z_variance) / (self._angular_velocity_z_variance + self._imu_angular_velocity_z_variance)

    def _odom_callback(self, odom: Odometry):
        self._kalman_odom = odom

        if self._is_first_ododm:
            self._odom_angular_velocity_z_last = odom.twist.twist.angular.z
            self._angular_velocity_z_mean = odom.twist.twist.angular.z
            self._is_first_ododm = False
            return
        
        self._angular_velocity_z_motion_mean = odom.twist.twist.angular.z - self._odom_angular_velocity_z_last
        if self._angular_velocity_z_motion_mean == math.inf or self._angular_velocity_z_motion_mean == math.nan:
            return 
        
        self._state_prediction()

        self._measurement_update()

        self._kalman_odom.twist.twist.angular.z = self._angular_velocity_z_mean
        self._odom_pub.publish(msg=self._kalman_odom)

    def _imu_callback(self, imu: Imu):
        self._imu_angular_velocity_z_mean = imu.angular_velocity.z

def main(args=None):
    try:
        rclpy.init(args=args)
        node = KalmanFilter()
        rclpy.spin(node=node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()


