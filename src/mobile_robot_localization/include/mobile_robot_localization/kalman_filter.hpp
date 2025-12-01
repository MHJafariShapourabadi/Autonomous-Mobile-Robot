#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;

class KalmanFilter: public rclcpp::Node
{
private:
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub;

    double angular_velocity_z_mean;
    double angular_velocity_z_variance;

    double imu_angular_velocity_z_mean;
    double imu_angular_velocity_z_variance;

    bool is_first_odom;

    double odom_angular_velocity_z_last;
    double angular_velocity_z_motion_mean;
    double angular_velocity_z_motion_variance;

    Odometry kalman_odom;

    void imu_callback(const Imu& imu);
    void odom_callback(const Odometry& odom);

    void state_prediction();
    void measurement_update();

public:
    KalmanFilter();
};

#endif