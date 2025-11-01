#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using std_msgs::msg::Float64MultiArray;
using geometry_msgs::msg::TwistStamped;

class SimpleController : public rclcpp::Node
{
public:
    SimpleController();

private:
    double wheel_radius;
    double wheel_separation;
    Eigen::Matrix<double, 2, 2> wheel_vel_to_cmd_vel_matrix;
    Eigen::Matrix<double, 2, 2> cmd_vel_to_wheel_vel_matrix;

    rclcpp::Publisher<Float64MultiArray>::SharedPtr wheel_vel_pub;
    rclcpp::Subscription<TwistStamped>::SharedPtr cmd_vel_sub;

    void cmd_vel_callback(TwistStamped msg);
};

#endif