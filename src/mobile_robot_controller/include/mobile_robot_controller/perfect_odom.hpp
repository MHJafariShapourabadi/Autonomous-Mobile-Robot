#ifndef PERFECT_ODOM_HPP
#define PERFECT_ODOM_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::JointState;
using nav_msgs::msg::Odometry;

class PerfectOdom : public rclcpp::Node
{
public:
    PerfectOdom();

private:
    double wheel_radius;
    double wheel_separation;
    Eigen::Matrix<double, 2, 2> wheel_vel_to_cmd_vel_matrix;
    Eigen::Matrix<double, 2, 2> cmd_vel_to_wheel_vel_matrix;
    Eigen::Vector<double, 2> last_wheel_pos;
    Eigen::Vector<double, 2> robot_pos;
    double robot_orein;
    Eigen::Vector<double, 2> robot_speed;
    rclcpp::Time last_time;
    Odometry odom_msg;
    TransformStamped odom_base_tf;

    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub;
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub;

    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;

    void joint_states_callback(const JointState& msg);
};

#endif