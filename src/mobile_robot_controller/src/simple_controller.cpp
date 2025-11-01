#include "mobile_robot_controller/simple_controller.hpp"

SimpleController::SimpleController() :
    Node("simple_controller")
{
    declare_parameter("wheel_radius", 0.1);
    declare_parameter("wheel_separation", 0.45);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_separation = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius);
    RCLCPP_INFO_STREAM(get_logger(), "wheel separation: " << wheel_separation);

    wheel_vel_to_cmd_vel_matrix << wheel_radius / 2.0, wheel_radius / 2.0, 
                                   wheel_radius / wheel_separation, - wheel_radius / wheel_separation;

    cmd_vel_to_wheel_vel_matrix = wheel_vel_to_cmd_vel_matrix.inverse();

    RCLCPP_INFO_STREAM(get_logger(), "conversion matrix: /n" << wheel_vel_to_cmd_vel_matrix);
    RCLCPP_INFO_STREAM(get_logger(), "conversion matrix inverse: /n" << cmd_vel_to_wheel_vel_matrix);

    wheel_vel_pub = create_publisher<Float64MultiArray>(
        "simple_velocity_controller/commands",
        10
    );

    cmd_vel_sub = create_subscription<TwistStamped>(
        "cmd_vel",
        10,
        [this](TwistStamped msg)-> void {cmd_vel_callback(msg);}
    );
}

void SimpleController::cmd_vel_callback(TwistStamped msg)
{
    double v = msg.twist.linear.x;
    double w = msg.twist.angular.z;

    Eigen::Vector<double, 2> robot_speed(v, w);
    Eigen::Vector<double, 2> wheel_speed = cmd_vel_to_wheel_vel_matrix * robot_speed;

    Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

    wheel_vel_pub->publish(wheel_speed_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}