#include "mobile_robot_controller/perfect_odom.hpp"

PerfectOdom::PerfectOdom() :
    Node("perfect_odom"), wheel_radius(0.0), wheel_separation(0.0),
    last_wheel_pos(0, 0), robot_pos(0, 0), robot_orein(0), robot_speed(0, 0), last_time(now())
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

    joint_states_sub = create_subscription<JointState>(
        "joint_states",
        10,
        [this](const JointState& msg)-> void {joint_states_callback(msg);}
    );

    odom_pub = create_publisher<Odometry>(
        "odom_perfect",
        10
    );

    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint_perfect";

    transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_base_tf.header.frame_id = "odom";
    odom_base_tf.child_frame_id = "base_footprint_perfect";

}

void PerfectOdom::joint_states_callback(const JointState& msg)
{
    auto current_time = rclcpp::Time(msg.header.stamp);
    double current_wheel_pos_left = msg.position.at(0);
    double current_wheel_pos_right = msg.position.at(1);
    Eigen::Vector<double, 2> current_wheel_pos(current_wheel_pos_right, current_wheel_pos_left);

    double dt = (current_time - last_time).nanoseconds() / 1e9;
    Eigen::Vector<double, 2> wheel_dp = current_wheel_pos - last_wheel_pos;

    last_time = current_time;
    last_wheel_pos = current_wheel_pos;

    Eigen::Vector<double, 2> wheel_speed = wheel_dp / dt;
    robot_speed = wheel_vel_to_cmd_vel_matrix * wheel_speed;

    double v = robot_speed.coeff(0);
    double w = robot_speed.coeff(1);

    Eigen::Vector<double, 2> robot_dtf = wheel_vel_to_cmd_vel_matrix * wheel_dp;
    double ds = robot_dtf.coeff(0);
    double dtheta = robot_dtf.coeff(1);

    double theta = robot_orein = robot_orein + dtheta;
    if (robot_orein >= 2 * M_PI)
    {
        theta = robot_orein = robot_orein -  2 * M_PI;
    }
    else if (robot_orein <= -2 * M_PI)
    {
        theta = robot_orein = robot_orein +  2 * M_PI;
    }

    double dx = std::cos(theta) * ds;
    double dy = std::sin(theta) * ds;

    Eigen::Vector<double, 2> dp(dx, dy);
    robot_pos += dp;

    double x = robot_pos.coeff(0);
    double y = robot_pos.coeff(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    q.normalize();

    // RCLCPP_INFO_STREAM(get_logger(), "linear velocity: " << v << " angular velocity: " << w);
    // RCLCPP_INFO_STREAM(get_logger(), "x: " << x << " y: " << y << " theta: " << theta);

    odom_msg.header.stamp = now();
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.angular.z = w;

    odom_pub->publish(odom_msg);

    odom_base_tf.header.stamp = now();
    odom_base_tf.transform.translation.x = x;
    odom_base_tf.transform.translation.y = y;
    odom_base_tf.transform.rotation.x = q.x();
    odom_base_tf.transform.rotation.y = q.y();
    odom_base_tf.transform.rotation.z = q.z();
    odom_base_tf.transform.rotation.w = q.w();

    transform_broadcaster->sendTransform(odom_base_tf);

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerfectOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}