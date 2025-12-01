#include "mobile_robot_localization/kalman_filter.hpp"

KalmanFilter::KalmanFilter()
: Node("kalman_filter")
, angular_velocity_z_mean(0.0)
, angular_velocity_z_variance(10.0)
, imu_angular_velocity_z_mean(0.0)
, imu_angular_velocity_z_variance(0.5)
, is_first_odom(true)
, odom_angular_velocity_z_last(0.0)
, angular_velocity_z_motion_mean(0.0)
, angular_velocity_z_motion_variance(4.0)
{
    odom_sub = create_subscription<Odometry>(
        "odom",
        10,
        [this](const Odometry& odom)-> void {odom_callback(odom);}
    );

    imu_sub = create_subscription<Imu>(
        "imu/data_raw",
        10,
        [this](const Imu& imu)-> void {imu_callback(imu);}
    );

    odom_pub = create_publisher<Odometry>(
        "odom_kalman",
        10
    );

    RCLCPP_INFO_STREAM(get_logger(), std::string(get_name()) << " node initialized!");
}

void KalmanFilter::state_prediction()
{
    angular_velocity_z_mean = angular_velocity_z_mean + angular_velocity_z_motion_mean;
    angular_velocity_z_variance = angular_velocity_z_variance + angular_velocity_z_motion_variance;
}

void KalmanFilter::measurement_update()
{
    angular_velocity_z_mean = (imu_angular_velocity_z_variance * angular_velocity_z_mean + angular_velocity_z_variance * imu_angular_velocity_z_mean) / (angular_velocity_z_variance + imu_angular_velocity_z_variance);
    angular_velocity_z_variance = (angular_velocity_z_variance * imu_angular_velocity_z_variance) / (angular_velocity_z_variance + imu_angular_velocity_z_variance);
}

void KalmanFilter::odom_callback(const Odometry& odom)
{
    kalman_odom = odom;

    if(is_first_odom)
    {
        odom_angular_velocity_z_last = odom.twist.twist.angular.z;
        angular_velocity_z_mean = odom.twist.twist.angular.z;
        is_first_odom = false;
        return;
    }

    angular_velocity_z_motion_mean = odom.twist.twist.angular.z - odom_angular_velocity_z_last;
    if(!std::isfinite(angular_velocity_z_motion_mean))
    {
        return;
    }

    state_prediction();

    measurement_update();

    kalman_odom.twist.twist.angular.z = angular_velocity_z_mean;
    odom_pub->publish(kalman_odom);

    // RCLCPP_INFO_STREAM(get_logger(), "=============================");
    // RCLCPP_INFO_STREAM(get_logger(), "angular z mean: " << angular_velocity_z_mean);
    // RCLCPP_INFO_STREAM(get_logger(), "imu angular z mean: " << imu_angular_velocity_z_mean);
    // RCLCPP_INFO_STREAM(get_logger(), "odom angular z: " << odom.twist.twist.angular.z); // This comes from w in noisy_odom.cpp and sometimes is nan
    // RCLCPP_INFO_STREAM(get_logger(), "motion angular z mean: " << angular_velocity_z_motion_mean);
    // RCLCPP_INFO_STREAM(get_logger(), "angular z variance: " << angular_velocity_z_variance);
}

void KalmanFilter::imu_callback(const Imu& imu)
{
    imu_angular_velocity_z_mean = imu.angular_velocity.z;
}


int main(int argc, const char * const * argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilter>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();

    return 0;
}