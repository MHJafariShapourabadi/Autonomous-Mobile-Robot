#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using sensor_msgs::msg::Imu;

class ImuFrameIDConverterNode : public rclcpp::Node
{
    private:
        rclcpp::Subscription<Imu>::SharedPtr node_subscriber;
        rclcpp::Publisher<Imu>::SharedPtr node_publisher;
        void subscriber_callback(const Imu::SharedPtr msg);

    public:
        ImuFrameIDConverterNode(): Node("imu_frame_id_converter")
        {
            this->declare_parameter("frame_id", "base_footprint_ekf");
            this->declare_parameter("subscription_topic", "imu/data_raw");
            this->declare_parameter("publisher_topic", "imu_ekf");

            node_subscriber = this->create_subscription<Imu>(
                this->get_parameter("subscription_topic").as_string(),
                rclcpp::QoS(rclcpp::SensorDataQoS()),
                [this](const Imu::SharedPtr msg){this->subscriber_callback(msg);}
            );

            node_publisher = this->create_publisher<Imu>(
                this->get_parameter("publisher_topic").as_string(),
                rclcpp::QoS(rclcpp::SensorDataQoS())
            );
        }
};

void ImuFrameIDConverterNode::subscriber_callback(const Imu::SharedPtr msg)
{
    auto new_msg = *msg;
    new_msg.header.frame_id = this->get_parameter("frame_id").as_string();
    this->node_publisher->publish(new_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuFrameIDConverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


