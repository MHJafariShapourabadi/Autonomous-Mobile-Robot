#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using sensor_msgs::msg::PointCloud2;

class PointcloudFrameIDConverterNode : public rclcpp::Node
{
    private:
        rclcpp::Subscription<PointCloud2>::SharedPtr node_subscriber;
        rclcpp::Publisher<PointCloud2>::SharedPtr node_publisher;
        void subscriber_callback(const PointCloud2::SharedPtr msg);

    public:
        PointcloudFrameIDConverterNode(): Node("pointcloud_frame_id_converter")
        {
            this->declare_parameter("frame_id", "lidar_link");
            this->declare_parameter("subscription_topic", "/lidar/pointcloud/points");
            this->declare_parameter("publisher_topic", "/lidar/pointcloud/points/corrected");

            node_subscriber = this->create_subscription<PointCloud2>(
                this->get_parameter("subscription_topic").as_string(),
                rclcpp::QoS(rclcpp::SensorDataQoS()),
                [this](const PointCloud2::SharedPtr msg){this->subscriber_callback(msg);}
            );

            node_publisher = this->create_publisher<PointCloud2>(
                this->get_parameter("publisher_topic").as_string(),
                rclcpp::QoS(rclcpp::SensorDataQoS())
            );
        }
};

void PointcloudFrameIDConverterNode::subscriber_callback(const PointCloud2::SharedPtr msg)
{
    auto new_msg = *msg;
    new_msg.header.frame_id = this->get_parameter("frame_id").as_string();
    this->node_publisher->publish(new_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointcloudFrameIDConverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


