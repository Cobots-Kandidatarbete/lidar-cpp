#include "rclcpp/rclcpp.hpp"
#include "custom/srv/lidar_service.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lidar_client");
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl", 10);
    rclcpp::Client<custom::srv::LidarService>::SharedPtr client = node->create_client<custom::srv::LidarService>("lidar_service");
    auto request = std::make_shared<custom::srv::LidarService::Request>();
    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interupted while waiting for service");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting ...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        publisher->publish(result.get()->pcl_response);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Published");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }

    rclcpp::shutdown();
    return 0;
}