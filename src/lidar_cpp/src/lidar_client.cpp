#include "rclcpp/rclcpp.hpp"
#include "custom/srv/lidar_service.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lidar_client");
    rclcpp::Client<custom::srv::LidarService>::SharedPtr client = node->create_client<custom::srv::LidarService>("lidar_service");
    auto request = std::make_shared<custom::srv::LidarService::Request>();
}