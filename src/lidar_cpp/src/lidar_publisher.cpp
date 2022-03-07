#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "librealsense2/rs.hpp" // Include RealSense Cross Platform API
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LidarPublisher : public rclcpp::Node
{
public:
    LidarPublisher()
        : Node("lidar_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl", 10);
        timer_ = this->create_wall_timer(
            5s, std::bind(&LidarPublisher::timer_callback, this));
    }

private:
    using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
    pcl_ptr points_to_pcl(const rs2::points &points)
    {
        pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        cloud->width = sp.width();
        cloud->height = sp.height();
        cloud->is_dense = false;
        cloud->points.resize(points.size());
        auto ptr = points.get_vertices();
        for (auto &p : cloud->points)
        {
            p.x = ptr->x;
            p.y = ptr->y;
            p.z = ptr->z;
            ptr++;
        }

        return cloud;
    }
    void timer_callback()
    {
        rs2::pipeline pipe;
        rs2::pointcloud pc;
        rs2::points points;
        pipe.start();
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        points = pc.calculate(depth);
        pcl::PCLPointCloud2 pcl_points;
        pcl::toPCLPointCloud2(*points_to_pcl(points), pcl_points);

        sensor_msgs::msg::PointCloud2 message;
        pcl_conversions::fromPCL(pcl_points, message);
        message.header.frame_id = "world";

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing pcl");
        pipe.stop();
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPublisher>());
    rclcpp::shutdown();
    return 0;
}