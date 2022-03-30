#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "librealsense2/rs.hpp" // Include RealSense Cross Platform API
#include "librealsense2/rsutil.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "custom/srv/lidar_service.hpp"
#include "custom/msg/lidar_message.hpp"
#include <opencv4/opencv2/opencv.hpp>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

void blue_point(float box_position[3], const rs2::video_frame video, const rs2::depth_frame depth)
{
    const int w = video.get_width();
    const int h = video.get_height();
    cv::Mat image{cv::Size{w, h}, CV_8UC3, (void *)video.get_data(), cv::Mat::AUTO_STEP};
    cv::Mat depth_image{cv::Size{w, h}, CV_16SC1, (void *)depth.get_data()};
    cv::Mat hsv, mask, non_zero;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar{100, 150, 0}, cv::Scalar{140, 255, 255}, mask);
    cv::findNonZero(mask, non_zero);
    cv::Scalar mean = cv::mean(non_zero);

    auto intrinsics{depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics()};
    float m[2] = {static_cast<float>(mean[0]), static_cast<float>(mean[1])};

    rs2_deproject_pixel_to_point(box_position, &intrinsics, m, depth.get_distance(static_cast<int>(mean[0]), static_cast<int>(mean[1])));
}

void take_picture(const std::shared_ptr<custom::srv::LidarService::Request> request, const std::shared_ptr<custom::srv::LidarService::Response> response)
{
    /* Description: Get the exakt pose of a box
     *
     * The algorithm works as follows:
     * - Get rgb and depth maps from Lidar
     * - Generate point cloud from given data
     * - Generate XYZHSV point cloud from all points
     * - Filter the points that are not blue
     * - Cluster blue points
     * - Select cluster of box (optional)
     * - Perform coherent point drift on cluster
     */

    // OBS: Current code does not implement the algorithm above, it will be replaced.

    rs2::pipeline pipe;
    rs2::pointcloud pc;
    rs2::points points;
    rs2::config cfg;

    // Enable lidar
    cfg.enable_device("f1120455");
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
    cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);

    pipe.start(cfg);
    rs2::frameset frames{pipe.wait_for_frames()};
    rs2::align align_to{RS2_STREAM_COLOR};
    frames = align_to.process(frames);
    auto depth = frames.get_depth_frame();
    auto video = frames.get_color_frame();

    float box_position[3];
    // Get center of blue pixels in xyz
    blue_point(box_position, video, depth);
    std::cout << box_position[0] << std::endl;
    std::cout << box_position[1] << std::endl;
    std::cout << box_position[2] << std::endl;

    points = pc.calculate(depth);
    pcl::PCLPointCloud2 pcl_points;

    // Points to pcl
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

    pcl::toPCLPointCloud2(*cloud, pcl_points);

    custom::msg::LidarMessage message;

    sensor_msgs::msg::PointCloud2 pointcloud;
    pcl_conversions::fromPCL(pcl_points, pointcloud);
    pointcloud.header.frame_id = "L515";

    geometry_msgs::msg::Vector3 blue_center;
    blue_center.x = box_position[0];
    blue_center.y = box_position[1];
    blue_center.z = box_position[2];

    message.pcl_response = pointcloud;
    message.blue_center = blue_center;
    response->data = message;
    pipe.stop();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lidar_service_server");
    rclcpp::Service<custom::srv::LidarService>::SharedPtr service =
        node->create_service<custom::srv::LidarService>("lidar_service", &take_picture);
    rclcpp::spin(node);
    rclcpp::shutdown();
}