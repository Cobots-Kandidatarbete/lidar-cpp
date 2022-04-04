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

using pt_t = pcl::PointXYZRGB;
using pcl_t = pcl::PointCloud<pt_t>;

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

    // Enable Lidar camera
    rs2::config config;
    config.enable_device("f1120455");
    config.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
    config.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);

    // Get RGB and depth data from the Lidar camera
    rs2::pipeline pipeline;
    pipeline.start(config);
    rs2::frameset frames{pipeline.wait_for_frames()};
    rs2::align align_to{RS2_STREAM_COLOR};
    frames = align_to.process(frames);
    auto depth_frame{frames.get_depth_frame()};
    auto color_frame{frames.get_color_frame()};

    auto frame_width = color_frame.get_width();
    auto frame_height = color_frame.get_height();

    // Create realsense point cloud and map it to color frames
    rs2::pointcloud rs_pointcloud;
    rs_pointcloud.map_to(color_frame);

    // Generate generate colored pointcloud from the old pointcloud and the depth map
    rs2::points points{rs_pointcloud.calculate(depth_frame)};
    pcl_t::Ptr pcl_pointcloud{new pcl_t};

    // TODO Explain this
    auto stream_profile{points.get_profile().as<rs2::video_stream_profile>()};
    pcl_pointcloud->width = stream_profile.width();
    pcl_pointcloud->height = stream_profile.height();
    pcl_pointcloud->is_dense = false;
    pcl_pointcloud->points.resize(points.size());

    // TODO Explain this
    auto vertex{points.get_vertices()};
    auto texture_coordinates{points.get_texture_coordinates()};

    // TODO: Fix this abomination
    // Big haram
    // https://stackoverflow.com/questions/61921324/pointer-exception-while-getting-rgb-values-from-video-frame-intel-realsense
    auto color_data{(uint8_t *)color_frame.get_data()};
    auto stride{color_frame.as<rs2::video_frame>().get_stride_in_bytes()};

    for (auto i{0}; i < points.size(); ++i)
    {
        auto pt{pcl_pointcloud->points[i]};
        auto vex{vertex[i]};
        auto tex{texture_coordinates[i]};

        pt.x = vex.x;
        pt.y = vex.y;
        pt.z = vex.z;

        auto color_x{static_cast<int>(texture_coordinates[i].u * frame_width)};
        auto color_y{static_cast<int>(texture_coordinates[i].v * frame_height)};

        if (color_x < 0 || color_x >= frame_width || color_y < 0 || color_y >= frame_height)
            continue;

        auto color_location{(color_y * frame_width + color_x) * 3};
        pt.r = color_data[color_location];
        pt.g = color_data[color_location + 1];
        pt.b = color_data[color_location + 2];

        std::cout << pt.r << ' ' << pt.g << ' ' << pt.b << std::endl;
    }

    /*for (auto &point : pcl_pointcloud->points)
    {
        point.x = vertex->x;
        point.y = vertex->y;
        point.z = vertex->z;

        ++vertex;
    }*/

    // Get center of blue pixels in xyz
    float box_position[3];
    blue_point(box_position, color_frame, depth_frame);
    std::cout << box_position[0] << std::endl;
    std::cout << box_position[1] << std::endl;
    std::cout << box_position[2] << std::endl;

    // Create message
    custom::msg::LidarMessage lidar_message;

    // Convert to ROS-compatible type
    pcl::PCLPointCloud2 pcl_points;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    pcl::toPCLPointCloud2(*pcl_pointcloud, pcl_points);
    pcl_conversions::fromPCL(pcl_points, pointcloud_msg);
    pointcloud_msg.header.frame_id = "L515";

    geometry_msgs::msg::Vector3 blue_center;
    blue_center.x = box_position[0];
    blue_center.y = box_position[1];
    blue_center.z = box_position[2];

    lidar_message.pcl_response = pointcloud_msg;
    lidar_message.blue_center = blue_center;
    response->data = lidar_message;

    pipeline.stop();
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