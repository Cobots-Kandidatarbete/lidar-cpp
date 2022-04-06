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
#include <pcl/filters/extract_indices.h>

using pt_t = pcl::PointXYZRGB;
using pcl_t = pcl::PointCloud<pt_t>;

struct RGB {
    int r;
    int b;
    int g;

    void print() {
        std::cout << r << "," << g << "," << b << std::endl;
    }
};

struct HSV {
    float h;
    float s;
    float v;

    void print() {
        std::cout << h << "," << s << "," << v << std::endl;
    }
};

bool feq(float a, float b) {
    return fabs(a - b) < FLT_EPSILON;
}

void rgb_to_hsv(const RGB rgb, HSV &hsv) {
    // H IS IN DEGREES

    float r = rgb.r / 255.0;
    float g = rgb.g / 255.0;
    float b = rgb.b / 255.0;
    float c_max = MAX(MAX(r, g), b); 
    float c_min = MIN(MIN(r, g), b);

    hsv.v = c_max;

    float delta = c_max - c_min;

    if (feq(delta, 0)) 
    {
        hsv.h = 0;
        hsv.s = 0;
        return;
    }

    if (c_max > 0.0)
    {
        hsv.s = delta / c_max;
    }
    else {
        hsv.s = 0;
        hsv.h = NAN;
        return;
    }

    if (feq(r, c_max)) 
    {
        hsv.h = (g - b) / delta;
    }
    else if (feq(g, c_max)) 
    {
        hsv.h = 2.0 + (b - r) / delta;
    }
    else {
        hsv.h = 4.0 + (r - g) / delta;
    }

    hsv.h *= 60;

    if (hsv.h < 0.0)
    {
        hsv.h += 360;
    }
}

bool is_blue(const HSV hsv) {
    // Blue is in the range 120-180
    return hsv.h > 120 && hsv.h < 180 && hsv.s > 0.3 && hsv.v > 0.5;  
}

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

std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();
    
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}

void take_picture(const std::shared_ptr<custom::srv::LidarService::Request> request, const std::shared_ptr<custom::srv::LidarService::Response> response)
{
    /* Description: Get the exakt pose of a box
     *
     * The algorithm works as follows:
     * - Get rgb and depth maps from Lidar
     * - Generate point cloud from given data
     * - Generate XYZRGB point cloud from all points
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

    // Warm up camera
    for (auto i {0}; i < 100; ++i)
        pipeline.wait_for_frames();


    rs2::frameset frames{pipeline.wait_for_frames()};
    rs2::align align_to{RS2_STREAM_COLOR};
    frames = align_to.process(frames);
    auto depth_frame{frames.get_depth_frame()};
    auto color_frame{frames.get_color_frame()};

    // Create realsense point cloud and map it to color frames
    rs2::pointcloud rs_pointcloud;
    rs_pointcloud.map_to(color_frame);

    // Generate generate colored pointcloud from the old pointcloud and the depth map
    rs2::points points{rs_pointcloud.calculate(depth_frame)};
    pcl_t::Ptr pcl_pointcloud{new pcl_t};

    // TODO Explain this
    auto stream_profile{points.get_profile().as<rs2::video_stream_profile>()};
    pcl_pointcloud->width = static_cast<uint32_t>(stream_profile.width());
    pcl_pointcloud->height = static_cast<uint32_t>(stream_profile.height());
    pcl_pointcloud->is_dense = false;
    pcl_pointcloud->points.resize(points.size());

    /*
     * https://github.com/eMrazSVK/JetsonSLAM/blob/master/pcl_testing.cpp
     */

    // OpenCV Mat for showing the rgb color image, just as part of processing
    cv::Mat colorr(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", colorr);
        
    // TODO Explain this
    auto vertices{points.get_vertices()};
    auto texture_coordinates{points.get_texture_coordinates()};

    auto color_data{(uint8_t *)color_frame.get_data()};
    auto stride{color_frame.as<rs2::video_frame>().get_stride_in_bytes()};

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pt_t> extract;

    for (auto i{0}; i < points.size(); ++i)
    {
        auto pt_ptr{&pcl_pointcloud->points[i]};
        auto vex{vertices[i]};
        auto tex{texture_coordinates[i]};

        pt_ptr->x = vex.x;
        pt_ptr->y = vex.y;
        pt_ptr->z = vex.z;

        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(color_frame, tex);

        pt_ptr->r = std::get<2>(current_color);
        pt_ptr->g = std::get<1>(current_color);
        pt_ptr->b = std::get<0>(current_color);

        RGB rgb {pt_ptr->r, pt_ptr->g, pt_ptr->b};
        
        HSV hsv;
        rgb_to_hsv(rgb, hsv);

        if (!is_blue(hsv))
        {
            inliers->indices.push_back(i);
        }
    }

    std::cout << pcl_pointcloud->size() << std::endl;
    extract.setInputCloud(pcl_pointcloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*pcl_pointcloud);
    std::cout << pcl_pointcloud->size() << std::endl;

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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar service ready to get request.");


    rclcpp::spin(node);
    rclcpp::shutdown();
}