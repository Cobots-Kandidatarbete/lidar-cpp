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


bool feq(float a, float b) {
    return fabs(a - b) < FLT_EPSILON;
}

struct HSV {
    float h;
    float s;
    float v;

    void print() const {
        std::cout << h << "," << s << "," << v << std::endl;
    }

    bool is_blue() {
        return h > 200 && h < 280 && s > 0.3 && v > 0.5;  
    }
};

struct RGB {
    float r;
    float g;
    float b;

    RGB(float r, float g, float b) {
        this->r = r / 255.0;
        this->g = g / 255.0;
        this->b = b / 255.0;
    }

    void print() const {
        std::cout << r << "," << g << "," << b << std::endl;
    }

    HSV to_hsv() const {
        HSV hsv;
    
        auto c_max = MAX(MAX(r, g), b); 
        auto c_min = MIN(MIN(r, g), b);

        hsv.v = c_max;

        float delta = c_max - c_min;

        if (feq(delta, 0)) 
        {
            hsv.h = 0;
            hsv.s = 0;
            return hsv;
        }

        if (feq(c_max, 0))
        {
            hsv.s = 0;
            hsv.h = NAN;
            return hsv;
        }

        hsv.s = delta / c_max;

        if (feq(r, c_max)) 
            hsv.h = 60 * (g - b) / delta;

        else if (feq(g, c_max)) 
            hsv.h = 60 * (2.0 + (b - r) / delta);

        else 
            hsv.h = 60 * (4.0 + (r - g) / delta);

        if (hsv.h < 0.0)
            hsv.h += 360;

        return hsv;
    }

    bool is_blue() {
        return to_hsv().is_blue();
    }
};

struct XYZ {
    float x;
    float y;
    float z;
};


RGB get_texture_color(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();
    
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    return RGB {texture_data[idx+2], texture_data[idx+1], texture_data[idx]};   // Reverse order to go from BGR to RGB
}

void write_point(pt_t *point, const XYZ coord, const RGB color) 
{
    point->x = coord.x;
    point->y = coord.y;
    point->z = coord.z;
    point->r = color.r;
    point->g = color.g;
    point->b = color.b;
}

std::tuple<pcl_t::Ptr, rs2::points, rs2::video_frame>  setup_pointcloud() {
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

    auto stream_profile{points.get_profile().as<rs2::video_stream_profile>()};
    pcl_pointcloud->width = static_cast<uint32_t>(stream_profile.width());
    pcl_pointcloud->height = static_cast<uint32_t>(stream_profile.height());
    pcl_pointcloud->is_dense = false;
    pcl_pointcloud->points.resize(points.size());

    pipeline.stop();

    return { pcl_pointcloud, points, color_frame};
}

pcl::PointIndices::Ptr process_pointcloud(pcl_t::Ptr pcl_pointcloud, rs2::points points, rs2::video_frame color_frame) {
    auto vertices{points.get_vertices()};
    auto texture_coordinates{points.get_texture_coordinates()};

    auto color_data{(uint8_t *)color_frame.get_data()};
    auto stride{color_frame.as<rs2::video_frame>().get_stride_in_bytes()};

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    for (auto i{0}; i < points.size(); ++i)
    {
        auto point_ptr{&pcl_pointcloud->points[i]};
        auto vex{vertices[i]};
        auto tex{texture_coordinates[i]};

        RGB color_rgb {get_texture_color(color_frame, tex)};
        XYZ pos_xyz {vex.x, vex.y, vex.z};

        write_point(point_ptr, pos_xyz, color_rgb);

        if (!color_rgb.is_blue())
            inliers->indices.push_back(i);
    }

    return inliers;
}

void filter_pointcloud(pcl_t::Ptr pcl_pointcloud, pcl::PointIndices::Ptr inliers) {
    pcl::ExtractIndices<pt_t> extract;
    int n_points_prefilter {pcl_pointcloud->size()};

    extract.setInputCloud(pcl_pointcloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*pcl_pointcloud);
    int n_points_postfilter {pcl_pointcloud->size()};

    std::cout << "Filtered " << n_points_prefilter - n_points_postfilter << " points." << std::endl; 
}

custom::msg::LidarMessage create_message(pcl_t::Ptr pcl_pointcloud) {
    // Create message
    custom::msg::LidarMessage lidar_message;

    // Convert to ROS-compatible type
    pcl::PCLPointCloud2 pcl_points;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    pcl::toPCLPointCloud2(*pcl_pointcloud, pcl_points);
    pcl_conversions::fromPCL(pcl_points, pointcloud_msg);
    pointcloud_msg.header.frame_id = "L515";

    lidar_message.pcl_response = pointcloud_msg;

    return lidar_message;
}



void take_picture(const std::shared_ptr<custom::srv::LidarService::Request> request, const std::shared_ptr<custom::srv::LidarService::Response> response)
{
    /* Description: Get the point cloud of a blue box (or any blue object, honestly)
     *
     * The algorithm works as follows:
     * - Get rgb and depth maps from Lidar
     * - Generate point cloud from given data
     * - Generate XYZRGB point cloud from all points
     * - Filter the points that are not blue
     * - Send the remaining point cloud in the response
     */

    // Some parts of the was inspired from https://github.com/eMrazSVK/JetsonSLAM/blob/master/pcl_testing.cpp

    const auto [pcl_pointcloud, points, color_frame] { setup_pointcloud()};
    auto inliers {process_pointcloud(pcl_pointcloud, points, color_frame)};
    filter_pointcloud(pcl_pointcloud, inliers);
    auto message {create_message(pcl_pointcloud)};

    response->data = message;
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