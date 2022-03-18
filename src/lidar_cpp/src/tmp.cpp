#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "librealsense2/rs.hpp" // Include RealSense Cross Platform API
#include "librealsense2/rsutil.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>

#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/visualization/cloud_viewer.h>
#include <opencv4/opencv2/opencv.hpp>
#include <cpd/rigid.hpp>

using namespace std::chrono_literals;
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

struct Picture
{
  pcl_ptr cloud;
  rs2::video_frame video;
  rs2::depth_frame depth;
};

pcl_ptr points_to_pcl(const rs2::points &points)
{
  pcl_ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};

  const rs2::video_stream_profile sp = points.get_profile().as<rs2::video_stream_profile>();
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

Picture take_picture()
{
  rs2::pipeline pipe;

  const rs2::pointcloud pc;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
  config.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);
  pipe.start(config);

  rs2::frameset frames{pipe.wait_for_frames()};
  rs2::align align_to{RS2_STREAM_COLOR};
  frames = align_to.process(frames);

  const rs2::video_frame color{frames.get_color_frame()};
  const rs2::depth_frame depth{frames.get_depth_frame()};

  const rs2::points points{pc.calculate(depth)};
  const pcl_ptr pcl_points{points_to_pcl(points)};
  pipe.stop();

  return Picture{pcl_points, color, depth};
}
pcl_ptr remove_plane(pcl_ptr pcl_points)
{
  pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg{};
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory!
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.02);

  seg.setInputCloud(pcl_points);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(pcl_points);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*pcl_points);

  return pcl_points;
}
void visualize_pcl(Picture &picture)
{
  float box_position[3];
  blue_point(box_position, picture.video, picture.depth);
  pcl_ptr blue_box{new pcl::PointCloud<pcl::PointXYZ>};
  blue_box->push_back(pcl::PointXYZ{box_position[0], box_position[1], box_position[2]});
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(picture.cloud, 0, 255, 0);
  pcl_ptr filtered{new pcl::PointCloud<pcl::PointXYZ>};
  for (auto p : picture.cloud->points)
  {
    float dist{(p.x - box_position[0]) * (p.x - box_position[0]) + (p.y - box_position[1]) * (p.y - box_position[1]) + (p.z - box_position[2]) * (p.z - box_position[2])};
    if (dist <= 0.16)
    {
      filtered->push_back(p);
    }
  }

  pcl_ptr box{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::io::loadPLYFile("/home/student/lidar-ws/src/lidar_cpp/src/box.ply", *box);
  pcl::RandomSample<pcl::PointXYZ> rand{};
  rand.setInputCloud(box);
  rand.setSample(1000);
  rand.filter(*box);

  rand.setInputCloud(filtered);
  rand.filter(*filtered);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(box);
  icp.setInputTarget(filtered);

  pcl_ptr filtered_icp{new pcl::PointCloud<pcl::PointXYZ>};
  icp.align(*filtered_icp);

  cpd::Matrix m;

  viewer->addPointCloud(box, single_color, "m");
  viewer->addPointCloud(filtered_icp, "box");
  viewer->addPointCloud(filtered, single_color, "main");

  viewer->initCameraParameters();

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Picture picture{take_picture()};
  remove_plane(picture.cloud);
  visualize_pcl(picture);
  rclcpp::shutdown();
}