#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "librealsense2/rs.hpp" // Include RealSense Cross Platform API
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

using namespace std::chrono_literals;
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

unsigned int text_id = 0;

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                     void* viewer_void)
{
  pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton && event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
    char str[512];

    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}




pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

void take_picture()
{
    rs2::pipeline pipe;
    rs2::pointcloud pc;
    rs2::points points;
    pipe.start();
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);
    auto pcl_points = points_to_pcl(points);
    pipe.stop();


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.02);

    seg.setInputCloud (pcl_points);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(pcl_points);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*pcl_points);

    pcl_ptr transformed_cloud;

    Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector, rotation_vector;

    floor_plane_normal_vector[1] = coefficients->values[1];
    floor_plane_normal_vector[0] = coefficients->values[0];
    floor_plane_normal_vector[2] = coefficients->values[2];
    
    xy_plane_normal_vector[0] = 0.0;
    xy_plane_normal_vector[1] = 0.0;
    xy_plane_normal_vector[2] = 1.0;
    
    rotation_vector = xy_plane_normal_vector.cross (floor_plane_normal_vector);
    
    float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    transform_2.rotate (Eigen::AngleAxisf (theta, rotation_vector));
    
    pcl::transformPointCloud (*pcl_points, *transformed_cloud, transform_2);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addPointCloud(pcl_points, "main");
    viewer->addCoordinateSystem (1.0);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    take_picture();
    rclcpp::shutdown();
}