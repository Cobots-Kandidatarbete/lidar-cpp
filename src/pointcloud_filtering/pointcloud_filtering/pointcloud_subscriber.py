from urllib import response
from custom.msg import LidarMessage

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import matplotlib.pyplot as plt
from probreg import cpd

import copy
import struct


import numpy as np
import open3d as o3d

from . import dangerzone

import os


def get_square_dist(pcd: np.ndarray) -> np.ndarray:
    return np.sum(np.square(pcd[:, :3]), axis=1)


def filter_points_outside_distance(pcd: np.ndarray, distance: float) -> np.ndarray:
    return pcd[get_square_dist(pcd) < (distance * distance)]


def filter_pointcloud(pcd: np.ndarray, max_distance: float) -> np.ndarray:
    n_points_prefilter = pcd.shape[0]

    pcd_filtered = filter_points_outside_distance(pcd, max_distance)

    n_points_postfilter = pcd_filtered.shape[0]
    print(f"Filtered all points beyond {max_distance}. Removed {n_points_prefilter - n_points_postfilter} points.")

    return pcd_filtered


def get_box_model():
    ws_path = '/home/student/kandidat/lidar-ws'
    if not os.path.isdir(ws_path):
        ws_path = os.getcwd()

    box_path_rel = '/src/pointcloud_filtering/pointcloud_filtering/box.ply'
    box_path_full = ws_path + box_path_rel
    if not os.path.isfile(box_path_full):
        raise FileNotFoundError("Could not file box file. Make sure the program is run from workspace")               

    box = o3d.io.read_point_cloud(box_path_full)
    box = box.random_down_sample(sampling_ratio=1500/len(np.asarray(box.points)))

    return box


def process_clusters(pcd):
    clusters = []
    bounding_boxes = []

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=15, print_progress=True))

    max_label = labels.max()

    if max_label > -1:
        for i in range(0, max_label + 1):
            cluster_index = np.where(labels == i)[0]
            clusters.append(pcd.select_by_index(cluster_index))
            try:
                bounding_boxes.append(clusters[i].get_oriented_bounding_box())
            except:
                print(f"Attempted to add bounding box of cluster {i} but failed.")
                bounding_boxes.pop()
            

    print(f"Found {max_label + 1} clusters in the Pointcloud")
    return max_label, labels, clusters, bounding_boxes


def find_largest_cluster(bounding_boxes):
    largest_volume = 0
    largest_index = -1

    for index, box in enumerate(bounding_boxes):
        volume = box.volume()

        if volume > largest_volume:
            largest_index, largest_volume = index, volume

    return largest_index, bounding_boxes[largest_index]



def find_blue_box_old(bounding_boxes):
    bounds = np.array([0.46235329, 0.36464842, 0.18794718])
    lower_margin = np.array([0.05, 0.05, 0.05])
    upper_margin = np.array([0.15, 0.15, 0.15])

    volume = 0.018781736409474
    volume_margin = 0.04

    lower_bound = bounds - lower_margin
    upper_bound = bounds + upper_margin
    lower_volume = volume - volume_margin
    upper_volume = volume + volume_margin

    blue_boxes = []
    blue_box_indexes = []
    for index, box in enumerate(bounding_boxes):
        volume = box.volume()
        extent = box.extent

        if np.all(lower_bound < extent) and np.all(extent < upper_bound) and lower_volume < volume < upper_volume:
            blue_boxes.append(box)
            blue_box_indexes.append(index)
    
    n_found = len(blue_boxes)

    if n_found > 1:
        print(f"Found {len(blue_boxes)} blue boxes, selecting first one")
        return blue_box_indexes[0], blue_boxes[0]

    elif n_found == 1:
        print("Found one blue box")
        return blue_box_indexes[0], blue_boxes[0]

    else:
        print("No blue boxes found")
        return None, None


def visualize_scene(pcd, max_label, labels, blue_box=None):
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

        geometries = [pcd, mesh_frame]

        if blue_box is not None:
            geometries.append(blue_box)
            print(f"Blue bounding box volume: {blue_box.volume()}")

        o3d.visualization.draw_geometries(geometries)


def get_cloud(msg, use_example=False, store_example=False):
    print("Getting cloud")
    o3d_pcd = None    
    path = "./src/pointcloud_filtering/pointcloud_filtering/example_box_cloud.pcd"

    if use_example:
        try:
            o3d_pcd = o3d.io.read_point_cloud(path)
        except:
            print("Failed to read example.")

    else:
        pcd = np.array(list(dangerzone.read_points(msg.pcl_response)))
        pcd = filter_pointcloud(pcd, max_distance=1.5)
        o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd[:, :3]))

        if store_example:
            o3d.io.write_point_cloud(path, o3d_pcd)
            
    return o3d_pcd


def numpy_to_ros2_pcl(points, parent_frame):
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = sensor_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]), 
        data=data
    )
 


class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')

        self.o3d_pcd = o3d.geometry.PointCloud()
        self.pcd_subscriber = self.create_subscription(LidarMessage, 'pcl', self.listener_callback, 10)

    def listener_callback(self, msg):
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py

        #pcd = np.array(list(dangerzone.read_points(msg.pcl_response)))
        #pcd = filter_pointcloud(pcd, max_distance=1.5)
        #o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd[:, :3]))

        o3d_pcd = get_cloud(msg, use_example=True)

        #o3d.visualization.draw_geometries([o3d_pcd]) 

        #plane_model, inliers = self.o3d_pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
        #self.o3d_pcd = self.o3d_pcd.select_by_index(inliers, invert=True)
        
        max_label, labels, clusters, bounding_boxes= process_clusters(o3d_pcd)
        blue_box_index, bound_blue_box = find_largest_cluster(bounding_boxes)

        #slim_pcd = self.o3d_pcd.voxel_down_sample(voxel_size=0.05)

        #visualize_scene(o3d_pcd, max_label, labels, bound_blue_box)

        if bound_blue_box is None:
            return

        blue_box_cluster = clusters[blue_box_index]
        blue_box_cluster_slim = blue_box_cluster.voxel_down_sample(voxel_size=0.03)
        blue_box_model = get_box_model()

        #blue_box_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
        #blue_box_model = blue_box_model.rotate(blue_box_rotation)

        blue_box_model = blue_box_model.scale(1.5E-3, blue_box_model.get_center()).translate(blue_box_cluster.get_center(), relative=False)
        #o3d.visualization.draw_geometries([blue_box_cluster_slim])
        o3d.visualization.draw_geometries([blue_box_model])

        # TODO Perform Coherent point drift on blue box and bo model
        # https://pypi.org/project/probreg/
        print("Starting coherent point drift...")
        # TODO Fix Coherent point drift and remove max_iter
        try:
            tf_param, _, _ = cpd.registration_cpd(source=blue_box_model, target=blue_box_cluster_slim, tf_type_name='affine', maxiter=10000)
            print("Coherent point drift finished")
        except:
            print("Failed to do coherent point drift")
            return

        fit_box = copy.deepcopy(blue_box_model)
        fit_box.points = tf_param.transform(fit_box.points) # Todo see if necessary

        blue_box_cluster_slim.paint_uniform_color([1, 0, 0])
        fit_box.paint_uniform_color([0, 1, 0])

        print("Showing CPD results. Red is before and green is after.")
        o3d.visualization.draw_geometries([blue_box_cluster_slim, blue_box_cluster_slim.get_oriented_bounding_box()])
        o3d.visualization.draw_geometries([fit_box])
        o3d.visualization.draw_geometries([blue_box_cluster_slim, fit_box])

        print(f"Volume before: {blue_box_cluster_slim.get_oriented_bounding_box().volume()}, volume after: {fit_box.get_oriented_bounding_box().volume()}")
        print(f"Center before: {blue_box_cluster_slim.get_oriented_bounding_box().get_center()}, center after: {fit_box.get_oriented_bounding_box().get_center()}")


        print("Returning pointcloud")
        fit_bounding_box = fit_box.get_oriented_bounding_box()
        o3d.visualization.draw_geometries([fit_box, fit_bounding_box])

        # ros2_pcl = numpy_to_ros2_pcl()
        

        



def main(args=None):
    rclpy.init(args=args)
    pcd_listener = PCDListener()

    rclpy.spin(pcd_listener)

    pcd_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()