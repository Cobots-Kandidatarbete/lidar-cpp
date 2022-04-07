from custom.msg import LidarMessage

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import matplotlib.pyplot as plt
from probreg import cpd


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
    box = box.random_down_sample(sampling_ratio=2000/len(np.asarray(box.points)))

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


def find_blue_box(bounding_boxes):
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

        o3d.visualization.draw_geometries(geometries)


class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')

        # This is for visualization of the received point cloud.
        self.o3d_pcd = o3d.geometry.PointCloud()

        # Set up a subscription to the 'pcd' topic with a callback to the
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            LidarMessage,    # Msg type
            'pcl',                      # topic
            self.listener_callback,      # Function to call
            10                          # QoS
        )

    def listener_callback(self, msg):
        # Here we convert the 'msg', which is of the type PointCloud2.
        # I ported the function read_points2 from
        # the ROS1 package.
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py

        pcd = np.array(list(dangerzone.read_points(msg.pcl_response)))
        pcd = filter_pointcloud(pcd, max_distance=1.5)

        self.o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd[:, :3]))

        #plane_model, inliers = self.o3d_pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
        #self.o3d_pcd = self.o3d_pcd.select_by_index(inliers, invert=True)

        
        max_label, labels, clusters, bounding_boxes= process_clusters(self.o3d_pcd)
        blue_box_index, bound_blue_box = find_blue_box(bounding_boxes)


        visualize_scene(self.o3d_pcd, max_label, labels, bound_blue_box)

        if bound_blue_box is None:
            return

        blue_box_cluster = clusters[blue_box_index]

        # https://pypi.org/project/probreg/
        print("Starting coherent point drift...")
        # TODO Fix Coherent point drift and remove max_iter
        cpd_reg = cpd.registration_cpd(source=self.o3d_pcd, target=blue_box_cluster, maxiter=5)
        print(cpd_reg)
        print("Coherent point drift finished")
        # TODO Perform Coherent point drift on blue box and bo model


def main(args=None):
    rclpy.init(args=args)
    pcd_listener = PCDListener()

    rclpy.spin(pcd_listener)

    pcd_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
