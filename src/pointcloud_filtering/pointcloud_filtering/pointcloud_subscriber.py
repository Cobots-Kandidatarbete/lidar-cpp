from custom.msg import LidarMessage

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import matplotlib.pyplot as plt
from probreg import cpd


import numpy as np
import open3d as o3d

from . import dangerzone


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

        pcd_as_numpy_array = np.array(
            list(dangerzone.read_points(msg.pcl_response)))

        print(pcd_as_numpy_array[:, 3])

        dist = 1
        filtered = np.array(
            [row for row in pcd_as_numpy_array if row[0]**2 + row[1]**2 + row[2]**2 < dist**2])

        self.o3d_pcd = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(filtered[:, :3]))

        plane_model, inliers = self.o3d_pcd.segment_plane(distance_threshold=0.02,
                                                          ransac_n=3,
                                                          num_iterations=1000)

        self.o3d_pcd = self.o3d_pcd.select_by_index(inliers, invert=True)

        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                self.o3d_pcd.cluster_dbscan(eps=0.05, min_points=20, print_progress=True))

        max_label = labels.max()
        clusters = []
        boxes = []

        if max_label > -1:
            for i in range(0, max_label + 1):
                cluster_index = np.where(labels == i)[0]
                clusters.append(self.o3d_pcd.select_by_index(
                    cluster_index))
                try:
                    boxes.append(clusters[i].get_oriented_bounding_box())
                except:
                    print("Oj")

        print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(
            labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        self.o3d_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

        box = o3d.io.read_point_cloud(
            '/home/student/kandidat/lidar-ws/src/pointcloud_filtering/pointcloud_filtering/box.ply')
        box = box.random_down_sample(
            sampling_ratio=2000/len(np.asarray(box.points)))

        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.1, origin=[0, 0, 0])
        blue_boxes = []
        dx = 0.46235329
        dy = 0.36464842
        dz = 0.18794718
        errorxd = 0.05
        erroryd = 0.05
        errorzd = 0.05
        errorxu = 0.15
        erroryu = 0.15
        errorzu = 0.15
        volume = 0.018781736409474
        verror = 0.04

        for box in boxes:
            print('----------Box----------')
            print(np.array(box.volume()))
            print(np.array(box.extent))
            if dx-errorxd < box.extent[0] < dx + errorxu and dy-erroryd < box.extent[1] < dy + erroryu and dz-errorzd < box.extent[2] < dz + errorzu and volume - verror < box.volume() < volume + verror:
                blue_boxes.append(box)

        o3d.visualization.draw_geometries(
            [self.o3d_pcd, mesh_frame] + blue_boxes)


def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
