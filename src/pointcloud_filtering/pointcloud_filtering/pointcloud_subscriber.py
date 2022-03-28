from sensor_msgs.msg import PointCloud2, PointField
import struct
import math
import ctypes
from collections import namedtuple
import sys
import os

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import matplotlib.pyplot as plt
from probreg import cpd


import numpy as np
import open3d as o3d


class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')

        # This is for visualization of the received point cloud.
        self.o3d_pcd = o3d.geometry.PointCloud()

        # Set up a subscription to the 'pcd' topic with a callback to the
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            'pcl',                      # topic
            self.listener_callback,      # Function to call
            10                          # QoS
        )

    def listener_callback(self, msg):
        # Here we convert the 'msg', which is of the type PointCloud2.
        # I ported the function read_points2 from
        # the ROS1 package.
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py

        pcd_as_numpy_array = np.array(list(read_points(msg)))
        dist = 1
        filtered = np.array(
            [row for row in pcd_as_numpy_array if row[0]**2 + row[1]**2 + row[2]**2 < dist**2])

        # The rest here is for visualization.
        self.o3d_pcd = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(filtered))

        plane_model, inliers = self.o3d_pcd.segment_plane(distance_threshold=0.02,
                                                          ransac_n=3,
                                                          num_iterations=1000)

        self.o3d_pcd = self.o3d_pcd.select_by_index(inliers, invert=True)

        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                self.o3d_pcd.cluster_dbscan(eps=0.05, min_points=10, print_progress=True))

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
        dy = 0.38135528
        dz = 0.24068656
        volume = 0.018781736409474
        verror = 0.04
        error = 0.15

        for box in boxes:
            print('----------Box----------')
            print(np.array(box.volume()))
            print(np.array(box.extent))
            if dx-error < box.extent[0] < dx + error and dy-error < box.extent[1] < dy + error and dz-error < box.extent[2] < dz + error and volume - verror < box.volume() < volume + verror:
                blue_boxes.append(box)
        o3d.visualization.draw_geometries(
            [self.o3d_pcd, mesh_frame] + blue_boxes)


# The code below is "ported" from
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
# I'll make an official port and PR to this repo later:
# https://github.com/ros2/common_interfaces

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(
        cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print(
                'Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


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