from custom.msg import LidarMessage

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import sensor_msgs.msg as sensor_msgs
from geometry_msgs.msg import Transform, TransformStamped
from builtin_interfaces.msg import Time
import matplotlib.pyplot as plt
from probreg import cpd

import copy


import numpy as np
import open3d as o3d

import transforms3d

from . import dangerzone

from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

import os
import sys


class StampContainer:
    stamps = []


def get_square_dist(pcd: np.ndarray) -> np.ndarray:
    return np.sum(np.square(pcd[:, :3]), axis=1)


def filter_points_outside_distance(pcd: np.ndarray, distance: float) -> np.ndarray:
    return pcd[get_square_dist(pcd) < (distance * distance)]


def filter_pointcloud(pcd: np.ndarray, max_distance: float) -> np.ndarray:
    n_points_prefilter = pcd.shape[0]

    pcd_filtered = filter_points_outside_distance(pcd, max_distance)

    n_points_postfilter = pcd_filtered.shape[0]
    print(
        f"Filtered all points beyond {max_distance}. Removed {n_points_prefilter - n_points_postfilter} points.")

    return pcd_filtered


def get_box_model():
    print("Getting box model...")
    ws_path = '/home/student/kandidat/lidar-ws'
    if not os.path.isdir(ws_path):
        ws_path = os.getcwd()
        if ws_path.split('/')[-1] != 'lidar-ws':
            ws_path += "/src/lidar-ws"
            if not os.path.isdir(ws_path):
                raise Exception(
                    "Could not find folder. Make sure program is run from workspace.")

    box_path_rel = '/src/pointcloud_filtering/pointcloud_filtering/box.ply'
    box_path_full = ws_path + box_path_rel
    if not os.path.isfile(box_path_full):
        raise FileNotFoundError(
            "Could not file box file. Make sure the program is run from workspace")

    box = o3d.io.read_point_cloud(box_path_full)
    box = box.random_down_sample(
        sampling_ratio=1200/len(np.asarray(box.points)))

    print("Returning model")
    return box


def process_clusters(pcd):
    clusters = []
    bounding_boxes = []

    print("Computing clusters...")
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(
            eps=0.02, min_points=15, print_progress=True))
    print("Finished computing clusters")

    max_label = labels.max()

    print("Parsing clusters...")
    if max_label > -1:
        for i in range(0, max_label + 1):
            print(f"\tGetting box at cluster {i}")
            cluster_index = np.where(labels == i)[0]
            clusters.append(pcd.select_by_index(cluster_index))
            try:
                bounding_box = clusters[i].get_oriented_bounding_box()
                bounding_boxes.append(bounding_box)
            except:
                print(f"Failed to get box at cluster {i}")
                clusters.pop()

    print(
        f"Finished parsing clusters. Found {max_label + 1} clusters in the Pointcloud")
    return max_label, labels, clusters, bounding_boxes


def find_largest_cluster(bounding_boxes):
    print("Finding blue box cluster")
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
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.1, origin=[0, 0, 0])
    colors = plt.get_cmap("tab20")(
        labels / (max_label if max_label > 0 else 1))
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
            print("Reading cloud from file")
            o3d_pcd = o3d.io.read_point_cloud(path)
        except:
            print("Failed to read example.")

    else:
        print("Capturing cloud from camera")
        pcd = np.array(list(dangerzone.read_points(msg.pcl_response)))
        print("Successfully read cloud")
        pcd = filter_pointcloud(pcd, max_distance=1.5)
        o3d_pcd = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(pcd[:, :3]))

        if store_example:
            print("Saving cloud to file")
            o3d.io.write_point_cloud(path, o3d_pcd)

    print("Using cloud")
    return o3d_pcd


def numpy_to_ros2_pcl(points, parent_frame):
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

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
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


class PCDListener(Node):

    def __init__(self, use_example=False):
        super().__init__('pcd_subscriber_node')

        self.o3d_pcd = o3d.geometry.PointCloud()
        self.pcd_subscriber = self.create_subscription(
            LidarMessage, 'pcl', self.listener_callback, 10)
        self.use_example = use_example
        #self.tf_publisher = self.create_publisher(TFMessage, "/tf", 20)
        #self.timer = self.create_timer(0.1, self.timer_callback)
        #self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("PCD listener started")

    def listener_callback(self, msg):
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py

        #pcd = np.array(list(dangerzone.read_points(msg.pcl_response)))
        #pcd = filter_pointcloud(pcd, max_distance=1.5)
        #o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd[:, :3]))

        o3d_pcd = get_cloud(
            msg, use_example=self.use_example, store_example=False)

        # o3d.visualization.draw_geometries([o3d_pcd])

        #plane_model, inliers = self.o3d_pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
        #self.o3d_pcd = self.o3d_pcd.select_by_index(inliers, invert=True)

        max_label, labels, clusters, bounding_boxes = process_clusters(o3d_pcd)
        blue_box_index, bound_blue_box = find_largest_cluster(bounding_boxes)

        #slim_pcd = self.o3d_pcd.voxel_down_sample(voxel_size=0.05)

        visualize_scene(o3d_pcd, max_label, labels, bound_blue_box)

        if bound_blue_box is None:
            return

        blue_box_cluster = clusters[blue_box_index]
        #blue_box_cluster_slim = blue_box_cluster.voxel_down_sample(voxel_size=0.03)
        blue_box_cluster_slim = blue_box_cluster.random_down_sample(0.025)
        blue_box_model = get_box_model()

        #blue_box_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
        #blue_box_model = blue_box_model.rotate(blue_box_rotation)

        box_p1 = np.array([150, 200, 200])
        box_p2 = np.array([150, -200, 200])
        box_p3 = np.array([-150, -200, 200])
        box_p4 = np.array([-150, 200, 200])
        box_p5 = np.array([150, 200, 0])
        box_p6 = np.array([150, -200, 0])
        box_p7 = np.array([-150, -200, 0])
        box_p8 = np.array([-150, 200, 0])

        corner_points = np.array([box_p1, box_p2, box_p3, box_p4])

        corner_cloud = o3d.geometry.PointCloud()
        corner_cloud.points = o3d.utility.Vector3dVector(corner_points)

        blue_box_model.paint_uniform_color([0, 0, 1])
        corner_cloud.paint_uniform_color([1, 0, 0])

        o3d.visualization.draw_geometries([blue_box_cluster_slim])
        #o3d.visualization.draw_geometries([blue_box_model, corner_cloud])

        rotation_matrix2 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

        center = blue_box_model.get_center()

        blue_box_model = blue_box_model.scale(1E-3, center)
        corner_cloud = corner_cloud.scale(1E-3, center)

        center_diff = blue_box_model.get_center()-corner_cloud.get_center()

        blue_box_model = blue_box_model.translate(
            blue_box_cluster.get_center(), relative=False).rotate(rotation_matrix2)
        corner_cloud = corner_cloud.translate(
            blue_box_cluster.get_center(), relative=False).translate(center_diff).rotate(rotation_matrix2)

        #o3d.visualization.draw_geometries([blue_box_model, corner_cloud])
        fit_box, _, _, rotation_matrix = self.fit_box_cpd(
            blue_box_cluster_slim, blue_box_model, corner_cloud)

        print("Returning pointcloud")

        translation = fit_box.get_center()
        transform = self.get_transform(
            rotation_matrix=rotation_matrix, translation=translation)

        self.publish_transform(transform, 0)

    def fit_box_cpd(self, blue_box_cluster_slim, blue_box_model, corner_points):
        # TODO Perform Coherent point drift on blue box and bo model
        # https://pypi.org/project/probreg/
        print("Starting coherent point drift...")

        # TODO Fix Coherent point drift and remove max_iter
        try:
            tf_param, sigma2, q = cpd.registration_cpd(
                source=blue_box_model, target=blue_box_cluster_slim, tf_type_name='rigid', maxiter=40)
            print("Coherent point drift finished")
            print("Sigma2:", sigma2)
            print("q:", q)
        except:
            print("Failed to do coherent point drift")
            return

        print("Beginning transformations...")
        print("\tFix box:", blue_box_model)
        print("\tcorner points:", corner_points)
        fit_box = copy.deepcopy(blue_box_model)
        fit_corners = copy.deepcopy(corner_points)
        print("\tTransforming box model")
        fit_box.points = tf_param.transform(fit_box.points)
        print("\tFinished box model transformation successfully")
        print("\tTransforming corner reference points...")
        fit_corners.points = tf_param.transform(fit_corners.points)
        print("\tFinished transforming corner points")
        print("Finished transformations")

        print("Getting rotation matrix...")
        line1 = fit_corners.points[0] - fit_corners.points[1]
        line2 = fit_corners.points[0] - fit_corners.points[2]
        print(fit_corners)
        dx = fit_corners.points[1]-fit_corners.points[0]
        dy = fit_corners.points[3]-fit_corners.points[0]
        print("dx", dx)
        print("dy", dy)

        normal = -np.cross(line1, line2)
        normal_point = fit_corners.points[0] + normal
        print("normal", normal)
        line_pts = [fit_corners.points[0], fit_corners.points[1],
                    fit_corners.points[2], fit_corners.points[3], normal_point]
        lines = [[0, 1], [0, 3], [0, 4]]

        rotation_matrix = np.array(
            [dy/np.linalg.norm(dy), dx/np.linalg.norm(dx), normal/np.linalg.norm(normal)]).T

        #normal_norm = normal / np.linalg.norm(normal)
        #euler = np.arccos(np.dot(normal_norm, np.eye(3)))
        #euler = np.array([euler[0], euler[2], euler[1]])
        #euler = np.array([0, 0, 0])

        #rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz(euler)
        print(f"Rotation matrix found:\n\t{rotation_matrix}")
        #print(f"In euler [deg]:\n\t{euler * (180 / np.pi)}")

        lineset = o3d.geometry.LineSet()
        lineset.points = o3d.utility.Vector3dVector(line_pts)
        lineset.lines = o3d.utility.Vector2iVector(lines)

        blue_box_cluster_slim.paint_uniform_color([1, 0, 0])
        fit_box.paint_uniform_color([0, 1, 0])
        fit_corners.paint_uniform_color([0, 0, 1])
        print("Showing CPD results. Red is before and green is after.")
        #o3d.visualization.draw_geometries([blue_box_cluster_slim, blue_box_cluster_slim.get_oriented_bounding_box()])
        # o3d.visualization.draw_geometries([fit_box])
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.1, origin=fit_corners.get_center()).rotate(rotation_matrix, center=fit_corners.get_center())
        o3d.visualization.draw_geometries(
            [blue_box_cluster_slim, fit_box, fit_corners, lineset, mesh_frame])

        print(
            f"Volume before: {blue_box_cluster_slim.get_oriented_bounding_box().volume()}, volume after: {fit_box.get_oriented_bounding_box().volume()}")
        print(
            f"Center before: {blue_box_cluster_slim.get_oriented_bounding_box().get_center()}, center after: {fit_box.get_oriented_bounding_box().get_center()}")
        return fit_box, fit_corners, lines, rotation_matrix

    def publish_transform(self, transform, box_id):
        print(f"Adding transform of box {box_id} to broadcast queue")
        transform_stamped = TransformStamped()

        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "L515"
        transform_stamped.child_frame_id = f"blue_box_{box_id}"
        transform_stamped.transform = transform

        StampContainer.stamps.append(transform_stamped)

    def get_transform(self, rotation_matrix, translation):
        transform = Transform()
        transform.translation.x, transform.translation.y, transform.translation.z = translation
        rotation_quaternion = transforms3d.quaternions.mat2quat(
            rotation_matrix)
        #rotation_quaternion = transforms3d.quaternions.mat2quat(np.eye(3))
        transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z = rotation_quaternion
        return transform


class LidarPublisher(Node):
    def __init__(self):
        super().__init__("lidar_publisher")

        history_depth = 20
        self.tf_publisher = self.create_publisher(
            TFMessage, "/tf", history_depth)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Lidar publisher started")

    def timer_callback(self):
        if not StampContainer.stamps:
            return

        messages = [t for t in StampContainer.stamps]

        for message in messages:
            message.header.stamp = self.get_clock().now().to_msg()

        tf_message = TFMessage()
        tf_message.transforms = messages

        self.tf_broadcaster.sendTransform(messages)


def main(args=None):
    rclpy.init(args=args)
    try:
        use_example = False
        if args:
            if args[0] == "example":
                use_example = True

        c1 = PCDListener(use_example=use_example)
        c2 = LidarPublisher()

        executor = MultiThreadedExecutor()
        executor.add_node(c1)
        executor.add_node(c2)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            c1.destroy_node()
            c2.destroy_node()

    finally:
        rclpy.shutdown()

    #pcd_listener = PCDListener()

    # rclpy.spin(pcd_listener)

    # pcd_listener.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
