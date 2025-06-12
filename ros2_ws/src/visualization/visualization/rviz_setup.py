# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from itertools import product
import numpy as np
import math
import rclpy
from pathlib import Path
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from drone_interfaces.msg import ToFDistances, TelemetryData
from drone_interfaces.srv import HeightCommands
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField, CameraInfo
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge, CvBridgeError
import threading
import cv2
import yaml


class RvizSetup(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.laserscan_publisher = self.create_publisher(LaserScan, 'laserscan', 10)
        self.tof8x8_publisher = self.create_publisher(PointCloud2, 'tof8x8', 10)
        self.camerainfo_publisher = self.create_publisher(CameraInfo, 'camera_info', 1)

        self.telemetry_subscription = self.create_subscription(
            ToFDistances,
            'ToF_distances',
            self.ToF_callback,
            10)

        # self.video_subscription = self.create_subscription(
        #     Image,
        #     'video_frames',
        #     self.video_callback,
        #     1)

        self.incoming_commands = self.create_subscription(
            TelemetryData,
            'telemetry',
            self.telemetry_callback,
            1)
        # self.srv = self.create_service(HeightCommands, 'set_height', self.height_command_callback)

        self.ci = CameraInfo()
        filename = (
            Path(get_package_share_directory("visualization"))
            / "config"
            / "camera_info.yaml"
        )
        try:
            with filename.open() as f:
                calib = yaml.safe_load(f)
                if calib is not None:
                    # fill in CameraInfo fields
                    self.ci.width = calib["image_width"]
                    self.ci.height = calib["image_height"]
                    self.ci.distortion_model = calib["distortion_model"]
                    self.ci.d = calib["distortion_coefficients"]["data"]
                    self.ci.k = calib["camera_matrix"]["data"]
                    self.ci.r = calib["rectification_matrix"]["data"]
                    self.ci.p = calib["projection_matrix"]["data"]
        except OSError:  # OK if file did not exist
            pass

        self.ci.header.frame_id = "tello_main"
        self.ci.header.stamp = self.get_clock().now().to_msg()

        self.sim_fps = 10  # freq of computing translation
        self.timer = self.create_timer(1/self.sim_fps, self.comp_translation)  # TODO: Do it properly!
        self.camtimer = self.create_timer(5, self.caminfo_pub)  # TODO: Do it properly!

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.tof4: list[float] = [0.0] * 4
        self.tof8x8: list[float] = []
        self.velocity: np.ndarray = np.array([0, 0, 0])
        self.position: np.ndarray = np.array([0, 0, 0])
        self.rotation: R = R.from_euler("xyz", [0, 0, 0], degrees=True)
        self.dividor = 1e2
        self.frame = None

    def ToF_callback(self, msg):
        with self.lock:
            self.tof4[0] = msg.front
            self.tof4[1] = msg.left
            self.tof4[2] = msg.back
            self.tof4[3] = msg.right
            # ---------------------------
            self.tof8x8 = msg.matrix
            # ---------------------------
            # self.rotation[2] = -msg.degree

        self.handle_pose()
        self.handle_scan()
        self.handle_tof8x8()
        self.get_logger().info(f'Publishing Pose & LaserScan to Rviz [{self.rotation.as_euler("xyz")[2]}, {self.tof4[0]}]')

    def video_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame_msg = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV Image: {e}")

    def caminfo_pub(self):

            self.camerainfo_publisher.publish(self.ci)

    def telemetry_callback(self, msg):
        with self.lock:
            self.velocity[0] = msg.vgx
            self.velocity[1] = msg.vgy
            self.velocity[2] = msg.vgz

            self.position[2] = msg.h

            self.rotation = R.from_euler("xyz", [msg.pitch, msg.roll, msg.yaw], degrees=True)

    def handle_pose(self):
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = "world"
        tfs._child_frame_id = "tello_main"
        tfs.transform.translation.x = self.position[0] / self.dividor
        tfs.transform.translation.y = self.position[1] / self.dividor
        tfs.transform.translation.z = self.position[2] / self.dividor

        r = self.rotation.as_quat()

        tfs.transform.rotation.x = r[0]
        tfs.transform.rotation.y = r[1]
        tfs.transform.rotation.z = r[2]
        tfs.transform.rotation.w = r[3]

        self.tf_broadcaster.sendTransform(tfs)

    def handle_scan(self):
        laser_scan = LaserScan()
        laser_scan.header.frame_id = "tello_main"
        laser_scan.header.stamp = self.get_clock().now().to_msg()
        laser_scan.angle_min = 0.0
        laser_scan.angle_max = 4.71
        laser_scan.angle_increment = 1.57
        laser_scan.time_increment = 0.001
        laser_scan.scan_time = 0.01
        laser_scan.range_min = 0.1
        laser_scan.range_max = 12.0
        laser_scan.ranges = [t / self.dividor for t in self.tof4]
        laser_scan.intensities = [1.0, 0.5, 0.1, 0.5]

        self.laserscan_publisher.publish(laser_scan)

    def handle_tof8x8(self):
        point_cloud = PointCloud2()
        point_cloud.header.frame_id = "tello_main"
        point_cloud.header.stamp = self.get_clock().now().to_msg()
        point_cloud.height = 8
        point_cloud.width = 8
        point_cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
        point_cloud.point_step = 4*4
        point_cloud.row_step = 4*4 * 8*8
        point_cloud.is_dense = True

        coords = []

        for (i, j), dist in zip(product([-22.5, -16.07, -9.64, -3.21, 3.21, 9.64, 16.07, 22.5], repeat=2), self.tof8x8):
            dist = dist / self.dividor
            i, j = math.radians(i), math.radians(j)
            coords.append(dist * math.cos(j) * math.cos(-i))
            coords.append(dist * math.sin(-j))
            coords.append(dist * math.sin(-i))
            coords.append(dist)

        point_cloud.data = np.array(coords, dtype=np.float32).tobytes()

        self.tof8x8_publisher.publish(point_cloud)

    def comp_translation(self):

        # theta = self.rotation.as_euler("xyz")[2]

        # dx, dy = math.cos(-theta), math.sin(-theta)

        self.position = self.position + self.rotation.apply(self.velocity*4/self.sim_fps)
        # self.position[0] += (+ self.velocity[1] * dx - self.velocity[0] * dy) * 4 / self.sim_fps
        # self.position[1] += (- self.velocity[1] * dy - self.velocity[0] * dx) * 4 / self.sim_fps
        # self.position[2] = 300.0
        # self.get_logger().warning(f'POSITION: {self.position}')

        self.handle_pose()


def main(args=None):
    rclpy.init(args=args)

    # initialize model
    rvizsetup = RvizSetup()
    rclpy.spin(rvizsetup)
    rvizsetup.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

