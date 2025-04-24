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
from rclpy.node import Node
from drone_interfaces.msg import Telemetry, RCcommands
from drone_interfaces.srv import HeightCommands
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge, CvBridgeError
import threading
import cv2


class RvizSetup(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.laserscan_publisher = self.create_publisher(LaserScan, 'laserscan', 10)
        self.tof8x8_publisher = self.create_publisher(PointCloud2, 'tof8x8', 10)

        self.telemetry_subscription = self.create_subscription(
            Telemetry,
            'telemtetry',
            self.listener_callback,
            10)

        self.video_subscription = self.create_subscription(
            Image,
            'video_frames',
            self.video_callback,
            1)

        self.incoming_commands = self.create_subscription(
            RCcommands,
            'rc_commands',
            self.rc_command_callback,
            1)
        # self.srv = self.create_service(HeightCommands, 'set_height', self.height_command_callback)

        self.sim_fps = 10  # freq of computing translation
        self.timer = self.create_timer(1/self.sim_fps, self.comp_translation)  # TODO: Do it properly!

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.telemetry: list[float] = [0.0]*4
        self.tof8x8: list[float] = []
        self.velocity: list[float] = [0.0]*4
        self.position: list[float] = [0.0]*4
        self.dividor = 1e2

    def listener_callback(self, msg):
        with self.lock:
            self.telemetry[0] = msg.front
            self.telemetry[1] = msg.left
            self.telemetry[2] = msg.back
            self.telemetry[3] = msg.right
            # ---------------------------
            self.tof8x8 = msg.matrix
            # ---------------------------
            self.position[3] = -msg.degree

        self.handle_pose()
        self.handle_scan()
        self.handle_tof8x8()
        self.get_logger().info(f'Publishing Pose & LaserScan to Rviz [{self.position[3]}, {self.telemetry[0]}]')

    def video_callback(self, msg):
        pass

    def rc_command_callback(self, msg):
        with self.lock:
            self.velocity[0] = msg.left_right_velocity
            self.velocity[1] = msg.forward_backward_velocity
            self.velocity[2] = msg.up_down_velocity
            self.velocity[3] = msg.yaw_velocity

    def handle_pose(self):
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = "world"
        tfs._child_frame_id = "tello_main"
        tfs.transform.translation.x = self.position[0] / self.dividor
        tfs.transform.translation.y = self.position[1] / self.dividor
        tfs.transform.translation.z = self.position[2] / self.dividor

        r = R.from_euler('xyz', [0, 0, self.position[3]], degrees=True).as_quat()

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
        laser_scan.ranges = [t / self.dividor for t in self.telemetry]
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

        theta = math.radians(self.position[3])

        dx, dy = math.cos(-theta), math.sin(-theta)

        self.position[0] += (+ self.velocity[1] * dx - self.velocity[0] * dy) * 4 / self.sim_fps
        self.position[1] += (- self.velocity[1] * dy - self.velocity[0] * dx) * 4 / self.sim_fps
        self.position[2] = 300.0
        self.get_logger().warning(f'POSITION: {self.position}')

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

