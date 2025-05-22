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
import numpy as np
import rclpy
from rclpy.node import Node
from drone_interfaces.msg import ToFDistances
from drone_interfaces.msg import Object, ObjectList
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
import cv2
import time


class DroneVis(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.lock = threading.Lock()
        self.data = None
        self.detected_objects = []

        self.telemetry_subscription = self.create_subscription(
            ToFDistances,
            'ToF_distances',
            self.listener_callback,
            10)

        self.video_subscription = self.create_subscription(
            Image,
            'video_frames',
            self.video_callback,
            1)

        self.ai_subscription = self.create_subscription(
            ObjectList,
            'object_detection',
            self.ai_callback,
            10
        )

        self.bridge = CvBridge()

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.start_time = time.time()
        self.log_time = time.strftime("%Y_%m_%d_%H_%M_%S", time.gmtime())
        self.dvis_size = (300, 400)
        self.dvis_ctr = (self.dvis_size[0] // 2, self.dvis_size[1] // 2)
        self.maxdist = 4500

        img = cv2.imread("/home/jan/Dokumenty/Dronogeddon-ros/ros2_ws/src/my_drone/my_drone/tello.png", cv2.IMREAD_UNCHANGED)
        self.get_logger().info(f"loaded Tello PNG: {img.shape}")
        self.bin_im = np.array(img[:, :, 3][::-1,::], dtype=np.bool_)

    def vis_dist(self, image, distance, clip, degree, color):
        scale = min(self.dvis_ctr)
        end_pnt = (self.dvis_ctr[1] + int(distance * scale * np.sin(-np.deg2rad(degree)) / clip),
                   self.dvis_ctr[0] + int(distance * scale * np.cos(-np.deg2rad(degree)) / clip))
        cv2.line(image, self.dvis_ctr[::-1], end_pnt, color, max(1, scale//50))

    def vis_drone(self, image, degree, color):
        scale = min(self.dvis_ctr)
        rows, cols = self.bin_im.astype(np.uint8).shape
        M = cv2.getRotationMatrix2D(((cols - 1) / 2.0, (rows - 1) / 2.0), -degree, 1)
        tello = cv2.warpAffine(self.bin_im.astype(np.uint8), M, (cols, rows))
        tello = cv2.resize(tello, (scale//4*2, scale//4*2))

        rows, cols = tello.shape
        dst = np.zeros((cols, rows, 3), dtype=np.uint8)
        for i, clr in enumerate(color):
            dst[:,:,i] = tello * clr

        yt, xl = self.dvis_ctr[0]-cols//2, self.dvis_ctr[1]-rows//2
        yb, xr = yt + cols, xl + rows
        image[yt:yb, xl:xr] &= np.bitwise_not(cv2.cvtColor(tello*255, cv2.COLOR_GRAY2RGB))
        image[yt:yb, xl:xr] |= dst

    def listener_callback(self, msg):
        self.get_logger().info(f"Drone distance ↑{msg.front} ←{msg.left} →{msg.right} ↓{msg.back} ø{msg.degree}")
        with self.lock:
            self.data = msg

    def ai_callback(self, msg):
        parsed = []
        for object in msg.entries:
            parsed.append({
                "cls": object.cls,
                "coords": [object.x0, object.y0, object.x1, object.y1]
            })

        self.detected_objects = parsed.copy()

    def video_callback(self, msg):

        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if self.data is None:
                return

            if len(self.detected_objects) != 0:
                self.get_logger().info("Got object data, rendering")
                for det_object in self.detected_objects:
                    cls = det_object["cls"]
                    if cls == 1:
                        cls_name = "Human"
                    else:
                        cls_name = "Undefined"

                    coords = det_object["coords"]

                    cv2.putText(frame, cls_name, (coords[0], coords[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.rectangle(frame, (coords[2], coords[3]), (coords[1], coords[2]), (0, 0, 255), 2)

            cv2.putText(frame, 
                f"{round(time.time()-self.start_time, 2)}s", 
                (10, 20), 
                self.font, 1/2, 
                (0, 255, 255),
                2,
                cv2.LINE_4) 

            cv2.putText(frame, 
                        f"{self.log_time}", 
                        (770, 20), 
                        self.font, 1/2, 
                        (0, 255, 255), 
                        2, 
                        cv2.LINE_4)

            dvis_img = np.zeros([*self.dvis_size, 3], dtype=np.uint8)

            with self.lock:
                cv2.putText(frame, 
                        f"left: {self.data.left} front: {self.data.front} right: {self.data.right}", 
                        (10, 700), 
                        self.font, 1/2, 
                        (0, 255, 255), 
                        2, 
                        cv2.LINE_4)

                # clip = max(self.maxdist, self.data.front, self.data.right, self.data.back, self.data.left)
                clip = self.maxdist

                self.vis_dist(dvis_img, self.data.front, clip, self.data.degree, (0, 0, 200))
                self.vis_dist(dvis_img, self.data.right, clip, self.data.degree+90, (200, 200, 200))
                self.vis_dist(dvis_img, self.data.back, clip, self.data.degree+180, (200, 50, 50))
                self.vis_dist(dvis_img, self.data.left, clip, self.data.degree+270, (200, 200, 200))
                self.vis_drone(dvis_img, self.data.degree, (50, 50, 120))

            mask = cv2.cvtColor(dvis_img, cv2.COLOR_BGR2GRAY).astype(dtype=np.bool_).astype(dtype=np.uint8)*255
            frame[-self.dvis_size[0]:, -self.dvis_size[1]:] &= np.bitwise_not(cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB))
            frame[-self.dvis_size[0]:, -self.dvis_size[1]:] |= dvis_img

            cv2.imshow('Tello Video Stream', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                pass

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV Image: {e}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = DroneVis()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
