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

import rclpy
from rclpy.node import Node
import random
from djitellopy import Tello

from drone_interfaces.msg import Telemetry   
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.telemetry_publisher = self.create_publisher(Telemetry, 'topic', 10)
        telemetry_timer_period = 1/10  # seconds
        self.timer = self.create_timer(telemetry_timer_period, self.telemetry_callback)
        
        self.video_publisher = self.create_publisher(Image, 'video_frames', 10)
        video_timer_period = 1/10  # seconds
        self.timer_video = self.create_timer(video_timer_period, self.publish_video_frame)
        self.bridge = CvBridge()

        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()

    def telemetry_callback(self):
        msg = Telemetry()
        data = self.mesurments()

        msg.front = data[0]
        msg.left = data[1]
        msg.right = data[2]
        msg.back = data[3]
        msg.degree = data[5]
        self.telemetry_publisher.publish(msg)
        self.get_logger().info('Publishing telemetry')
        
    def publish_video_frame(self):
        frame = self.frame_read.frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.video_publisher.publish(ros_image)
        self.get_logger().info('Publishing video frame')

    def mesurments(self):
        try:
            responses = self.tello.send_read_command('EXT tof?').split()
            if len(responses) < 6:
                return self.mesurments()
            else:
                return [int(responses[1]),int(responses[4]),int(responses[3]),int(responses[2]),int(responses[5]),self.tello.get_yaw()+180]
        except Exception as e:
            return self.mesurments()
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
