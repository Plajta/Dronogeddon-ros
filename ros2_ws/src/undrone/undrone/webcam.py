#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
import cv_bridge
from sensor_msgs.msg import Image

class CameraNode(Node):
    def __init__(self):
        super().__init__("webcam_node")
        self.cv_bridge = cv_bridge.CvBridge()
        self.video_publisher = self.create_publisher(Image, 'video_frames', 10)
        video_timer_period = 1/10  # period of publishing
        cam = cv2.VideoCapture(0)
        self.timer = self.create_timer(video_timer_period, lambda: self.publish_video_frame(cam))

    def publish_video_frame(self, cam):
        ret, frame = cam.read()
        frame = cv2.resize(frame, (960, 720))
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # convert OpenCV image to ROS Image message
        ros_image = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.video_publisher.publish(ros_image)
        self.get_logger().info('Publishing un-droned video frame (webcam)')


def main(args=None):
    rclpy.init(args=args)

    # initialize model
    cam = CameraNode()
    rclpy.spin(cam)
    cam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
