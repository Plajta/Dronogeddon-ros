from ai.model_def import DetectionWrapper
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ModelNode(Node):
    def __init__(self):
        super().__init__("model_node")
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.callback,
            1
        )
        self.bridge = CvBridge()
        self.detection = DetectionWrapper()

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            self.detection.detect_img(frame)
            self.detection.parse_objects(frame, 1, 0.5)
            cv2.imshow('Neural result', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                pass

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV Image: {e}")


def main(args=None):
    rclpy.init(args=args)

    # initialize model
    model = ModelNode()

    rclpy.spin(model)
    model.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
