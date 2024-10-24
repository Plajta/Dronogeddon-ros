from ai.model_def import DetectionWrapper
from drone_interfaces.msg import Object, ObjectList

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ModelNode(Node):
    def __init__(self):
        super().__init__("model_node")
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.callback,
            1
        )
        self.publishing = self.create_publisher(ObjectList, "object_detection", 10)

        self.bridge = CvBridge()
        self.detection = DetectionWrapper()

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            self.detection.detect_img(frame)
            output = self.detection.filter_objects(1, 0.5)

            message = ObjectList()
            object_list = []
            
            for out_object in output:
                object = Object()
                object.cls = out_object["class"]
                object.x0 = out_object["bbox"][0]
                object.x1 = out_object["bbox"][1]
                object.y0 = out_object["bbox"][2]
                object.y1 = out_object["bbox"][3]

                object_list.append(object)

            message = object_list
            self.publishing.publish(message)

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
