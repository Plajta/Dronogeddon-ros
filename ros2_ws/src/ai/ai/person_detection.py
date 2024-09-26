from model_def import DetectionWrapper, torch, cv2, np
import rlcpy
from rlcpy.node import Node


class ModelNode(Node):
    def __init__(self):
        super().__init__("model_node")
        self.subscription = self.create_subcription(
            "TODO"
        )
        

    def callback(self, image):
        pass


def main(args=None):
    rlcpy.init(args=args)

    # initialize model
    model = ModelNode()
    rclpy.spin(model)
    rclpy.shutdown()

if __name__ == '__main__':
    main()