

# test_python_path.py
import sys
import rclpy
from rclpy.node import Node

class PythonPathNode(Node):
    def __init__(self):
        super().__init__('python_path_node')
        self.get_logger().info(f"Python executable: {sys.executable}")

def main(args=None):
    rclpy.init(args=args)
    node = PythonPathNode()
    rclpy.spin(node)
    rclpy.shutdown()