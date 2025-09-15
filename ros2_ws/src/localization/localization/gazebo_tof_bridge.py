#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from drone_interfaces.msg import ToFDistances, MatrixToFData
import numpy as np

class GazeboToFBridge(Node):
    """
    Bridge node to convert Gazebo laser scan data to ToF sensor format
    compatible with existing exploration system
    """
    
    def __init__(self):
        super().__init__('gazebo_tof_bridge')
        
        # Subscribers for Gazebo laser data
        self.tof_forward_sub = self.create_subscription(
            LaserScan, '/tof_forward', self.forward_callback, 10)
        self.tof_backward_sub = self.create_subscription(
            LaserScan, '/tof_backward', self.backward_callback, 10)
        self.tof_left_sub = self.create_subscription(
            LaserScan, '/tof_left', self.left_callback, 10)
        self.tof_right_sub = self.create_subscription(
            LaserScan, '/tof_right', self.right_callback, 10)
        self.matrix_tof_sub = self.create_subscription(
            LaserScan, '/matrix_tof', self.matrix_callback, 10)
        
        # Publishers for drone interface format
        self.tof_publisher = self.create_publisher(ToFDistances, 'ToF_distances', 10)
        self.matrix_publisher = self.create_publisher(MatrixToFData, 'matrix_tof_data', 10)
        
        # Store latest distances
        self.distances = {
            'front': 4.0,
            'back': 4.0,
            'left': 4.0,
            'right': 4.0
        }
        
        # Timer to publish combined ToF data
        self.timer = self.create_timer(0.1, self.publish_tof_data)
        
        self.get_logger().info('Gazebo ToF Bridge initialized')
    
    def forward_callback(self, msg):
        if len(msg.ranges) > 0 and not np.isinf(msg.ranges[0]):
            self.distances['front'] = min(msg.ranges[0], 4.0)
    
    def backward_callback(self, msg):
        if len(msg.ranges) > 0 and not np.isinf(msg.ranges[0]):
            self.distances['back'] = min(msg.ranges[0], 4.0)
    
    def left_callback(self, msg):
        if len(msg.ranges) > 0 and not np.isinf(msg.ranges[0]):
            self.distances['left'] = min(msg.ranges[0], 4.0)
    
    def right_callback(self, msg):
        if len(msg.ranges) > 0 and not np.isinf(msg.ranges[0]):
            self.distances['right'] = min(msg.ranges[0], 4.0)
    
    def matrix_callback(self, msg):
        """Convert Gazebo 8x8 laser scan to matrix ToF format"""
        if len(msg.ranges) >= 64:  # 8x8 = 64 points
            matrix_msg = MatrixToFData()
            
            # Convert ranges to 8x8 matrix (in cm, like real sensor after packCharsUsing95)
            matrix_data = []
            for i in range(64):
                distance_m = msg.ranges[i] if not np.isinf(msg.ranges[i]) else 4.0
                distance_cm = int(min(distance_m * 100, 400))  # Convert to cm, max 4m
                matrix_data.append(distance_cm)
            
            matrix_msg.distances = matrix_data
            matrix_msg.width = 8
            matrix_msg.height = 8
            matrix_msg.header.stamp = self.get_clock().now().to_msg()
            matrix_msg.header.frame_id = "matrix_tof_link"
            
            self.matrix_publisher.publish(matrix_msg)
    
    def publish_tof_data(self):
        """Publish combined ToF distances in drone interface format"""
        tof_msg = ToFDistances()
        
        # Convert to mm (like real Tello sensors)
        tof_msg.front = int(self.distances['front'] * 1000)
        tof_msg.back = int(self.distances['back'] * 1000)
        tof_msg.left = int(self.distances['left'] * 1000)
        tof_msg.right = int(self.distances['right'] * 1000)
        
        tof_msg.header.stamp = self.get_clock().now().to_msg()
        tof_msg.header.frame_id = "base_link"
        
        self.tof_publisher.publish(tof_msg)

def main(args=None):
    rclpy.init(args=args)
    
    bridge = GazeboToFBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
