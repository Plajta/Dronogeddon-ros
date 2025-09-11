#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from threading import Lock
import math
import json
from datetime import datetime

from drone_interfaces.msg import TelemetryData, ToFDistances
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
# Import centralized tf_transformations fix
from .tf_transformations_fix import quaternion_from_euler, euler_from_quaternion

class SLAMMapper(Node):
    def __init__(self):
        super().__init__('slam_mapper')
        
        # Map parameters
        self.map_resolution = 0.05  # meters per pixel (5cm resolution)
        self.map_width = 800  # pixels (40m x 40m map)
        self.map_height = 800
        self.map_origin_x = -20.0  # meters
        self.map_origin_y = -20.0
        
        # Initialize occupancy grid map
        self.occupancy_map = np.full((self.map_height, self.map_width), -1, dtype=np.int8)  # -1 = unknown, 0 = free, 100 = occupied
        
        # Robot pose tracking
        self.robot_x = 0.0  # meters
        self.robot_y = 0.0
        self.robot_yaw = 0.0  # radians
        self.robot_height = 0.0
        
        # Data synchronization
        self.data_lock = Lock()
        self.last_telemetry = None
        self.last_distances = None
        
        # Subscribers
        self.telemetry_sub = self.create_subscription(
            TelemetryData, 'telemetry', self.telemetry_callback, 10)
        self.distances_sub = self.create_subscription(
            ToFDistances, 'ToF_distances', self.distances_callback, 10)
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        
        # Timer for map publishing
        self.map_timer = self.create_timer(1.0, self.publish_map)
        
        # Exploration tracking
        self.explored_cells = set()
        self.room_boundaries = []
        
        self.get_logger().info('SLAM Mapper initialized')

    def telemetry_callback(self, msg):
        with self.data_lock:
            self.last_telemetry = msg
            # Update robot pose from telemetry
            self.robot_yaw = math.radians(msg.yaw)
            self.robot_height = msg.h / 100.0  # convert cm to meters
            
            # Simple dead reckoning using velocity (basic odometry)
            if hasattr(self, 'last_update_time'):
                dt = 0.1  # assume 10Hz update rate
                # Convert velocities from cm/s to m/s
                vx = msg.vgx / 100.0
                vy = msg.vgy / 100.0
                
                # Update position (simple integration)
                self.robot_x += (vx * math.cos(self.robot_yaw) - vy * math.sin(self.robot_yaw)) * dt
                self.robot_y += (vx * math.sin(self.robot_yaw) + vy * math.cos(self.robot_yaw)) * dt
            
            self.last_update_time = self.get_clock().now()

    def distances_callback(self, msg):
        with self.data_lock:
            self.last_distances = msg
            if self.last_telemetry is not None:
                self.update_map_with_sensors(msg)

    def update_map_with_sensors(self, distances):
        """Update occupancy map using ToF sensor data"""
        
        # Convert robot position to map coordinates
        map_x = int((self.robot_x - self.map_origin_x) / self.map_resolution)
        map_y = int((self.robot_y - self.map_origin_y) / self.map_resolution)
        
        if not (0 <= map_x < self.map_width and 0 <= map_y < self.map_height):
            return
        
        # Mark current position as free
        self.occupancy_map[map_y, map_x] = 0
        self.explored_cells.add((map_x, map_y))
        
        # Process single-point ToF sensors
        sensor_angles = [0, -math.pi/2, math.pi/2, math.pi]  # front, left, right, back
        sensor_distances = [distances.front/100.0, distances.left/100.0, 
                          distances.right/100.0, distances.back/100.0]  # convert cm to m
        
        for i, (angle_offset, distance) in enumerate(zip(sensor_angles, sensor_distances)):
            if distance > 0 and distance < 12.0:  # valid range up to 12m
                self.update_map_ray(map_x, map_y, self.robot_yaw + angle_offset, distance)
        
        # Process 8x8 matrix sensor (forward-facing)
        self.update_map_with_matrix(map_x, map_y, distances.matrix)

    def update_map_ray(self, start_x, start_y, angle, distance):
        """Update map along a ray from sensor"""
        
        # Calculate end point
        end_x = start_x + int(distance * math.cos(angle) / self.map_resolution)
        end_y = start_y + int(distance * math.sin(angle) / self.map_resolution)
        
        # Bresenham's line algorithm to trace ray
        points = self.bresenham_line(start_x, start_y, end_x, end_y)
        
        # Mark cells along ray as free (except the last one)
        for i, (x, y) in enumerate(points[:-1]):
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.occupancy_map[y, x] = 0
                self.explored_cells.add((x, y))
        
        # Mark end point as occupied (obstacle detected)
        if 0 <= end_x < self.map_width and 0 <= end_y < self.map_height:
            self.occupancy_map[end_y, end_x] = 100

    def update_map_with_matrix(self, robot_map_x, robot_map_y, matrix):
        """Update map using 8x8 matrix sensor data"""
        
        if len(matrix) != 64:
            return
        
        # Matrix sensor parameters
        fov_angle = math.radians(45)  # 45-degree field of view
        max_range = 4.0  # 4 meters max range
        
        for row in range(8):
            for col in range(8):
                idx = row * 8 + col
                distance = matrix[idx] / 1000.0  # convert mm to meters
                
                if distance > 0 and distance < max_range:
                    # Calculate angle for this pixel
                    pixel_angle_h = (col - 3.5) * (fov_angle / 8)
                    pixel_angle_v = (row - 3.5) * (fov_angle / 8)
                    
                    # For simplicity, project to 2D (ignore vertical angle for now)
                    total_angle = self.robot_yaw + pixel_angle_h
                    
                    # Update map along this ray
                    self.update_map_ray(robot_map_x, robot_map_y, total_angle, distance)

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points

    def publish_map(self):
        """Publish the current occupancy grid map"""
        
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Map metadata
        msg.info = MapMetaData()
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Convert numpy array to list
        msg.data = self.occupancy_map.flatten().tolist()
        
        self.map_pub.publish(msg)
        
        # Log exploration progress
        explored_percentage = len(self.explored_cells) / (self.map_width * self.map_height) * 100
        self.get_logger().info(f'Map published. Explored: {explored_percentage:.1f}% '
                             f'Robot pos: ({self.robot_x:.2f}, {self.robot_y:.2f})')

    def save_map(self, filename=None):
        """Save current map to file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"map_{timestamp}.json"
        
        map_data = {
            'map': self.occupancy_map.tolist(),
            'resolution': self.map_resolution,
            'width': self.map_width,
            'height': self.map_height,
            'origin_x': self.map_origin_x,
            'origin_y': self.map_origin_y,
            'robot_pose': {
                'x': self.robot_x,
                'y': self.robot_y,
                'yaw': self.robot_yaw
            },
            'explored_cells': list(self.explored_cells)
        }
        
        with open(filename, 'w') as f:
            json.dump(map_data, f)
        
        self.get_logger().info(f'Map saved to {filename}')

    def get_exploration_frontiers(self):
        """Find frontier cells for exploration"""
        frontiers = []
        
        for x in range(1, self.map_width - 1):
            for y in range(1, self.map_height - 1):
                if self.occupancy_map[y, x] == -1:  # unknown cell
                    # Check if adjacent to free space
                    adjacent_free = False
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            if self.occupancy_map[y + dy, x + dx] == 0:
                                adjacent_free = True
                                break
                        if adjacent_free:
                            break
                    
                    if adjacent_free:
                        # Convert to world coordinates
                        world_x = x * self.map_resolution + self.map_origin_x
                        world_y = y * self.map_resolution + self.map_origin_y
                        frontiers.append((world_x, world_y))
        
        return frontiers

def main(args=None):
    rclpy.init(args=args)
    
    slam_mapper = SLAMMapper()
    
    try:
        rclpy.spin(slam_mapper)
    except KeyboardInterrupt:
        slam_mapper.save_map()
    finally:
        slam_mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
