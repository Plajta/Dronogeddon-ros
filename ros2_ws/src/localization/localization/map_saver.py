#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import json
import os
from datetime import datetime
import cv2
import pickle
from threading import Lock

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from drone_interfaces.msg import TelemetryData

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        
        # Map data storage
        self.current_map = None
        self.map_metadata = {}
        self.drone_trajectory = []
        self.exploration_start_time = None
        self.data_lock = Lock()
        
        # File paths
        self.maps_directory = "saved_maps"
        self.ensure_maps_directory()
        
        # Current session info
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.map_updates_count = 0
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 1)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'drone_pose', self.pose_callback, 10)
        self.telemetry_sub = self.create_subscription(
            TelemetryData, 'telemetry', self.telemetry_callback, 10)
        self.mission_status_sub = self.create_subscription(
            String, 'mission_status', self.mission_status_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'map_saver_status', 10)
        
        # Timer for periodic saves
        self.save_timer = self.create_timer(30.0, self.periodic_save)  # Save every 30 seconds
        
        self.get_logger().info(f'Map Saver initialized - Session ID: {self.session_id}')
        
    def ensure_maps_directory(self):
        """Create maps directory if it doesn't exist"""
        if not os.path.exists(self.maps_directory):
            os.makedirs(self.maps_directory)
            self.get_logger().info(f'Created maps directory: {self.maps_directory}')
    
    def map_callback(self, msg):
        """Handle new map data from SLAM"""
        with self.data_lock:
            self.current_map = msg
            self.map_updates_count += 1
            
            # Update metadata
            self.map_metadata.update({
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
                'origin_z': msg.info.origin.position.z,
                'last_update': datetime.now().isoformat(),
                'updates_count': self.map_updates_count
            })
            
            if self.exploration_start_time is None:
                self.exploration_start_time = datetime.now()
                self.get_logger().info('Started tracking exploration map')
    
    def pose_callback(self, msg):
        """Handle drone pose updates"""
        with self.data_lock:
            pose_data = {
                'timestamp': datetime.now().isoformat(),
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'orientation_w': msg.pose.orientation.w,
                'orientation_x': msg.pose.orientation.x,
                'orientation_y': msg.pose.orientation.y,
                'orientation_z': msg.pose.orientation.z
            }
            self.drone_trajectory.append(pose_data)
            
            # Keep only last 1000 poses to avoid memory issues
            if len(self.drone_trajectory) > 1000:
                self.drone_trajectory = self.drone_trajectory[-1000:]
    
    def telemetry_callback(self, msg):
        """Handle telemetry data for trajectory tracking"""
        with self.data_lock:
            # Simple position tracking if no pose data available
            if len(self.drone_trajectory) == 0 or datetime.now().timestamp() - \
               datetime.fromisoformat(self.drone_trajectory[-1]['timestamp'].replace('Z', '+00:00')).timestamp() > 1.0:
                
                pose_data = {
                    'timestamp': datetime.now().isoformat(),
                    'x': 0.0,  # Would need position estimation from telemetry
                    'y': 0.0,
                    'z': msg.h / 100.0,  # Convert cm to meters
                    'yaw': msg.yaw,
                    'source': 'telemetry'
                }
                self.drone_trajectory.append(pose_data)
    
    def mission_status_callback(self, msg):
        """Handle mission status changes"""
        if 'EXPLORATION' in msg.data.upper():
            if self.exploration_start_time is None:
                self.exploration_start_time = datetime.now()
                self.get_logger().info('Exploration mode detected - starting map tracking')
        elif 'IDLE' in msg.data.upper() or 'LANDED' in msg.data.upper():
            if self.exploration_start_time is not None:
                self.save_final_map()
    
    def periodic_save(self):
        """Periodically save map during exploration"""
        if self.current_map is not None and self.exploration_start_time is not None:
            self.save_map_incremental()
    
    def save_map_incremental(self):
        """Save current map state incrementally"""
        try:
            with self.data_lock:
                timestamp = datetime.now().strftime("%H%M%S")
                filename_base = f"exploration_map_{self.session_id}_{timestamp}"
                
                # Save map data
                self.save_occupancy_grid(filename_base)
                self.save_map_metadata(filename_base)
                self.save_trajectory_data(filename_base)
                
                # Publish status
                status_msg = String()
                status_msg.data = f"Map saved incrementally: {filename_base} (updates: {self.map_updates_count})"
                self.status_pub.publish(status_msg)
                
                self.get_logger().info(f'Incremental map save: {filename_base}')
                
        except Exception as e:
            self.get_logger().error(f'Error saving incremental map: {e}')
    
    def save_final_map(self):
        """Save final map when exploration ends"""
        try:
            with self.data_lock:
                if self.current_map is None:
                    self.get_logger().warning('No map data to save')
                    return
                
                end_time = datetime.now()
                duration = (end_time - self.exploration_start_time).total_seconds()
                
                filename_base = f"final_exploration_map_{self.session_id}"
                
                # Save all data formats
                self.save_occupancy_grid(filename_base)
                self.save_map_metadata(filename_base)
                self.save_trajectory_data(filename_base)
                self.save_map_image(filename_base)
                self.save_analysis_data(filename_base, duration)
                
                # Publish status
                status_msg = String()
                status_msg.data = f"Final exploration map saved: {filename_base}"
                self.status_pub.publish(status_msg)
                
                self.get_logger().info(f'Final map saved: {filename_base} (duration: {duration:.1f}s, updates: {self.map_updates_count})')
                
                # Reset for next exploration
                self.exploration_start_time = None
                self.map_updates_count = 0
                
        except Exception as e:
            self.get_logger().error(f'Error saving final map: {e}')
    
    def save_occupancy_grid(self, filename_base):
        """Save raw occupancy grid data"""
        if self.current_map is None:
            return
        
        # Save as pickle for exact data preservation
        map_data = {
            'header': {
                'stamp_sec': self.current_map.header.stamp.sec,
                'stamp_nanosec': self.current_map.header.stamp.nanosec,
                'frame_id': self.current_map.header.frame_id
            },
            'info': {
                'map_load_time_sec': self.current_map.info.map_load_time.sec,
                'map_load_time_nanosec': self.current_map.info.map_load_time.nanosec,
                'resolution': self.current_map.info.resolution,
                'width': self.current_map.info.width,
                'height': self.current_map.info.height,
                'origin': {
                    'position': {
                        'x': self.current_map.info.origin.position.x,
                        'y': self.current_map.info.origin.position.y,
                        'z': self.current_map.info.origin.position.z
                    },
                    'orientation': {
                        'x': self.current_map.info.origin.orientation.x,
                        'y': self.current_map.info.origin.orientation.y,
                        'z': self.current_map.info.origin.orientation.z,
                        'w': self.current_map.info.origin.orientation.w
                    }
                }
            },
            'data': list(self.current_map.data)
        }
        
        pickle_path = os.path.join(self.maps_directory, f"{filename_base}.pkl")
        with open(pickle_path, 'wb') as f:
            pickle.dump(map_data, f)
    
    def save_map_metadata(self, filename_base):
        """Save map metadata as JSON"""
        metadata = self.map_metadata.copy()
        metadata.update({
            'session_id': self.session_id,
            'exploration_start': self.exploration_start_time.isoformat() if self.exploration_start_time else None,
            'trajectory_points': len(self.drone_trajectory),
            'saved_at': datetime.now().isoformat()
        })
        
        json_path = os.path.join(self.maps_directory, f"{filename_base}_metadata.json")
        with open(json_path, 'w') as f:
            json.dump(metadata, f, indent=2)
    
    def save_trajectory_data(self, filename_base):
        """Save drone trajectory data"""
        trajectory_path = os.path.join(self.maps_directory, f"{filename_base}_trajectory.json")
        with open(trajectory_path, 'w') as f:
            json.dump(self.drone_trajectory, f, indent=2)
    
    def save_map_image(self, filename_base):
        """Save map as image for visualization"""
        if self.current_map is None:
            return
        
        try:
            # Convert occupancy grid to image
            width = self.current_map.info.width
            height = self.current_map.info.height
            data = np.array(self.current_map.data).reshape((height, width))
            
            # Convert to image format (0=free, 100=occupied, -1=unknown)
            img = np.zeros((height, width), dtype=np.uint8)
            img[data == 0] = 255    # Free space = white
            img[data == 100] = 0    # Occupied = black
            img[data == -1] = 128   # Unknown = gray
            
            # Flip image (occupancy grid origin is bottom-left, image origin is top-left)
            img = np.flipud(img)
            
            # Save image
            img_path = os.path.join(self.maps_directory, f"{filename_base}.png")
            cv2.imwrite(img_path, img)
            
            # Save image with trajectory overlay
            if len(self.drone_trajectory) > 1:
                img_with_trajectory = img.copy()
                if len(img_with_trajectory.shape) == 2:
                    img_with_trajectory = cv2.cvtColor(img_with_trajectory, cv2.COLOR_GRAY2BGR)
                
                # Draw trajectory
                resolution = self.current_map.info.resolution
                origin_x = self.current_map.info.origin.position.x
                origin_y = self.current_map.info.origin.position.y
                
                for i in range(1, len(self.drone_trajectory)):
                    if 'x' in self.drone_trajectory[i] and 'x' in self.drone_trajectory[i-1]:
                        # Convert world coordinates to image coordinates
                        x1 = int((self.drone_trajectory[i-1]['x'] - origin_x) / resolution)
                        y1 = height - int((self.drone_trajectory[i-1]['y'] - origin_y) / resolution)
                        x2 = int((self.drone_trajectory[i]['x'] - origin_x) / resolution)
                        y2 = height - int((self.drone_trajectory[i]['y'] - origin_y) / resolution)
                        
                        if 0 <= x1 < width and 0 <= y1 < height and 0 <= x2 < width and 0 <= y2 < height:
                            cv2.line(img_with_trajectory, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red trajectory
                
                trajectory_img_path = os.path.join(self.maps_directory, f"{filename_base}_with_trajectory.png")
                cv2.imwrite(trajectory_img_path, img_with_trajectory)
            
        except Exception as e:
            self.get_logger().error(f'Error saving map image: {e}')
    
    def save_analysis_data(self, filename_base, exploration_duration):
        """Save exploration analysis data"""
        analysis = {
            'session_id': self.session_id,
            'exploration_duration_seconds': exploration_duration,
            'map_updates_count': self.map_updates_count,
            'trajectory_points': len(self.drone_trajectory),
            'map_area_explored': self.calculate_explored_area(),
            'exploration_efficiency': self.calculate_exploration_efficiency(),
            'room_detection_hints': self.generate_room_hints()
        }
        
        analysis_path = os.path.join(self.maps_directory, f"{filename_base}_analysis.json")
        with open(analysis_path, 'w') as f:
            json.dump(analysis, f, indent=2)
    
    def calculate_explored_area(self):
        """Calculate total explored area in square meters"""
        if self.current_map is None:
            return 0.0
        
        data = np.array(self.current_map.data)
        free_cells = np.sum(data == 0)  # Free space cells
        resolution = self.current_map.info.resolution
        
        return float(free_cells * resolution * resolution)
    
    def calculate_exploration_efficiency(self):
        """Calculate exploration efficiency metrics"""
        if not self.drone_trajectory or self.exploration_start_time is None:
            return {}
        
        total_distance = 0.0
        for i in range(1, len(self.drone_trajectory)):
            if 'x' in self.drone_trajectory[i] and 'x' in self.drone_trajectory[i-1]:
                dx = self.drone_trajectory[i]['x'] - self.drone_trajectory[i-1]['x']
                dy = self.drone_trajectory[i]['y'] - self.drone_trajectory[i-1]['y']
                total_distance += np.sqrt(dx*dx + dy*dy)
        
        explored_area = self.calculate_explored_area()
        duration = (datetime.now() - self.exploration_start_time).total_seconds()
        
        return {
            'total_distance_meters': total_distance,
            'area_per_distance': explored_area / max(total_distance, 0.1),
            'area_per_time': explored_area / max(duration, 0.1),
            'average_speed': total_distance / max(duration, 0.1)
        }
    
    def generate_room_hints(self):
        """Generate hints for manual room configuration"""
        if self.current_map is None:
            return []
        
        hints = []
        
        # Basic room detection hints based on map analysis
        data = np.array(self.current_map.data).reshape((self.current_map.info.height, self.current_map.info.width))
        
        # Find connected free space regions (potential rooms)
        free_space = (data == 0).astype(np.uint8)
        
        if np.any(free_space):
            # Find contours of free spaces
            contours, _ = cv2.findContours(free_space, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            resolution = self.current_map.info.resolution
            origin_x = self.current_map.info.origin.position.x
            origin_y = self.current_map.info.origin.position.y
            
            for i, contour in enumerate(contours):
                if cv2.contourArea(contour) > 100:  # Filter small areas
                    # Calculate bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Convert to world coordinates
                    world_x = origin_x + (x + w/2) * resolution
                    world_y = origin_y + (self.current_map.info.height - y - h/2) * resolution
                    world_w = w * resolution
                    world_h = h * resolution
                    
                    area = cv2.contourArea(contour) * resolution * resolution
                    
                    hints.append({
                        'region_id': i + 1,
                        'center_x': round(world_x, 2),
                        'center_y': round(world_y, 2),
                        'width': round(world_w, 2),
                        'height': round(world_h, 2),
                        'area_sqm': round(area, 2),
                        'suggested_room_config': {
                            'center': [world_x, world_y],
                            'size': [world_w, world_h]
                        }
                    })
        
        return hints

def main(args=None):
    rclpy.init(args=args)
    
    map_saver = MapSaver()
    
    try:
        rclpy.spin(map_saver)
    except KeyboardInterrupt:
        map_saver.get_logger().info('Map Saver shutting down')
        # Save final map on shutdown
        if map_saver.exploration_start_time is not None:
            map_saver.save_final_map()
    
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
