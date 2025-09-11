#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA, String, Int32
from nav_msgs.msg import OccupancyGrid
from .room_config import get_room_config
import math
from threading import Lock
from drone_interfaces.msg import TelemetryData, ToFDistances
# Import centralized tf_transformations fix
from .tf_transformations_fix import quaternion_from_euler

class RVizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')
        
        # Load room configuration from external file
        self.room_config = get_room_config()
        self.rooms = {}
        self._load_rooms_from_config()
        
        # Drone state
        self.drone_position = (0.0, 0.0, 1.5)
        self.drone_yaw = 0.0
        self.drone_trajectory = []
        self.max_trajectory_points = 100
        
        # Mission state
        self.current_mission_state = "IDLE"
        self.investigation_target = None
        self.data_lock = Lock()
        
        # Subscribers
        self.telemetry_sub = self.create_subscription(
            TelemetryData, 'telemetry', self.telemetry_callback, 10)
        self.distances_sub = self.create_subscription(
            ToFDistances, 'ToF_distances', self.distances_callback, 10)
        self.motion_sub = self.create_subscription(
            String, 'motion_detected', self.motion_callback, 10)
        self.mission_status_sub = self.create_subscription(
            String, 'mission_status', self.mission_status_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 1)
        
        # Publishers
        self.room_marker_pub = self.create_publisher(MarkerArray, 'room_markers', 10)
        self.drone_marker_pub = self.create_publisher(Marker, 'drone_marker', 10)
        self.trajectory_marker_pub = self.create_publisher(Marker, 'drone_trajectory', 10)
        self.tof_marker_pub = self.create_publisher(MarkerArray, 'tof_sensors', 10)
        self.investigation_marker_pub = self.create_publisher(Marker, 'investigation_target', 10)
        
        # Timers
        self.visualization_timer = self.create_timer(0.5, self.publish_visualizations)
        
        self.get_logger().info('RViz Visualizer initialized')
    
    def _load_rooms_from_config(self):
        """Load room definitions from configuration file"""
        config_rooms = self.room_config.get_all_rooms()
        
        for room_name, room_info in config_rooms.items():
            self.rooms[room_name] = {
                'center': tuple(room_info['center']),
                'size': tuple(room_info['size']),
                'color': tuple(room_info['color']),
                'name': room_info['name'],
                'id': room_info['id'],
                'detected_motion': False
            }
        
        self.get_logger().info(f'Loaded {len(self.rooms)} rooms from configuration')

    def telemetry_callback(self, msg):
        with self.data_lock:
            # Update drone position (simple dead reckoning)
            self.drone_yaw = math.radians(msg.yaw)
            
            # For now, use simple position estimation
            # In real implementation, you'd integrate this with SLAM
            if hasattr(self, 'last_telemetry_time'):
                dt = 0.1  # assume 10Hz
                vx = msg.vgx / 100.0  # cm/s to m/s
                vy = msg.vgy / 100.0
                
                # Update position
                old_pos = self.drone_position
                new_x = old_pos[0] + (vx * math.cos(self.drone_yaw) - vy * math.sin(self.drone_yaw)) * dt
                new_y = old_pos[1] + (vx * math.sin(self.drone_yaw) + vy * math.cos(self.drone_yaw)) * dt
                new_z = msg.h / 100.0  # height in meters
                
                self.drone_position = (new_x, new_y, new_z)
                
                # Add to trajectory
                self.drone_trajectory.append(self.drone_position)
                if len(self.drone_trajectory) > self.max_trajectory_points:
                    self.drone_trajectory.pop(0)
            
            self.last_telemetry_time = self.get_clock().now()

    def distances_callback(self, msg):
        # Store sensor data for visualization
        with self.data_lock:
            self.current_distances = {
                'front': msg.front / 100.0,
                'left': msg.left / 100.0,
                'right': msg.right / 100.0,
                'back': msg.back / 100.0
            }

    def motion_callback(self, msg):
        """Handle motion detection events"""
        with self.data_lock:
            # Parse room from motion message
            if 'Room' in msg.data:
                try:
                    # Extract room number from message like "Room 1 (living_room)"
                    parts = msg.data.split()
                    for i, part in enumerate(parts):
                        if part == 'Room' and i + 1 < len(parts):
                            room_id = parts[i + 1]
                            room_name = self.get_room_name_by_id(room_id)
                            if room_name in self.rooms:
                                self.rooms[room_name]['detected_motion'] = True
                                self.get_logger().info(f'Motion detected in {room_name}')
                            break
                except Exception as e:
                    self.get_logger().error(f'Error parsing motion message: {e}')

    def mission_status_callback(self, msg):
        """Handle mission status updates"""
        with self.data_lock:
            if '[' in msg.data and ']' in msg.data:
                # Extract state from message like "[NAVIGATING_TO_TARGET] ..."
                start = msg.data.find('[') + 1
                end = msg.data.find(']')
                self.current_mission_state = msg.data[start:end]

    def map_callback(self, msg):
        """Handle SLAM map updates"""
        # Could be used to detect room boundaries automatically
        pass

    def get_room_name_by_id(self, room_id):
        """Map room ID to room name using configuration"""
        room_name, _ = self.room_config.get_room_by_id(room_id)
        return room_name

    def publish_visualizations(self):
        """Publish all visualization markers"""
        self.publish_room_markers()
        self.publish_drone_marker()
        self.publish_trajectory()
        self.publish_sensor_ranges()
        self.publish_investigation_target()

    def publish_room_markers(self):
        """Publish room boundary markers"""
        marker_array = MarkerArray()
        
        for i, (room_name, room_info) in enumerate(self.rooms.items()):
            # Room boundary marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "rooms"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = room_info['center'][0]
            marker.pose.position.y = room_info['center'][1]
            marker.pose.position.z = 0.1  # Slightly above ground
            marker.pose.orientation.w = 1.0
            
            # Size
            marker.scale.x = room_info['size'][0]
            marker.scale.y = room_info['size'][1]
            marker.scale.z = 0.1
            
            # Color - change if motion detected
            if room_info['detected_motion']:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.7  # More opaque when motion detected
            else:
                marker.color.r = room_info['color'][0]
                marker.color.g = room_info['color'][1]
                marker.color.b = room_info['color'][2]
                marker.color.a = room_info['color'][3]
            
            marker.lifetime.sec = 0  # Persistent
            marker_array.markers.append(marker)
            
            # Room label
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "room_labels"
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = room_info['center'][0]
            text_marker.pose.position.y = room_info['center'][1]
            text_marker.pose.position.z = 2.0  # Above room
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.5  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = room_info['name']
            if room_info['detected_motion']:
                text_marker.text += " [MOTION!]"
            
            marker_array.markers.append(text_marker)
        
        self.room_marker_pub.publish(marker_array)

    def publish_drone_marker(self):
        """Publish drone position marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "drone"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        
        # Use drone mesh if available, otherwise use arrow
        marker.mesh_resource = "package://my_drone/tello.png"
        marker.type = Marker.ARROW  # Fallback to arrow
        
        # Position
        marker.pose.position.x = self.drone_position[0]
        marker.pose.position.y = self.drone_position[1]
        marker.pose.position.z = self.drone_position[2]
        
        # Orientation
        quaternion = quaternion_from_euler(0, 0, self.drone_yaw)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        
        # Size
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Color based on mission state
        if self.current_mission_state == "EXPLORING":
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif "INVESTIGATION" in self.current_mission_state or "NAVIGATING" in self.current_mission_state:
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        
        self.drone_marker_pub.publish(marker)

    def publish_trajectory(self):
        """Publish drone trajectory"""
        if len(self.drone_trajectory) < 2:
            return
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Add trajectory points
        for pos in self.drone_trajectory:
            point = Point()
            point.x = pos[0]
            point.y = pos[1]
            point.z = pos[2]
            marker.points.append(point)
        
        marker.scale.x = 0.05  # Line width
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        self.trajectory_pub.publish(marker)

    def publish_sensor_ranges(self):
        """Publish ToF sensor range visualization"""
        if not hasattr(self, 'current_distances'):
            return
            
        marker_array = MarkerArray()
        sensor_angles = [0, -math.pi/2, math.pi/2, math.pi]  # front, left, right, back
        sensor_names = ['front', 'left', 'right', 'back']
        
        for i, (angle_offset, sensor_name) in enumerate(zip(sensor_angles, sensor_names)):
            if sensor_name not in self.current_distances:
                continue
                
            distance = self.current_distances[sensor_name]
            if distance <= 0 or distance > 12.0:  # Invalid or max range
                continue
                
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "sensor_ranges"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Start at drone position
            marker.pose.position.x = self.drone_position[0]
            marker.pose.position.y = self.drone_position[1]
            marker.pose.position.z = self.drone_position[2]
            
            # Point in sensor direction
            sensor_yaw = self.drone_yaw + angle_offset
            quaternion = quaternion_from_euler(0, 0, sensor_yaw)
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            
            # Scale based on distance
            marker.scale.x = distance
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # Color based on distance (red = close, green = far)
            if distance < 1.0:
                marker.color.r = 1.0
                marker.color.g = 0.0
            elif distance < 3.0:
                marker.color.r = 1.0
                marker.color.g = 0.5
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
            
            marker.color.b = 0.0
            marker.color.a = 0.6
            
            marker_array.markers.append(marker)
        
        self.sensor_ranges_pub.publish(marker_array)

    def publish_investigation_target(self):
        """Publish investigation target marker"""
        if self.investigation_target is None:
            return
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "investigation"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.investigation_target[0]
        marker.pose.position.y = self.investigation_target[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.1
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.investigation_marker_pub.publish(marker)

    def reset_motion_alerts(self):
        """Reset motion detection flags after some time"""
        with self.data_lock:
            for room_info in self.rooms.values():
                room_info['detected_motion'] = False

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = RVizVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
