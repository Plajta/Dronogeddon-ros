#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from enum import Enum
from threading import Lock
import time
import json

from drone_interfaces.msg import TelemetryData, ToFDistances, RCcommands
from drone_interfaces.srv import HeightCommands
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import String

class MissionState(Enum):
    IDLE = 0
    MAPPING_MODE = 1
    INVESTIGATION_MODE = 2
    NAVIGATING_TO_TARGET = 3
    INVESTIGATING_AREA = 4
    RETURNING_HOME = 5
    EMERGENCY_LAND = 6

class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')
        
        # Mission parameters
        self.home_position = (0.0, 0.0)  # home base coordinates
        self.investigation_height = 1.5  # meters
        self.navigation_speed = 40  # RC command speed
        self.investigation_duration = 15.0  # seconds to investigate an area
        self.position_tolerance = 1.0  # meters - how close to target is "arrived"
        
        # State management
        self.state = MissionState.IDLE
        self.data_lock = Lock()
        
        # Current data
        self.current_telemetry = None
        self.current_distances = None
        self.current_map = None
        
        # Mission targets
        self.investigation_target = None
        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.investigation_start_time = 0
        
        # Navigation
        self.robot_x = 0.0  # estimated position
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Safety
        self.last_heartbeat = time.time()
        self.emergency_battery_level = 20  # percent
        
        # Subscribers
        self.telemetry_sub = self.create_subscription(
            TelemetryData, 'telemetry', self.telemetry_callback, 10)
        self.distances_sub = self.create_subscription(
            ToFDistances, 'ToF_distances', self.distances_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 1)
        self.motion_sub = self.create_subscription(
            String, 'motion_detected', self.motion_callback, 10)
        self.investigation_sub = self.create_subscription(
            Point, 'investigate_location', self.investigation_callback, 10)
        
        # Publishers
        self.rc_pub = self.create_publisher(RCcommands, 'rc_commands', 10)
        self.status_pub = self.create_publisher(String, 'mission_status', 10)
        
        # Service clients
        self.height_client = self.create_client(HeightCommands, 'set_height')
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        self.safety_timer = self.create_timer(1.0, self.safety_check)  # 1Hz
        
        # Initialize in mapping mode
        self.state = MissionState.MAPPING_MODE
        self.publish_status('Mission Coordinator initialized - Mapping mode active')
        
        self.get_logger().info('Mission Coordinator initialized')

    def telemetry_callback(self, msg):
        with self.data_lock:
            self.current_telemetry = msg
            self.last_heartbeat = time.time()
            
            # Update robot pose estimation
            self.robot_yaw = math.radians(msg.yaw)
            
            # Simple dead reckoning (basic odometry)
            if hasattr(self, 'last_telemetry_time'):
                dt = 0.1  # assume 10Hz
                vx = msg.vgx / 100.0  # cm/s to m/s
                vy = msg.vgy / 100.0
                
                # Update position
                self.robot_x += (vx * math.cos(self.robot_yaw) - vy * math.sin(self.robot_yaw)) * dt
                self.robot_y += (vx * math.sin(self.robot_yaw) + vy * math.cos(self.robot_yaw)) * dt
            
            self.last_telemetry_time = time.time()

    def distances_callback(self, msg):
        with self.data_lock:
            self.current_distances = msg

    def map_callback(self, msg):
        with self.data_lock:
            self.current_map = msg

    def motion_callback(self, msg):
        """Handle motion detection events"""
        if self.state == MissionState.MAPPING_MODE:
            self.get_logger().info(f'Motion detected: {msg.data}')
            self.publish_status(f'Motion detected - Switching to investigation mode')
            self.state = MissionState.INVESTIGATION_MODE

    def investigation_callback(self, msg):
        """Handle investigation target requests"""
        if self.state in [MissionState.MAPPING_MODE, MissionState.INVESTIGATION_MODE]:
            self.investigation_target = (msg.x, msg.y, msg.z)
            self.current_target_x = msg.x
            self.current_target_y = msg.y
            self.state = MissionState.NAVIGATING_TO_TARGET
            
            self.get_logger().info(f'New investigation target: ({msg.x:.2f}, {msg.y:.2f})')
            self.publish_status(f'Navigating to investigation target ({msg.x:.1f}, {msg.y:.1f})')

    def control_loop(self):
        """Main mission control loop"""
        
        if self.state == MissionState.IDLE:
            self.send_rc_command(0, 0, 0, 0)
            
        elif self.state == MissionState.MAPPING_MODE:
            # Let autonomous explorer handle movement
            # Just monitor for investigation requests
            pass
            
        elif self.state == MissionState.INVESTIGATION_MODE:
            # Wait for investigation target
            self.send_rc_command(0, 0, 0, 0)
            
        elif self.state == MissionState.NAVIGATING_TO_TARGET:
            self.navigate_to_target()
            
        elif self.state == MissionState.INVESTIGATING_AREA:
            self.investigate_current_area()
            
        elif self.state == MissionState.RETURNING_HOME:
            self.return_to_home()
            
        elif self.state == MissionState.EMERGENCY_LAND:
            self.emergency_landing()

    def navigate_to_target(self):
        """Navigate to the current investigation target"""
        
        if not self.current_telemetry or not self.current_distances:
            return
            
        # Check for obstacles
        if self.is_obstacle_too_close():
            self.avoid_obstacle()
            return
            
        # Calculate distance to target
        dx = self.current_target_x - self.robot_x
        dy = self.current_target_y - self.robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if arrived at target
        if distance < self.position_tolerance:
            self.state = MissionState.INVESTIGATING_AREA
            self.investigation_start_time = time.time()
            self.get_logger().info('Arrived at investigation target')
            self.publish_status('Arrived at target - Beginning investigation')
            return
            
        # Calculate movement commands
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.robot_yaw)
        
        # Navigation logic
        forward_speed = 0
        yaw_speed = 0
        left_right_speed = 0
        
        if abs(angle_diff) > 0.3:  # need to rotate first
            yaw_speed = 40 if angle_diff > 0 else -40
        else:
            # Move towards target
            forward_speed = min(self.navigation_speed, int(distance * 30))
            
            # Add some lateral correction if needed
            lateral_error = dx * math.sin(self.robot_yaw) - dy * math.cos(self.robot_yaw)
            if abs(lateral_error) > 0.5:
                left_right_speed = int(lateral_error * 20)
                left_right_speed = max(-30, min(30, left_right_speed))
        
        self.send_rc_command(left_right_speed, forward_speed, 0, yaw_speed)

    def investigate_current_area(self):
        """Investigate the current area by rotating and scanning"""
        
        current_time = time.time()
        investigation_elapsed = current_time - self.investigation_start_time
        
        if investigation_elapsed < self.investigation_duration:
            # Rotate slowly for 360-degree scan
            rotation_progress = investigation_elapsed / self.investigation_duration
            if rotation_progress < 0.8:  # rotate for 80% of the time
                self.send_rc_command(0, 0, 0, 25)  # slow rotation
            else:
                self.send_rc_command(0, 0, 0, 0)  # stop and observe
        else:
            # Investigation complete
            self.get_logger().info('Investigation complete, returning to mapping mode')
            self.publish_status('Investigation complete - Returning to mapping mode')
            self.state = MissionState.MAPPING_MODE
            self.investigation_target = None

    def return_to_home(self):
        """Return to home position"""
        
        # Set home as target
        self.current_target_x = self.home_position[0]
        self.current_target_y = self.home_position[1]
        
        # Use same navigation logic as navigate_to_target
        self.navigate_to_target()
        
        # Check if arrived home
        dx = self.current_target_x - self.robot_x
        dy = self.current_target_y - self.robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.position_tolerance:
            self.get_logger().info('Arrived home, landing')
            self.publish_status('Arrived home - Landing')
            self.land_drone()
            self.state = MissionState.IDLE

    def emergency_landing(self):
        """Emergency landing procedure"""
        
        self.send_rc_command(0, 0, 0, 0)  # stop all movement
        self.land_drone()
        self.state = MissionState.IDLE

    def is_obstacle_too_close(self):
        """Check if any obstacle is too close"""
        
        if not self.current_distances:
            return False
            
        safe_distance_cm = 80  # 80cm safety margin
        
        return (self.current_distances.front < safe_distance_cm or
                self.current_distances.left < safe_distance_cm or
                self.current_distances.right < safe_distance_cm or
                self.current_distances.back < safe_distance_cm)

    def avoid_obstacle(self):
        """Simple obstacle avoidance"""
        
        if not self.current_distances:
            return
            
        # Find direction with most space
        distances = [
            self.current_distances.front,
            self.current_distances.left,
            self.current_distances.right,
            self.current_distances.back
        ]
        
        max_distance = max(distances)
        max_idx = distances.index(max_distance)
        
        # Move towards the direction with most space
        if max_idx == 0:  # front
            self.send_rc_command(0, 20, 0, 0)
        elif max_idx == 1:  # left
            self.send_rc_command(-20, 0, 0, 0)
        elif max_idx == 2:  # right
            self.send_rc_command(20, 0, 0, 0)
        else:  # back
            self.send_rc_command(0, -20, 0, 0)

    def safety_check(self):
        """Perform safety checks"""
        
        current_time = time.time()
        
        # Check heartbeat
        if current_time - self.last_heartbeat > 5.0:
            self.get_logger().warn('Lost telemetry data - Emergency landing')
            self.state = MissionState.EMERGENCY_LAND
            return
            
        # Check battery level
        if self.current_telemetry and self.current_telemetry.bat < self.emergency_battery_level:
            self.get_logger().warn(f'Low battery ({self.current_telemetry.bat}%) - Returning home')
            self.state = MissionState.RETURNING_HOME
            return
            
        # Check if drone is too high or too low
        if self.current_telemetry:
            height_m = self.current_telemetry.h / 100.0
            if height_m > 3.0:  # too high
                self.get_logger().warn('Drone too high - Descending')
                self.send_rc_command(0, 0, -20, 0)
            elif height_m < 0.5 and self.state != MissionState.IDLE:  # too low
                self.get_logger().warn('Drone too low - Ascending')
                self.send_rc_command(0, 0, 20, 0)

    def send_rc_command(self, left_right, forward_back, up_down, yaw):
        """Send RC command to drone"""
        
        msg = RCcommands()
        msg.left_right_velocity = int(left_right)
        msg.forward_backward_velocity = int(forward_back)
        msg.up_down_velocity = int(up_down)
        msg.yaw_velocity = int(yaw)
        
        self.rc_pub.publish(msg)

    def land_drone(self):
        """Send land command"""
        
        if self.height_client.wait_for_service(timeout_sec=2.0):
            request = HeightCommands.Request()
            request.command = 0  # land
            future = self.height_client.call_async(request)

    def publish_status(self, status):
        """Publish mission status"""
        
        msg = String()
        msg.data = f"[{self.state.name}] {status}"
        self.status_pub.publish(msg)
        self.get_logger().info(status)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def save_mission_log(self):
        """Save mission log to file"""
        
        log_data = {
            'timestamp': time.time(),
            'robot_position': (self.robot_x, self.robot_y, self.robot_yaw),
            'state': self.state.name,
            'investigation_target': self.investigation_target,
            'battery_level': self.current_telemetry.bat if self.current_telemetry else None
        }
        
        try:
            with open('mission_log.json', 'a') as f:
                json.dump(log_data, f)
                f.write('\n')
        except Exception as e:
            self.get_logger().error(f'Error saving mission log: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    coordinator = MissionCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        coordinator.save_mission_log()
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
