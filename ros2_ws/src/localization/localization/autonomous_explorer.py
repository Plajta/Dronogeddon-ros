#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from enum import Enum
from threading import Lock
import time
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from drone_interfaces.msg import TelemetryData, ToFDistances, RCcommands
from drone_interfaces.srv import HeightCommands
from std_srvs.srv import Empty

class ExplorationState(Enum):
    IDLE = 0
    TAKEOFF = 1
    EXPLORING = 2
    MOVING_TO_FRONTIER = 3
    AVOIDING_OBSTACLE = 4
    LANDING = 5
    COMPLETED = 6

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Exploration parameters
        self.exploration_height = 1.5  # meters above ground
        self.safe_distance = 0.8  # minimum distance to obstacles (meters)
        self.exploration_speed = 30  # RC command speed (0-100)
        self.rotation_speed = 50  # rotation speed for scanning
        
        # State management
        self.state = ExplorationState.IDLE
        self.data_lock = Lock()
        
        # Current sensor data
        self.current_distances = None
        self.current_telemetry = None
        self.current_map = None
        
        # Navigation targets
        self.target_x = 0.0
        self.target_y = 0.0
        self.frontiers = []
        self.current_frontier_idx = 0
        
        # Safety and control
        self.last_command_time = 0
        self.obstacle_avoidance_start = 0
        self.scan_start_time = 0
        self.scan_duration = 8.0  # seconds to scan at each position
        
        # Subscribers
        self.distances_sub = self.create_subscription(
            ToFDistances, 'ToF_distances', self.distances_callback, 10)
        self.telemetry_sub = self.create_subscription(
            TelemetryData, 'telemetry', self.telemetry_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 1)
        
        # Publishers
        self.rc_pub = self.create_publisher(RCcommands, 'rc_commands', 10)
        self.status_pub = self.create_publisher(String, 'exploration_status', 10)
        self.mission_status_pub = self.create_publisher(String, 'mission_status', 10)
        
        # Service clients
        self.height_client = self.create_client(HeightCommands, 'height_commands')
        
        # Services
        self.start_exploration_service = self.create_service(
            Empty, 'start_exploration', self.start_exploration_callback)
        
        # Main control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        # Exploration timer
        self.exploration_timer = self.create_timer(2.0, self.update_exploration_targets)
        
        self.get_logger().info('Autonomous Explorer initialized. Call start_exploration service to begin.')

    def distances_callback(self, msg):
        with self.data_lock:
            self.current_distances = msg

    def telemetry_callback(self, msg):
        with self.data_lock:
            self.current_telemetry = msg

    def map_callback(self, msg):
        with self.data_lock:
            self.current_map = msg

    def start_exploration_callback(self, request, response):
        """Service callback to start autonomous exploration"""
        self.start_exploration()
        response.success = True
        response.message = "Exploration started"
        return response
    
    def start_exploration(self):
        """Start autonomous exploration"""
        self.state = ExplorationState.TAKEOFF
        self.publish_status('Starting exploration - Taking off')
        self.publish_mission_status('[EXPLORATION] Starting autonomous exploration')
        
        # Send takeoff command
        if self.height_client.wait_for_service(timeout_sec=5.0):
            request = HeightCommands.Request()
            request.command = 1  # takeoff
            future = self.height_client.call_async(request)

    def stop_exploration(self):
        """Stop exploration and land"""
        self.state = ExplorationState.LANDING
        self.publish_status('Stopping exploration - Landing')
        self.send_rc_command(0, 0, 0, 0)  # stop movement
        
        # Send land command
        if self.height_client.wait_for_service(timeout_sec=5.0):
            request = HeightCommands.Request()
            request.command = 0  # land
            future = self.height_client.call_async(request)

    def control_loop(self):
        """Main control loop"""
        
        if self.state == ExplorationState.IDLE:
            return
            
        elif self.state == ExplorationState.TAKEOFF:
            # Wait for takeoff to complete
            if self.current_telemetry and self.current_telemetry.h > 50:  # 0.5m height (reduced threshold)
                if not hasattr(self, 'takeoff_completed'):
                    self.takeoff_completed = True
                    self.state = ExplorationState.EXPLORING
                    self.scan_start_time = time.time()
                    self.get_logger().info('Takeoff complete, starting exploration')
                    self.publish_status('Takeoff complete - Beginning exploration')
            else:
                # Debug: log current height (less frequently)
                if not hasattr(self, 'last_height_log') or time.time() - self.last_height_log > 2.0:
                    self.last_height_log = time.time()
                    if self.current_telemetry:
                        self.get_logger().info(f'Waiting for takeoff... Current height: {self.current_telemetry.h}cm')
                    else:
                        self.get_logger().info('Waiting for takeoff... No telemetry data')
                
        elif self.state == ExplorationState.EXPLORING:
            self.explore_current_area()
            
        elif self.state == ExplorationState.MOVING_TO_FRONTIER:
            self.move_to_frontier()
            
        elif self.state == ExplorationState.AVOIDING_OBSTACLE:
            self.avoid_obstacle()
            
        elif self.state == ExplorationState.LANDING:
            # Wait for landing to complete
            if self.current_telemetry and self.current_telemetry.h < 10:  # close to ground
                self.state = ExplorationState.COMPLETED
                self.get_logger().info('Exploration completed')
                self.publish_status('Exploration completed')
                self.publish_mission_status('[IDLE] Exploration completed - drone landed')

    def explore_current_area(self):
        """Explore current area by rotating and scanning"""
        
        if not self.current_distances:
            return
            
        # Check for obstacles
        if self.is_obstacle_detected():
            self.state = ExplorationState.AVOIDING_OBSTACLE
            self.obstacle_avoidance_start = time.time()
            return
            
        current_time = time.time()
        
        # Rotate slowly to scan the area
        if current_time - self.scan_start_time < self.scan_duration:
            # Rotate slowly for 360-degree scan
            self.send_rc_command(0, 0, 0, 30)  # increased rotation speed
            self.get_logger().info(f'Scanning area... {current_time - self.scan_start_time:.1f}s / {self.scan_duration}s')
        else:
            # Scanning complete, find next frontier
            self.send_rc_command(0, 0, 0, 0)  # stop rotation
            self.get_logger().info('Scan complete, looking for frontiers')
            self.find_frontiers()  # Update frontier list
            if self.frontiers and self.current_frontier_idx < len(self.frontiers):
                self.state = ExplorationState.MOVING_TO_FRONTIER
                target_x, target_y = self.frontiers[self.current_frontier_idx]
                self.get_logger().info(f'Moving to frontier {self.current_frontier_idx + 1}/{len(self.frontiers)} at ({target_x:.1f}, {target_y:.1f})')
            else:
                # No more frontiers, continue exploring or complete
                self.current_frontier_idx = 0
                self.scan_start_time = time.time()
                self.get_logger().info('No frontiers found, continuing exploration')

    def move_to_frontier(self):
        """Move towards the current frontier target"""
        
        if not self.current_distances or not self.current_telemetry:
            return
            
        # Check for obstacles
        if self.is_obstacle_detected():
            self.state = ExplorationState.AVOIDING_OBSTACLE
            self.obstacle_avoidance_start = time.time()
            return
            
        if not self.frontiers or self.current_frontier_idx >= len(self.frontiers):
            # No valid frontier, go back to exploring
            self.state = ExplorationState.EXPLORING
            self.scan_start_time = time.time()
            return
            
        target_x, target_y = self.frontiers[self.current_frontier_idx]
        
        # Get current position from telemetry or estimate
        current_x = getattr(self, 'current_x', 0.0)
        current_y = getattr(self, 'current_y', 0.0)
        
        # Calculate relative position to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.5:  # reached frontier
            self.current_frontier_idx += 1
            self.state = ExplorationState.EXPLORING
            self.scan_start_time = time.time()
            self.get_logger().info('Reached frontier, scanning area')
            return
            
        # Calculate movement commands
        target_angle = math.atan2(dy, dx)
        current_yaw = math.radians(self.current_telemetry.yaw)
        angle_diff = self.normalize_angle(target_angle - current_yaw)
        
        # Movement logic - actually move forward
        forward_speed = 30 if abs(angle_diff) < 0.3 else 0  # move forward when aligned
        yaw_speed = 0
        
        if abs(angle_diff) > 0.2:  # need to rotate first
            yaw_speed = 30 if angle_diff > 0 else -30
        else:
            forward_speed = min(self.exploration_speed, int(distance * 20))
            
        self.send_rc_command(0, forward_speed, 0, yaw_speed)

    def avoid_obstacle(self):
        """Simple obstacle avoidance behavior"""
        
        if not self.current_distances:
            return
            
        current_time = time.time()
        
        # Obstacle avoidance timeout
        if current_time - self.obstacle_avoidance_start > 10.0:
            self.state = ExplorationState.EXPLORING
            self.scan_start_time = time.time()
            return
            
        # Simple avoidance: rotate away from closest obstacle
        distances = [
            self.current_distances.front / 100.0,
            self.current_distances.left / 100.0,
            self.current_distances.right / 100.0,
            self.current_distances.back / 100.0
        ]
        
        # Find direction with most space
        max_distance = max(distances)
        max_idx = distances.index(max_distance)
        
        if max_distance > self.safe_distance * 1.5:
            # Found safe direction
            if max_idx == 0:  # front
                self.send_rc_command(0, 20, 0, 0)
            elif max_idx == 1:  # left
                self.send_rc_command(-20, 0, 0, 0)
            elif max_idx == 2:  # right
                self.send_rc_command(20, 0, 0, 0)
            else:  # back
                self.send_rc_command(0, -20, 0, 0)
        else:
            # All directions blocked, rotate to find opening
            self.send_rc_command(0, 0, 0, 40)

    def is_obstacle_detected(self):
        """Check if obstacles are too close"""
        
        if not self.current_distances:
            return False
            
        min_distance_cm = self.safe_distance * 100  # convert to cm
        
        return (self.current_distances.front < min_distance_cm or
                self.current_distances.left < min_distance_cm or
                self.current_distances.right < min_distance_cm or
                self.current_distances.back < min_distance_cm)

    def update_exploration_targets(self):
        """Update list of exploration frontiers from SLAM map"""
        
        if not self.current_map:
            return
            
        # Simple frontier detection from occupancy grid
        # This is a simplified version - in practice you'd want more sophisticated frontier detection
        self.frontiers = []
        
        width = self.current_map.info.width
        height = self.current_map.info.height
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        data = np.array(self.current_map.data).reshape((height, width))
        
        # Find frontier cells (unknown cells adjacent to free cells)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == -1:  # unknown cell
                    # Check if adjacent to free space
                    adjacent_free = False
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if data[y + dy, x + dx] == 0:  # free cell
                                adjacent_free = True
                                break
                        if adjacent_free:
                            break
                    
                    if adjacent_free:
                        # Convert to world coordinates
                        world_x = x * resolution + origin_x
                        world_y = y * resolution + origin_y
                        self.frontiers.append((world_x, world_y))
        
        # Limit number of frontiers and sort by distance
        if len(self.frontiers) > 10:
            self.frontiers = self.frontiers[:10]

    def send_rc_command(self, left_right, forward_back, up_down, yaw):
        """Send RC command to drone"""
        
        msg = RCcommands()
        msg.left_right_velocity = int(left_right)
        msg.forward_backward_velocity = int(forward_back)
        msg.up_down_velocity = int(up_down)
        msg.yaw_velocity = int(yaw)
        
        # Debug: log commands being sent
        if any([left_right, forward_back, up_down, yaw]):
            self.get_logger().info(f'Sending RC command: LR={left_right}, FB={forward_back}, UD={up_down}, YAW={yaw}')
        
        self.rc_pub.publish(msg)
        self.last_command_time = time.time()

    def publish_status(self, status):
        """Publish exploration status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def publish_mission_status(self, status):
        """Publish mission status for map saver"""
        msg = String()
        msg.data = status
        self.mission_status_pub.publish(msg)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    
    explorer = AutonomousExplorer()
    
    # Start exploration automatically after 3 seconds
    def delayed_start():
        time.sleep(3.0)
        explorer.start_exploration()
    
    import threading
    start_thread = threading.Thread(target=delayed_start)
    start_thread.daemon = True
    start_thread.start()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        explorer.stop_exploration()
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
