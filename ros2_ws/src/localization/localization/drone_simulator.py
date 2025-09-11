#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import json
import cv2
import os
from threading import Lock
import time
from enum import Enum

from drone_interfaces.msg import TelemetryData, ToFDistances, RCcommands
from drone_interfaces.srv import HeightCommands
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
# Import centralized tf_transformations fix
from .tf_transformations_fix import quaternion_from_euler

class DroneState(Enum):
    LANDED = 0
    TAKING_OFF = 1
    FLYING = 2
    LANDING = 3

class DroneSimulator(Node):
    def __init__(self):
        super().__init__('drone_simulator')
        
        # Simulation parameters
        self.simulation_map = None
        self.map_resolution = 0.05  # meters per pixel
        self.map_origin_x = -10.0   # map origin in world coordinates
        self.map_origin_y = -10.0
        self.map_width = 400        # pixels
        self.map_height = 400       # pixels
        
        # Drone state
        self.drone_state = DroneState.LANDED
        self.position_x = 0.0       # world coordinates (meters)
        self.position_y = 0.0
        self.position_z = 0.0       # height in meters
        self.yaw = 0.0              # radians
        self.velocity_x = 0.0       # m/s
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        self.yaw_velocity = 0.0     # rad/s
        
        # Physical parameters
        self.max_speed = 2.0        # m/s
        self.max_yaw_rate = 1.0     # rad/s
        self.takeoff_height = 1.5   # meters
        self.takeoff_speed = 0.5    # m/s
        self.landing_speed = 0.3    # m/s
        
        # ToF sensor configuration (4 sensors + matrix sensor)
        self.tof_range = 4.0        # maximum range in meters
        self.tof_angles = [0, math.pi/2, math.pi, -math.pi/2]  # forward, left, back, right
        self.matrix_sensor_fov = math.pi/3  # 60 degrees field of view
        self.matrix_sensor_resolution = 8   # 8x8 matrix
        
        # Sensor noise and limitations
        self.sensor_noise_std = 0.02    # Standard deviation for sensor noise (2cm)
        self.max_range_variation = 0.3  # Variation when at max range (30cm)
        self.measurement_error_prob = 0.05  # 5% chance of measurement error
        
        # Control inputs
        self.rc_commands = RCcommands()
        self.data_lock = Lock()
        
        # Load simulation environment
        self.load_simulation_environment()
        
        # Publishers
        self.telemetry_pub = self.create_publisher(TelemetryData, 'telemetry', 10)
        self.distances_pub = self.create_publisher(ToFDistances, 'ToF_distances', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'drone_pose', 10)
        
        # Subscribers
        self.rc_sub = self.create_subscription(RCcommands, 'rc_commands', self.rc_callback, 10)
        
        # Services
        self.height_service = self.create_service(HeightCommands, 'height_commands', self.height_command_callback)
        
        # Simulation timer (50Hz for smooth simulation)
        self.sim_timer = self.create_timer(0.02, self.simulation_step)
        
        # Sensor publishing timer (10Hz)
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)
        
        self.get_logger().info('Drone Simulator initialized')
        self.get_logger().info(f'Environment: {self.map_width}x{self.map_height} pixels, resolution: {self.map_resolution}m/px')
    
    def load_simulation_environment(self):
        """Load or create simulation environment"""
        # Try to load from saved map first
        maps_dir = "saved_maps"
        if os.path.exists(maps_dir):
            map_files = [f for f in os.listdir(maps_dir) if f.endswith('.png') and not 'trajectory' in f]
            if map_files:
                # Use the most recent map
                latest_map = sorted(map_files)[-1]
                map_path = os.path.join(maps_dir, latest_map)
                self.get_logger().info(f'Loading simulation environment from: {map_path}')
                self.load_map_from_file(map_path)
                return
        
        # Create default environment if no saved maps
        self.get_logger().info('Creating default simulation environment')
        self.create_default_environment()
    
    def load_map_from_file(self, map_path):
        """Load simulation environment from saved exploration map"""
        try:
            # Load the image
            img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                raise Exception(f"Could not load image: {map_path}")
            
            # Convert image to occupancy grid format
            # Image: 0=black (occupied), 255=white (free), 128=gray (unknown)
            # Occupancy: 0=free, 100=occupied, -1=unknown
            self.simulation_map = np.zeros_like(img, dtype=np.int8)
            self.simulation_map[img == 255] = 0    # Free space
            self.simulation_map[img == 0] = 100    # Occupied space
            self.simulation_map[img == 128] = -1   # Unknown space (treat as free for simulation)
            
            # Update map dimensions
            self.map_height, self.map_width = img.shape
            
            # Try to load metadata for proper scaling
            metadata_path = map_path.replace('.png', '_metadata.json')
            if os.path.exists(metadata_path):
                with open(metadata_path, 'r') as f:
                    metadata = json.load(f)
                    self.map_resolution = metadata.get('resolution', 0.05)
                    self.map_origin_x = metadata.get('origin_x', -10.0)
                    self.map_origin_y = metadata.get('origin_y', -10.0)
            
            # Flip image (PNG has origin at top-left, occupancy grid at bottom-left)
            self.simulation_map = np.flipud(self.simulation_map)
            
            self.get_logger().info(f'Loaded map: {self.map_width}x{self.map_height}, resolution: {self.map_resolution}')
            
        except Exception as e:
            self.get_logger().error(f'Error loading map: {e}')
            self.create_default_environment()
    
    def create_default_environment(self):
        """Create a default simulation environment with rooms"""
        # Create a simple apartment layout
        self.simulation_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Add outer walls
        self.simulation_map[0:5, :] = 100      # Top wall
        self.simulation_map[-5:, :] = 100     # Bottom wall
        self.simulation_map[:, 0:5] = 100     # Left wall
        self.simulation_map[:, -5:] = 100     # Right wall
        
        # Add internal walls to create rooms
        # Vertical wall (dividing left/right)
        self.simulation_map[50:350, 195:205] = 100
        
        # Horizontal wall (dividing top/bottom)
        self.simulation_map[195:205, 50:350] = 100
        
        # Add doors (gaps in walls)
        self.simulation_map[180:220, 195:205] = 0  # Door in vertical wall
        self.simulation_map[195:205, 180:220] = 0  # Door in horizontal wall
        
        # Add some furniture/obstacles
        # Living room furniture
        self.simulation_map[80:120, 80:120] = 100   # Sofa
        self.simulation_map[300:320, 300:340] = 100 # Table
        
        # Kitchen furniture
        self.simulation_map[80:100, 250:350] = 100  # Kitchen counter
        
        # Bedroom furniture
        self.simulation_map[250:300, 80:130] = 100  # Bed
        
        self.get_logger().info('Created default apartment environment')
    
    def rc_callback(self, msg):
        """Handle RC commands from exploration system"""
        with self.data_lock:
            self.rc_commands = msg
    
    def height_command_callback(self, request, response):
        """Handle takeoff/landing commands"""
        if request.command == 1:  # Takeoff
            if self.drone_state == DroneState.LANDED:
                self.drone_state = DroneState.TAKING_OFF
                self.get_logger().info('Simulator: Starting takeoff')
                response.success = True
            else:
                response.success = False
        elif request.command == 0:  # Land
            if self.drone_state == DroneState.FLYING:
                self.drone_state = DroneState.LANDING
                self.get_logger().info('Simulator: Starting landing')
                response.success = True
            else:
                response.success = False
        else:
            response.success = False
        
        return response
    
    def simulation_step(self):
        """Main simulation step - update drone physics"""
        dt = 0.02  # 50Hz simulation
        
        with self.data_lock:
            if self.drone_state == DroneState.LANDED:
                return
            
            elif self.drone_state == DroneState.TAKING_OFF:
                # Simple takeoff - just increase altitude
                self.position_z += self.takeoff_speed * dt
                if self.position_z >= self.takeoff_height:
                    self.position_z = self.takeoff_height
                    self.drone_state = DroneState.FLYING
                    self.get_logger().info('Simulator: Takeoff complete')
            
            elif self.drone_state == DroneState.LANDING:
                # Simple landing - decrease altitude
                self.position_z -= self.landing_speed * dt
                if self.position_z <= 0.0:
                    self.position_z = 0.0
                    self.drone_state = DroneState.LANDED
                    self.get_logger().info('Simulator: Landing complete')
            
            elif self.drone_state == DroneState.FLYING:
                # Process RC commands and update position
                self.update_drone_physics(dt)
    
    def update_drone_physics(self, dt):
        """Update drone position based on RC commands"""
        # Convert RC commands (0-100) to velocities
        # RC commands: forward/backward, left/right, up/down, yaw
        
        # Forward/backward (pitch)
        forward_cmd = (self.rc_commands.pitch - 50) / 50.0  # -1 to 1
        # Left/right (roll)
        right_cmd = (self.rc_commands.roll - 50) / 50.0     # -1 to 1
        # Up/down (throttle)
        up_cmd = (self.rc_commands.throttle - 50) / 50.0    # -1 to 1
        # Yaw rotation
        yaw_cmd = (self.rc_commands.yaw_velocity - 50) / 50.0  # -1 to 1
        
        # Apply deadzone
        deadzone = 0.1
        if abs(forward_cmd) < deadzone: forward_cmd = 0.0
        if abs(right_cmd) < deadzone: right_cmd = 0.0
        if abs(up_cmd) < deadzone: up_cmd = 0.0
        if abs(yaw_cmd) < deadzone: yaw_cmd = 0.0
        
        # Update yaw first
        self.yaw_velocity = yaw_cmd * self.max_yaw_rate
        self.yaw += self.yaw_velocity * dt
        self.yaw = self.normalize_angle(self.yaw)
        
        # Calculate world velocities based on drone orientation
        # Forward/backward in drone frame
        vel_forward = forward_cmd * self.max_speed
        vel_right = right_cmd * self.max_speed
        
        # Convert to world frame
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        self.velocity_x = vel_forward * cos_yaw - vel_right * sin_yaw
        self.velocity_y = vel_forward * sin_yaw + vel_right * cos_yaw
        self.velocity_z = up_cmd * self.max_speed * 0.5  # Slower vertical movement
        
        # Update position
        new_x = self.position_x + self.velocity_x * dt
        new_y = self.position_y + self.velocity_y * dt
        new_z = self.position_z + self.velocity_z * dt
        
        # Check collision with environment
        if self.is_position_valid(new_x, new_y):
            self.position_x = new_x
            self.position_y = new_y
        else:
            # Stop if collision detected
            self.velocity_x = 0.0
            self.velocity_y = 0.0
        
        # Limit altitude
        if new_z < 0.1:
            new_z = 0.1
        elif new_z > 5.0:
            new_z = 5.0
        self.position_z = new_z
    
    def is_position_valid(self, x, y):
        """Check if position is valid (not in obstacle)"""
        if self.simulation_map is None:
            return True
        
        # Convert world coordinates to map coordinates
        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)
        
        # Check bounds
        if map_x < 0 or map_x >= self.map_width or map_y < 0 or map_y >= self.map_height:
            return False
        
        # Check if occupied (add small safety margin)
        safety_radius = int(0.3 / self.map_resolution)  # 30cm safety margin
        for dx in range(-safety_radius, safety_radius + 1):
            for dy in range(-safety_radius, safety_radius + 1):
                check_x = map_x + dx
                check_y = map_y + dy
                if (0 <= check_x < self.map_width and 0 <= check_y < self.map_height):
                    if self.simulation_map[check_y, check_x] == 100:  # Occupied
                        return False
        
        return True
    
    def simulate_tof_sensors(self):
        """Simulate ToF sensor readings"""
        distances = ToFDistances()
        
        if self.simulation_map is None:
            # Default readings if no map
            distances.front = 400  # 4.0m in cm
            distances.back = 400
            distances.left = 400
            distances.right = 400
            distances.matrix = [400] * 64  # 8x8 matrix
            return distances
        
        # Simulate 4 directional ToF sensors
        sensor_distances = []
        for angle_offset in self.tof_angles:
            sensor_angle = self.yaw + angle_offset
            distance = self.raycast(self.position_x, self.position_y, sensor_angle)
            distance = self.apply_sensor_limitations(distance)
            sensor_distances.append(min(int(distance * 100), 400))  # Convert to cm, max 400cm
        
        distances.front = sensor_distances[0]
        distances.left = sensor_distances[1]
        distances.back = sensor_distances[2]
        distances.right = sensor_distances[3]
        
        # Simulate matrix sensor (forward-facing 8x8 grid)
        matrix_data = []
        matrix_half_fov = self.matrix_sensor_fov / 2
        for row in range(self.matrix_sensor_resolution):
            for col in range(self.matrix_sensor_resolution):
                # Calculate angle for this matrix element
                angle_v = (row / (self.matrix_sensor_resolution - 1) - 0.5) * matrix_half_fov
                angle_h = (col / (self.matrix_sensor_resolution - 1) - 0.5) * matrix_half_fov
                
                # For simplicity, just use horizontal angle (2D simulation)
                sensor_angle = self.yaw + angle_h
                distance = self.raycast(self.position_x, self.position_y, sensor_angle)
                distance = self.apply_sensor_limitations(distance)
                matrix_data.append(min(int(distance * 100), 400))
        
        distances.matrix = matrix_data
        return distances
    
    def raycast(self, start_x, start_y, angle):
        """Cast a ray and return distance to first obstacle"""
        if self.simulation_map is None:
            return self.tof_range
        
        step_size = self.map_resolution / 2  # Half pixel steps for accuracy
        max_steps = int(self.tof_range / step_size)
        
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        
        for step in range(max_steps):
            # Current ray position
            ray_x = start_x + step * step_size * cos_angle
            ray_y = start_y + step * step_size * sin_angle
            
            # Convert to map coordinates
            map_x = int((ray_x - self.map_origin_x) / self.map_resolution)
            map_y = int((ray_y - self.map_origin_y) / self.map_resolution)
            
            # Check bounds
            if map_x < 0 or map_x >= self.map_width or map_y < 0 or map_y >= self.map_height:
                return step * step_size
            
            # Check if hit obstacle
            if self.simulation_map[map_y, map_x] == 100:
                return step * step_size
        
        return self.tof_range
    
    def apply_sensor_limitations(self, true_distance):
        """Apply realistic sensor limitations and noise"""
        # If distance is at or near maximum range, simulate sensor limitations
        if true_distance >= self.tof_range:
            # When sensor can't reach the target, it returns a random value
            # typically somewhere between 80-100% of max range with variation
            base_reading = self.tof_range * (0.8 + 0.2 * np.random.random())
            # Add random variation (sensor doesn't know it can't reach)
            variation = np.random.normal(0, self.max_range_variation)
            return max(0.1, base_reading + variation)
        
        # Add measurement noise for normal readings
        noise = np.random.normal(0, self.sensor_noise_std)
        noisy_distance = true_distance + noise
        
        # Occasional measurement errors (spikes, dropouts)
        if np.random.random() < self.measurement_error_prob:
            if np.random.random() < 0.5:
                # Spike - false short reading
                noisy_distance = true_distance * (0.3 + 0.4 * np.random.random())
            else:
                # Dropout - false long reading (but not max range)
                noisy_distance = true_distance + np.random.uniform(0.5, 1.5)
        
        # Ensure minimum reading (sensors can't read exactly 0)
        return max(0.05, noisy_distance)
    
    def publish_sensor_data(self):
        """Publish simulated sensor data"""
        if self.drone_state == DroneState.LANDED:
            return
        
        current_time = self.get_clock().now()
        
        # Publish telemetry
        telemetry = TelemetryData()
        telemetry.h = int(self.position_z * 100)  # Height in cm
        telemetry.yaw = int(math.degrees(self.yaw))  # Yaw in degrees
        telemetry.vgx = int(self.velocity_x * 100)  # Velocity in cm/s
        telemetry.vgy = int(self.velocity_y * 100)
        telemetry.vgz = int(self.velocity_z * 100)
        telemetry.bat = 85  # Simulated battery level
        telemetry.time = int(time.time())
        self.telemetry_pub.publish(telemetry)
        
        # Publish ToF distances
        distances = self.simulate_tof_sensors()
        self.distances_pub.publish(distances)
        
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.position.x = self.position_x
        pose_msg.pose.position.y = self.position_y
        pose_msg.pose.position.z = self.position_z
        
        # Convert yaw to quaternion
        quat = quaternion_from_euler(0, 0, self.yaw)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.pose_pub.publish(pose_msg)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    
    simulator = DroneSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info('Drone Simulator shutting down')
    
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
