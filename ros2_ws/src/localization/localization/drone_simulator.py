#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import json
import cv2
import os
import yaml
from threading import Lock
import time
import random
from enum import Enum
import matplotlib.pyplot as plt
import matplotlib.patches as patches

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
    FALLING = 4

class DroneSimulator(Node):
    # Simulation timing constants
    SIMULATION_TIMER_INTERVAL = 0.02  # 50Hz simulation
    
    def __init__(self):
        super().__init__('drone_simulator')
        
        # Simulation parameters
        self.simulation_map = None
        self.map_resolution = 0.05  # meters per pixel
        self.map_origin_x = -10.0   # map origin in world coordinates
        self.map_origin_y = -10.0
        self.map_width = 400        # pixels
        self.map_height = 400       # pixels
        self.doors = []             # Store door positions for visualization
        
        # Drone state
        self.drone_state = DroneState.LANDED
        self.position_x = 0.0       # world coordinates (meters) - will be set from config
        self.position_y = 0.0       # will be set from config
        self.position_z = 0.0       # height in meters - will be set from config
        self.yaw = 0.0              # radians
        self.velocity_x = 0.0       # m/s
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        self.yaw_velocity = 0.0     # rad/s
        
        # Physical parameters
        self.max_speed = 0.5        # m/s (reduced from 2.0 for safer indoor navigation)
        self.max_yaw_rate = 0.5     # rad/s (reduced from 1.0 for smoother turns)
        self.takeoff_height = 1.5   # meters
        self.takeoff_speed = 0.5    # m/s (reduced from 2.0)
        self.landing_speed = 0.3    # m/s
        
        # ToF sensor configuration (4 sensors + matrix sensor)
        self.tof_range = 4.0        # maximum range in meters
        self.tof_angles = [0, math.pi/2, math.pi, -math.pi/2]  # forward, left, back, right
        self.matrix_sensor_fov = math.pi/3  # 60 degrees field of view
        self.matrix_sensor_resolution = 8   # 8x8 matrix
        
        # Sensor noise and limitations
        self.sensor_noise_std = 0.01    # Standard deviation for sensor noise (1cm)
        self.max_range_variation = 0.1  # Variation when at max range (10cm)
        self.measurement_error_prob = 0.01  # 1% chance of measurement error (reduced from 5%)
        
        # Control inputs
        self.rc_commands = RCcommands()
        self.data_lock = Lock()
        
        # Time tracking for accurate physics simulation
        self.last_simulation_time = None
        
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
        self.sim_timer = self.create_timer(self.SIMULATION_TIMER_INTERVAL, self.simulation_step)
        
        # Sensor publishing timer (10Hz)
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)
        
        self.get_logger().info('Drone Simulator initialized')
        self.get_logger().info(f'Environment: {self.map_width}x{self.map_height} pixels, resolution: {self.map_resolution}m/px')
    
    def load_simulation_environment(self):
        """Load simulation environment from rooms.yaml configuration"""
        self.get_logger().info('Creating simulation environment from rooms.yaml configuration')
        self.rooms_config = self.load_rooms_config()
        self.create_environment_from_config()
    
    def load_rooms_config(self):
        """Load rooms configuration from rooms.yaml"""
        config_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..', 'config', 'rooms.yaml')
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                self.get_logger().info(f'Loaded rooms configuration from: {config_path}')
                return config
        except Exception as e:
            self.get_logger().error(f'CRITICAL ERROR: Cannot load rooms config from {config_path}: {e}')
            self.get_logger().error('Simulator cannot start without valid rooms.yaml configuration')
            raise RuntimeError(f'Failed to load required configuration file: {config_path}')
    
    
    def create_environment_from_config(self):
        """Create simulation environment from loaded rooms configuration"""
        # Get environment settings
        env_config = self.rooms_config.get('environment', {})
        wall_thickness = env_config.get('wall_thickness', 3)
        
        # Set drone start position from configuration
        drone_start_pos = env_config.get('drone_start_position', [0.0, 0.0, 0.0])
        if isinstance(drone_start_pos, list) and len(drone_start_pos) >= 2:
            self.position_x = float(drone_start_pos[0])
            self.position_y = float(drone_start_pos[1])
            if len(drone_start_pos) >= 3:
                self.position_z = float(drone_start_pos[2])
            self.get_logger().info(f'Set drone start position from config: ({self.position_x:.2f}, {self.position_y:.2f}, {self.position_z:.2f})')
        else:
            self.get_logger().warn('Invalid drone_start_position in config, using default (0, 0, 0)')
        
        # Initialize empty map (all unknown/free space)
        self.simulation_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Process each room from configuration
        rooms = self.rooms_config.get('rooms', {})
        
        for room_name, room_data in rooms.items():
            # Validate required parameters
            if 'center' not in room_data:
                raise ValueError(f"Room '{room_name}' is missing required 'center' parameter in rooms.yaml")
            if 'size' not in room_data:
                raise ValueError(f"Room '{room_name}' is missing required 'size' parameter in rooms.yaml")
            
            center = room_data['center']
            size = room_data['size']
            
            # Validate parameter formats
            if not isinstance(center, list) or len(center) != 2:
                raise ValueError(f"Room '{room_name}' center must be a list of 2 numbers [x, y]")
            if not isinstance(size, list) or len(size) != 2:
                raise ValueError(f"Room '{room_name}' size must be a list of 2 numbers [width, height]")
            
            # Convert to map coordinates
            room_x, room_y = self.world_to_map(center[0], center[1])
            room_w = int(size[0] / self.map_resolution)
            room_h = int(size[1] / self.map_resolution)
            
            # Add room walls for all rooms
            self.add_room_walls(room_x, room_y, room_w, room_h, wall_thickness)
        
        # Add doors from configuration
        self.add_doors_from_config()
        
        # Add furniture from configuration
        self.add_furniture_from_config()
        
        self.get_logger().info(f'Created environment with {len(rooms)} rooms from configuration')
        
        # Validate drone start position after environment is created
        if not self.is_position_valid(self.position_x, self.position_y):
            self.get_logger().warn(f'Drone start position ({self.position_x:.2f}, {self.position_y:.2f}) is not valid (in wall/obstacle)!')
            self.get_logger().warn('Consider adjusting drone_start_position in rooms.yaml')
        else:
            self.get_logger().info(f'Drone start position ({self.position_x:.2f}, {self.position_y:.2f}) is valid')
        
        # Generate visualization of the environment (async to not block startup)
        # Run in a separate thread to avoid blocking
        import threading
        viz_thread = threading.Thread(target=self.save_environment_visualization, daemon=True)
        viz_thread.start()
    
    def add_doors_from_config(self):
        """Add doors between rooms based on configuration"""
        doors = self.rooms_config.get('doors', [])
        env_config = self.rooms_config.get('environment', {})
        door_size = env_config.get('door_size', 12)
        wall_thickness = env_config.get('wall_thickness', 3)
        
        for i, door in enumerate(doors):
            # Validate required parameters
            if 'position' not in door:
                raise ValueError(f"Door {i+1} is missing required 'position' parameter in rooms.yaml")
            if 'from' not in door:
                raise ValueError(f"Door {i+1} is missing required 'from' parameter in rooms.yaml")
            if 'to' not in door:
                raise ValueError(f"Door {i+1} is missing required 'to' parameter in rooms.yaml")
            
            position = door['position']
            
            # Validate parameter format
            if not isinstance(position, list) or len(position) != 2:
                raise ValueError(f"Door {i+1} position must be a list of 2 numbers [x, y]")
            
            door_x, door_y = self.world_to_map(position[0], position[1])
            
            # Store door info for visualization and sensor transparency
            door_info = {
                'position': position,
                'from': door['from'],
                'to': door['to'],
                'size': door_size * self.map_resolution,  # Convert to meters
                'thickness': wall_thickness * self.map_resolution
            }
            self.doors.append(door_info)
            
            # Create door opening - mark as transparent (50) instead of free (0)
            # This allows sensors to see through but distinguishes from walls
            x1 = max(0, door_x - door_size//2)
            x2 = min(self.map_width, door_x + door_size//2)
            y1 = max(0, door_y - wall_thickness//2)
            y2 = min(self.map_height, door_y + wall_thickness//2)
            
            self.simulation_map[y1:y2, x1:x2] = 50  # 50 = door (transparent to sensors)
    
    def add_furniture_from_config(self):
        """Add furniture to rooms based on configuration"""
        furniture_config = self.rooms_config.get('furniture', {})
        
        # Skip if no furniture configuration found
        if not furniture_config:
            self.get_logger().info('No furniture configuration found - creating environment without furniture')
            return
        
        for room_name, furniture_list in furniture_config.items():
            for i, furniture in enumerate(furniture_list):
                # Validate required parameters
                if 'position' not in furniture:
                    raise ValueError(f"Furniture {i+1} in room '{room_name}' is missing required 'position' parameter in rooms.yaml")
                if 'size' not in furniture:
                    raise ValueError(f"Furniture {i+1} in room '{room_name}' is missing required 'size' parameter in rooms.yaml")
                if 'type' not in furniture:
                    raise ValueError(f"Furniture {i+1} in room '{room_name}' is missing required 'type' parameter in rooms.yaml")
                
                position = furniture['position']
                size = furniture['size']
                
                # Validate parameter formats
                if not isinstance(position, list) or len(position) != 2:
                    raise ValueError(f"Furniture {i+1} in room '{room_name}' position must be a list of 2 numbers [x, y]")
                if not isinstance(size, list) or len(size) != 2:
                    raise ValueError(f"Furniture {i+1} in room '{room_name}' size must be a list of 2 numbers [width, height]")
                
                # Convert to map coordinates
                furn_x, furn_y = self.world_to_map(position[0], position[1])
                furn_w = int(size[0] / self.map_resolution)
                furn_h = int(size[1] / self.map_resolution)
                
                # Add furniture as occupied space
                x1 = max(0, furn_x - furn_w//2)
                x2 = min(self.map_width, furn_x + furn_w//2)
                y1 = max(0, furn_y - furn_h//2)
                y2 = min(self.map_height, furn_y + furn_h//2)
                
                self.simulation_map[y1:y2, x1:x2] = 100

    def add_room_walls(self, center_x, center_y, width, height, wall_thickness):
        """Add walls around a room"""
        x1 = max(0, center_x - width//2)
        x2 = min(self.map_width, center_x + width//2)
        y1 = max(0, center_y - height//2)
        y2 = min(self.map_height, center_y + height//2)
        
        # Top and bottom walls
        self.simulation_map[y1-wall_thickness:y1, x1:x2] = 100
        self.simulation_map[y2:y2+wall_thickness, x1:x2] = 100
        
        # Left and right walls
        self.simulation_map[y1:y2, x1-wall_thickness:x1] = 100
        self.simulation_map[y1:y2, x2:x2+wall_thickness] = 100
    
    def world_to_map(self, world_x, world_y):
        """Convert world coordinates to map pixel coordinates"""
        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y
    
    def rc_callback(self, msg):
        """Handle RC commands from exploration system"""
        with self.data_lock:
            self.rc_commands = msg
    
    def height_command_callback(self, request, response):
        """Handle takeoff/landing commands"""
        self.get_logger().info(f'Height command received: {request.command} (1=takeoff, 0=land)')
        self.get_logger().info(f'Current drone state: {self.drone_state}')
        
        if request.command == 1:  # Takeoff
            if self.drone_state == DroneState.LANDED:
                # Check if starting position is valid before takeoff
                if self.is_position_valid(self.position_x, self.position_y):
                    self.drone_state = DroneState.TAKING_OFF
                    self.get_logger().info('Simulator: Starting takeoff')
                    response.success = True
                else:
                    self.get_logger().error('Cannot takeoff: drone is in invalid position (collision detected)')
                    response.success = False
            else:
                self.get_logger().warn(f'Cannot takeoff: drone is not landed (current state: {self.drone_state})')
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
        # Calculate actual time delta for accurate physics
        current_time = self.get_clock().now()
        
        if self.last_simulation_time is None:
            # First simulation step - use nominal dt
            dt = self.SIMULATION_TIMER_INTERVAL
        else:
            # Calculate actual time difference in seconds (millisecond precision)
            dt_ms = (current_time - self.last_simulation_time).nanoseconds / 1e6  # Convert to milliseconds
            dt = dt_ms / 1000.0  # Convert to seconds
            # Clamp dt to reasonable bounds to avoid instability
            dt = max(0.001, min(dt, 0.1))  # Between 1ms and 100ms
        
        self.last_simulation_time = current_time
        
        with self.data_lock:
            if self.drone_state == DroneState.LANDED:
                return
            
            elif self.drone_state == DroneState.TAKING_OFF:
                # Simple takeoff - just increase altitude
                height_increase = self.takeoff_speed * dt
                self.position_z += height_increase
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
            
            elif self.drone_state == DroneState.FALLING:
                # Drone is falling due to collision - gravity acceleration
                fall_acceleration = 9.81  # m/s^2
                self.velocity_z -= fall_acceleration * dt
                self.position_z += self.velocity_z * dt
                
                # Check if drone hit the ground
                if self.position_z <= 0.0:
                    self.position_z = 0.0
                    self.velocity_z = 0.0
                    self.drone_state = DroneState.LANDED
                    self.get_logger().error('Drone crashed! Landing due to collision')
            
            elif self.drone_state == DroneState.FLYING:
                # Process RC commands and update position
                self.update_drone_physics(dt)
    
    def update_drone_physics(self, dt):
        """Update drone position based on RC commands"""
        # Convert RC commands (0-100) to velocities
        # RC commands: forward/backward, left/right, up/down, yaw
        
        # Forward/backward (pitch)
        forward_cmd = self.rc_commands.forward_backward_velocity / 100.0  # -1 to 1
        # Left/right (roll)
        right_cmd = self.rc_commands.left_right_velocity / 100.0     # -1 to 1
        # Up/down (throttle)
        up_cmd = self.rc_commands.up_down_velocity / 100.0    # -1 to 1
        # Yaw rotation
        yaw_cmd = self.rc_commands.yaw_velocity / 100.0  # -1 to 1
        
        # Apply deadzone
        deadzone = 0.05  # Reduced deadzone for better responsiveness
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
            # Collision detected - drone starts falling
            self.drone_state = DroneState.FALLING
            self.velocity_x = 0.0
            self.velocity_y = 0.0
            self.velocity_z = 0.0
            self.get_logger().warn('Collision detected! Drone is falling to ground')
        
        # Limit altitude
        if new_z < 0.1:
            new_z = 0.1
        elif new_z > 5.0:
            new_z = 5.0
        self.position_z = new_z
    
    def is_position_valid(self, x, y):
        """Check if position is valid (not in obstacle) - optimized version"""
        if self.simulation_map is None:
            return True
        
        # Convert world coordinates to map coordinates
        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)
        
        # Check bounds
        if map_x < 0 or map_x >= self.map_width or map_y < 0 or map_y >= self.map_height:
            return False
        
        # Check if occupied (add small safety margin)
        # Use numpy slicing for much faster checking
        safety_radius = int(0.1 / self.map_resolution)  # 10cm safety margin
        
        # Calculate slice bounds
        x_min = max(0, map_x - safety_radius)
        x_max = min(self.map_width, map_x + safety_radius + 1)
        y_min = max(0, map_y - safety_radius)
        y_max = min(self.map_height, map_y + safety_radius + 1)
        
        # Check if any cell in the safety area is occupied using numpy
        # This is much faster than nested loops
        if np.any(self.simulation_map[y_min:y_max, x_min:x_max] == 100):
            # Debug: Log collision details
            occupied_cells = np.where(self.simulation_map[y_min:y_max, x_min:x_max] == 100)
            if len(occupied_cells[0]) > 0:
                self.get_logger().warn(f'Collision detected at world pos ({x:.2f}, {y:.2f}) -> map pos ({map_x}, {map_y})')
                self.get_logger().warn(f'Safety area: x[{x_min}:{x_max}], y[{y_min}:{y_max}]')
                self.get_logger().warn(f'Found {len(occupied_cells[0])} occupied cells in safety area')
            return False
        
        return True
    
    def simulate_tof_sensors(self):
        """Simulate ToF sensors with realistic readings that respect room boundaries"""
        
        # Directional sensors (forward, left, back, right)
        # Pre-calculate angles for efficiency - ORDER MUST MATCH ToFDistances message fields
        angles = [
            self.yaw,                    # forward (0 degrees offset)
            self.yaw - math.pi/2,        # left (-90 degrees offset)
            self.yaw + math.pi,          # backward (180 degrees offset)
            self.yaw + math.pi/2         # right (+90 degrees offset)
        ]
        
        tof_distances = []
        for angle in angles:
            distance = self.raycast(self.position_x, self.position_y, angle)
            # Add some noise and measurement error
            if random.random() < self.measurement_error_prob:
                distance += random.uniform(-20, 20)  # small error instead of completely random
            else:
                distance += random.gauss(0, 1)  # reduced noise from 2cm to 1cm
            
            tof_distances.append(max(10, min(400, distance)))  # clamp to sensor range
        
        # Matrix sensor (8x8 grid) - forward facing
        # OPTIMIZATION: Use lower resolution for matrix sensor to reduce raycast calls
        # Real sensor has 8x8, but we can simulate with fewer rays and interpolate
        matrix_data = []
        base_angle = self.yaw
        fov = math.pi/6  # 60-degree FOV
        
        # Sample only key points and fill the rest (reduces from 64 to ~16 raycasts)
        sample_cols = [0, 2, 5, 7]  # Sample 4 columns instead of 8
        sample_rows = [0, 2, 5, 7]  # Sample 4 rows instead of 8
        
        # Create a sparse sampling
        samples = {}
        for row in sample_rows:
            for col in sample_cols:
                # Calculate angle for this matrix element
                angle_offset = (col - 3.5) * fov / 8  # spread across FOV
                angle = base_angle + angle_offset
                
                distance = self.raycast(self.position_x, self.position_y, angle)
                
                # Add noise
                if random.random() < self.measurement_error_prob:
                    distance += random.uniform(-20, 20)  # small error instead of completely random
                else:
                    distance += random.gauss(0, 1)  # reduced noise from 2cm to 1cm
                
                samples[(row, col)] = max(10, min(400, distance))
        
        # Fill in the full 8x8 matrix with interpolation
        for row in range(8):
            for col in range(8):
                if (row, col) in samples:
                    matrix_data.append(samples[(row, col)])
                else:
                    # Simple nearest neighbor interpolation
                    nearest_col = min(sample_cols, key=lambda c: abs(c - col))
                    nearest_row = min(sample_rows, key=lambda r: abs(r - row))
                    matrix_data.append(samples[(nearest_row, nearest_col)])
        
        distances = ToFDistances()
        distances.front = int(tof_distances[0])  # forward sensor
        distances.left = int(tof_distances[1])   # left sensor
        distances.back = int(tof_distances[2])   # backward sensor
        distances.right = int(tof_distances[3])  # right sensor
        distances.matrix = [int(x) for x in matrix_data]
        return distances
    
    def raycast(self, start_x, start_y, angle):
        """Cast a ray and return distance to first obstacle using DDA algorithm for speed
        
        Doors (value 50) are transparent to sensors - ray passes through them.
        Walls (value 100) block the ray.
        """
        if self.simulation_map is None:
            return 400.0  # Return max range if no map
        
        # Use optimized step size for balance between performance and accuracy
        # Smaller step = more accurate wall detection, but slower
        step_size_meters = self.map_resolution * 0.5  # Step by half pixel (2.5cm) for better accuracy
        max_distance_meters = 4.0  # 400cm = 4m
        max_steps = int(max_distance_meters / step_size_meters)
        
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        
        for step in range(1, max_steps):  # Start from step 1 to avoid starting position
            # Current ray position in world coordinates (meters)
            ray_x = start_x + (step * step_size_meters) * cos_angle
            ray_y = start_y + (step * step_size_meters) * sin_angle
            
            # Convert to map coordinates
            map_x = int((ray_x - self.map_origin_x) / self.map_resolution)
            map_y = int((ray_y - self.map_origin_y) / self.map_resolution)
            
            # Check bounds - if outside map, hit boundary
            if map_x < 0 or map_x >= self.map_width or map_y < 0 or map_y >= self.map_height:
                return step * step_size_meters * 100.0  # Return distance in cm
            
            # Check if hit obstacle (wall=100)
            # Doors (50) are transparent - sensors see through them
            if self.simulation_map[map_y, map_x] == 100:
                return step * step_size_meters * 100.0  # Return distance in cm
        
        return 400.0  # Max range if no obstacle found
    
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
    
    def save_environment_visualization(self):
        """Create and save a visualization of the environment with walls, furniture, and drone position"""
        try:
            # Create figure and axis
            fig, ax = plt.subplots(1, 1, figsize=(12, 12))
            
            # Set up the coordinate system to match world coordinates
            world_width = self.map_width * self.map_resolution
            world_height = self.map_height * self.map_resolution
            
            ax.set_xlim(self.map_origin_x, self.map_origin_x + world_width)
            ax.set_ylim(self.map_origin_y, self.map_origin_y + world_height)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_xlabel('X (meters)')
            ax.set_ylabel('Y (meters)')
            ax.set_title('Drone Simulator Environment')
            
            # Draw rooms from configuration
            rooms = self.rooms_config.get('rooms', {})
            for room_name, room_data in rooms.items():
                center = room_data['center']
                size = room_data['size']
                color = room_data.get('color', [0.9, 0.9, 0.9, 0.3])
                
                # Draw room area (background)
                room_rect = patches.Rectangle(
                    (center[0] - size[0]/2, center[1] - size[1]/2),
                    size[0], size[1],
                    linewidth=1, edgecolor='black', facecolor=color[:3], alpha=color[3]
                )
                ax.add_patch(room_rect)
                
                # Add room label
                ax.text(center[0], center[1], room_name.replace('_', ' ').title(), 
                       ha='center', va='center', fontsize=10, fontweight='bold')
            
            # Draw walls (from simulation map) - OPTIMIZED VERSION
            if self.simulation_map is not None:
                # Use imshow for much faster rendering instead of individual patches
                # Create a visualization array
                viz_map = np.zeros((self.map_height, self.map_width, 4))  # RGBA
                
                # Set wall pixels to black
                wall_mask = self.simulation_map == 100
                viz_map[wall_mask] = [0, 0, 0, 0.8]  # Black with alpha
                
                # Display using imshow (much faster than patches)
                extent = [self.map_origin_x, self.map_origin_x + world_width,
                         self.map_origin_y, self.map_origin_y + world_height]
                ax.imshow(viz_map, extent=extent, origin='lower', interpolation='nearest')
            
            # Draw furniture
            furniture_config = self.rooms_config.get('furniture', {})
            furniture_colors = {
                'sofa': 'brown',
                'table': 'saddlebrown', 
                'counter': 'gray',
                'bed': 'blue',
                'desk': 'darkgreen'
            }
            
            for room_name, furniture_list in furniture_config.items():
                for furniture in furniture_list:
                    position = furniture['position']
                    size = furniture['size']
                    furn_type = furniture['type']
                    color = furniture_colors.get(furn_type, 'purple')
                    
                    # Draw furniture
                    furn_rect = patches.Rectangle(
                        (position[0] - size[0]/2, position[1] - size[1]/2),
                        size[0], size[1],
                        linewidth=2, edgecolor='black', facecolor=color, alpha=0.7
                    )
                    ax.add_patch(furn_rect)
                    
                    # Add furniture label
                    ax.text(position[0], position[1], furn_type, 
                           ha='center', va='center', fontsize=8, color='white', fontweight='bold')
            
            # Draw doors
            doors = self.rooms_config.get('doors', [])
            for door in doors:
                position = door['position']
                # Draw door as a green circle
                door_circle = patches.Circle(position, 0.15, facecolor='green', edgecolor='darkgreen', linewidth=2)
                ax.add_patch(door_circle)
                ax.text(position[0], position[1]-0.3, 'DOOR', ha='center', va='center', fontsize=6, color='darkgreen')
            
            # Draw drone position
            drone_circle = patches.Circle(
                (self.position_x, self.position_y), 0.2, 
                facecolor='red', edgecolor='darkred', linewidth=3
            )
            ax.add_patch(drone_circle)
            
            # Draw drone orientation arrow
            arrow_length = 0.5
            arrow_x = self.position_x + arrow_length * math.cos(self.yaw)
            arrow_y = self.position_y + arrow_length * math.sin(self.yaw)
            ax.arrow(self.position_x, self.position_y, 
                    arrow_x - self.position_x, arrow_y - self.position_y,
                    head_width=0.1, head_length=0.1, fc='red', ec='darkred')
            
            # Add drone label
            ax.text(self.position_x, self.position_y-0.4, 'DRONE', 
                   ha='center', va='center', fontsize=8, color='darkred', fontweight='bold')
            
            # Add legend
            legend_elements = [
                patches.Patch(color='black', label='Walls'),
                patches.Patch(color='brown', label='Furniture'),
                patches.Patch(color='green', label='Doors'),
                patches.Patch(color='red', label='Drone')
            ]
            ax.legend(handles=legend_elements, loc='upper right')
            
            # Save the visualization
            output_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..', 'environment_map.png')
            output_path = os.path.abspath(output_path)
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            plt.close()
            
            self.get_logger().info(f'Environment visualization saved to: {output_path}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create environment visualization: {e}')

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
