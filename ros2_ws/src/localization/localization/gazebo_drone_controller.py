#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from drone_interfaces.msg import TelemetryData, RCCommand
from std_srvs.srv import Empty
import math
import numpy as np

class GazeboDroneController(Node):
    """
    Gazebo drone controller that simulates Tello behavior
    Converts RC commands to Gazebo movement and publishes telemetry
    """
    
    def __init__(self):
        super().__init__('gazebo_drone_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.telemetry_pub = self.create_publisher(TelemetryData, 'telemetry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone_pose', 10)
        
        # Subscribers
        self.rc_sub = self.create_subscription(RCCommand, 'rc_command', self.rc_callback, 10)
        
        # Services
        self.takeoff_srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(Empty, 'land', self.land_callback)
        
        # Drone state
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0  # radians
        self.is_flying = False
        self.target_height = 1.0  # meters
        
        # Control parameters
        self.max_speed = 2.0  # m/s
        self.max_yaw_rate = 1.0  # rad/s
        
        # Timer for physics update and telemetry
        self.timer = self.create_timer(0.05, self.update_physics)  # 20 Hz
        
        self.get_logger().info('Gazebo Drone Controller initialized')
    
    def takeoff_callback(self, request, response):
        """Handle takeoff service call"""
        if not self.is_flying:
            self.is_flying = True
            self.position[2] = self.target_height
            self.get_logger().info('Drone taking off')
        return response
    
    def land_callback(self, request, response):
        """Handle land service call"""
        if self.is_flying:
            self.is_flying = False
            self.position[2] = 0.0
            self.velocity = np.array([0.0, 0.0, 0.0])
            self.get_logger().info('Drone landing')
        return response
    
    def rc_callback(self, msg):
        """Process RC command and convert to velocity"""
        if not self.is_flying:
            return
        
        # Convert RC values (-100 to 100) to velocities
        # Forward/backward (pitch)
        forward_vel = (msg.pitch / 100.0) * self.max_speed
        
        # Left/right (roll) 
        right_vel = (msg.roll / 100.0) * self.max_speed
        
        # Up/down (throttle)
        up_vel = (msg.throttle / 100.0) * self.max_speed
        
        # Rotation (yaw)
        yaw_rate = (msg.yaw / 100.0) * self.max_yaw_rate
        
        # Transform to world coordinates based on current yaw
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        world_x_vel = forward_vel * cos_yaw - right_vel * sin_yaw
        world_y_vel = forward_vel * sin_yaw + right_vel * cos_yaw
        
        self.velocity[0] = world_x_vel
        self.velocity[1] = world_y_vel
        self.velocity[2] = up_vel
        
        # Update yaw
        self.yaw += yaw_rate * 0.05  # dt = 0.05s
        
        # Normalize yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi
    
    def update_physics(self):
        """Update drone physics and publish data"""
        dt = 0.05  # 20 Hz
        
        if self.is_flying:
            # Update position
            self.position += self.velocity * dt
            
            # Keep drone above ground
            if self.position[2] < 0.1:
                self.position[2] = 0.1
                self.velocity[2] = 0.0
            
            # Limit maximum height
            if self.position[2] > 5.0:
                self.position[2] = 5.0
                self.velocity[2] = 0.0
        else:
            # On ground
            self.position[2] = 0.0
            self.velocity = np.array([0.0, 0.0, 0.0])
        
        # Publish cmd_vel for Gazebo
        self.publish_cmd_vel()
        
        # Publish telemetry
        self.publish_telemetry()
        
        # Publish pose
        self.publish_pose()
    
    def publish_cmd_vel(self):
        """Publish velocity command to Gazebo"""
        cmd_vel = Twist()
        
        if self.is_flying:
            cmd_vel.linear.x = self.velocity[0]
            cmd_vel.linear.y = self.velocity[1] 
            cmd_vel.linear.z = self.velocity[2]
            cmd_vel.angular.z = self.velocity[2] * 0.1 if abs(self.velocity[2]) > 0.1 else 0.0  # Simulate yaw from RC
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def publish_telemetry(self):
        """Publish telemetry data in Tello format"""
        telemetry = TelemetryData()
        
        # Position (convert to cm like Tello)
        telemetry.x = int(self.position[0] * 100)
        telemetry.y = int(self.position[1] * 100)
        telemetry.z = int(self.position[2] * 100)
        
        # Velocity (convert to cm/s)
        telemetry.vx = int(self.velocity[0] * 100)
        telemetry.vy = int(self.velocity[1] * 100)
        telemetry.vz = int(self.velocity[2] * 100)
        
        # Yaw (convert to degrees)
        telemetry.yaw = int(math.degrees(self.yaw))
        
        # Flight status
        telemetry.flying = self.is_flying
        
        # Battery (simulate decreasing battery)
        telemetry.battery = 85  # Fixed for simulation
        
        # Temperature (simulate)
        telemetry.temperature = 25
        
        # Flight time (simulate)
        telemetry.flight_time = 120
        
        telemetry.header.stamp = self.get_clock().now().to_msg()
        telemetry.header.frame_id = "base_link"
        
        self.telemetry_pub.publish(telemetry)
    
    def publish_pose(self):
        """Publish current pose for visualization"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        
        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        
        # Convert yaw to quaternion
        cos_yaw_half = math.cos(self.yaw / 2.0)
        sin_yaw_half = math.sin(self.yaw / 2.0)
        
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = sin_yaw_half
        pose_msg.pose.orientation.w = cos_yaw_half
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    
    controller = GazeboDroneController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
