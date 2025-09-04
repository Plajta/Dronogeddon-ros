#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import json
from datetime import datetime

from std_msgs.msg import String
from geometry_msgs.msg import Point

class MotionCommandInterface(Node):
    def __init__(self):
        super().__init__('motion_command_interface')
        
        # Room locations mapping (coordinates in meters)
        self.room_locations = {
            '1': {'name': 'living_room', 'x': 0.0, 'y': 0.0},
            '2': {'name': 'kitchen', 'x': 5.0, 'y': 0.0},
            '3': {'name': 'bedroom', 'x': 0.0, 'y': 5.0},
            '4': {'name': 'bathroom', 'x': 5.0, 'y': 5.0},
            '5': {'name': 'hallway', 'x': 2.5, 'y': 2.5},
            '6': {'name': 'office', 'x': -3.0, 'y': 3.0},
            '7': {'name': 'balcony', 'x': 7.0, 'y': -2.0},
            '8': {'name': 'storage', 'x': -2.0, 'y': -3.0}
        }
        
        # Publishers
        self.motion_pub = self.create_publisher(String, 'motion_detected', 10)
        self.investigation_pub = self.create_publisher(Point, 'investigate_location', 10)
        self.status_pub = self.create_publisher(String, 'motion_command_status', 10)
        
        # Command history
        self.command_history = []
        
        self.get_logger().info('Motion Command Interface initialized')
        self.print_help()

    def process_motion_command(self, room_id):
        """Process motion detection command for specific room"""
        
        if room_id not in self.room_locations:
            self.get_logger().error(f'Unknown room ID: {room_id}. Available rooms: {list(self.room_locations.keys())}')
            self.publish_status(f'ERROR: Unknown room ID {room_id}')
            return False
        
        room_info = self.room_locations[room_id]
        room_name = room_info['name']
        
        # Log command
        timestamp = datetime.now().isoformat()
        command_entry = {
            'timestamp': timestamp,
            'room_id': room_id,
            'room_name': room_name,
            'coordinates': (room_info['x'], room_info['y'])
        }
        self.command_history.append(command_entry)
        
        # Publish motion detection event
        motion_msg = String()
        motion_msg.data = f'External motion alert: Room {room_id} ({room_name}) at {timestamp}'
        self.motion_pub.publish(motion_msg)
        
        # Publish investigation target
        target_msg = Point()
        target_msg.x = float(room_info['x'])
        target_msg.y = float(room_info['y'])
        target_msg.z = 1.5  # investigation height in meters
        self.investigation_pub.publish(target_msg)
        
        # Publish status
        status_msg = f'Motion alert sent for Room {room_id} ({room_name}) at coordinates ({room_info["x"]}, {room_info["y"]})'
        self.publish_status(status_msg)
        
        self.get_logger().info(f'Motion command processed: {status_msg}')
        
        # Save to log file
        self.save_command_log(command_entry)
        
        return True

    def publish_status(self, status):
        """Publish command status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def save_command_log(self, command_entry):
        """Save command to log file"""
        try:
            with open('motion_commands.json', 'a') as f:
                json.dump(command_entry, f)
                f.write('\n')
        except Exception as e:
            self.get_logger().error(f'Error saving command log: {e}')

    def print_help(self):
        """Print usage instructions"""
        help_text = """
=== Motion Command Interface ===

Usage: ros2 run ai motion_command_interface <room_id>

Available rooms:
"""
        for room_id, info in self.room_locations.items():
            help_text += f"  {room_id}: {info['name']} at ({info['x']}, {info['y']})\n"
        
        help_text += """
Examples:
  ros2 run ai motion_command_interface 1    # Motion in living room
  ros2 run ai motion_command_interface 3    # Motion in bedroom
  
Or use the command line interface:
  python3 motion_command_interface.py 2    # Motion in kitchen
"""
        
        print(help_text)

    def list_rooms(self):
        """List available rooms"""
        print("\nAvailable rooms:")
        for room_id, info in self.room_locations.items():
            print(f"  Room {room_id}: {info['name']} at coordinates ({info['x']}, {info['y']})")
        print()

    def show_history(self):
        """Show command history"""
        print(f"\nCommand History ({len(self.command_history)} entries):")
        for i, entry in enumerate(self.command_history[-10:], 1):  # Show last 10
            print(f"  {i}. {entry['timestamp']}: Room {entry['room_id']} ({entry['room_name']})")
        print()

def main(args=None):
    rclpy.init(args=args)
    
    interface = MotionCommandInterface()
    
    # Check if room ID provided as command line argument
    if len(sys.argv) > 1:
        room_id = sys.argv[1].strip()
        
        if room_id in ['help', '-h', '--help']:
            interface.print_help()
        elif room_id in ['list', '-l', '--list']:
            interface.list_rooms()
        elif room_id in ['history', '-hist', '--history']:
            interface.show_history()
        else:
            # Process motion command
            success = interface.process_motion_command(room_id)
            if success:
                print(f"✓ Motion alert sent for room {room_id}")
                
                # Keep node alive for a moment to ensure message is published
                rclpy.spin_once(interface, timeout_sec=1.0)
            else:
                print(f"✗ Failed to process motion command for room {room_id}")
                interface.list_rooms()
    else:
        # Interactive mode
        print("\n=== Interactive Motion Command Interface ===")
        print("Commands: <room_id>, 'list', 'history', 'help', 'quit'")
        
        try:
            while rclpy.ok():
                try:
                    user_input = input("\nEnter command: ").strip()
                    
                    if user_input.lower() in ['quit', 'exit', 'q']:
                        break
                    elif user_input.lower() in ['help', 'h']:
                        interface.print_help()
                    elif user_input.lower() in ['list', 'l']:
                        interface.list_rooms()
                    elif user_input.lower() in ['history', 'hist']:
                        interface.show_history()
                    elif user_input in interface.room_locations:
                        success = interface.process_motion_command(user_input)
                        if success:
                            print(f"✓ Motion alert sent for room {user_input}")
                        else:
                            print(f"✗ Failed to process motion command")
                    else:
                        print(f"Unknown command: {user_input}")
                        interface.list_rooms()
                    
                    # Process ROS messages
                    rclpy.spin_once(interface, timeout_sec=0.1)
                    
                except KeyboardInterrupt:
                    break
                except EOFError:
                    break
                    
        except KeyboardInterrupt:
            pass
    
    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
