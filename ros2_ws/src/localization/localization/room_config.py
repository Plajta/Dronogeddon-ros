#!/usr/bin/env python3

import yaml
import os
from pathlib import Path

class RoomConfig:
    """Centralized room configuration loader"""
    
    def __init__(self, config_file=None):
        if config_file is None:
            # Default config file location
            config_file = self._find_config_file()
        
        self.config_file = config_file
        self.rooms = {}
        self.motion_settings = {}
        self.doors = []
        self.environment = {}
        self.default_investigation_height = 1.5
        self.load_config()
    
    def _find_config_file(self):
        """Find the rooms.yaml config file"""
        # Try multiple possible locations
        possible_paths = [
            "config/rooms.yaml",
            "../config/rooms.yaml", 
            "../../config/rooms.yaml",
            "../../../config/rooms.yaml",
            os.path.join(os.path.dirname(__file__), "../../../config/rooms.yaml")
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                return path
        
        # If not found, use default path
        return "config/rooms.yaml"
    
    def load_config(self):
        """Load room configuration from YAML file"""
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            self.rooms = config.get('rooms', {})
            self.motion_settings = config.get('motion_settings', {})
            self.doors = config.get('doors', [])
            self.environment = config.get('environment', {})
            self.default_investigation_height = config.get('default_investigation_height', 1.5)
            
            print(f"Loaded room config from {self.config_file}")
            print(f"Found {len(self.rooms)} rooms: {list(self.rooms.keys())}")
            
        except FileNotFoundError:
            print(f"Warning: Room config file not found at {self.config_file}")
            print("Using default room configuration")
            self._load_default_config()
        except Exception as e:
            print(f"Error loading room config: {e}")
            print("Using default room configuration")
            self._load_default_config()
    
    def _load_default_config(self):
        """Load default room configuration if file is not found"""
        self.rooms = {
            'living_room': {
                'id': '1',
                'name': 'Living Room',
                'center': [0.0, 0.0],
                'size': [4.0, 4.0],
                'color': [0.2, 0.8, 0.2, 0.3],
                'description': 'Main living area'
            },
            'kitchen': {
                'id': '2',
                'name': 'Kitchen',
                'center': [5.0, 0.0],
                'size': [3.0, 3.0],
                'color': [0.8, 0.2, 0.2, 0.3],
                'description': 'Cooking area'
            },
            'bedroom': {
                'id': '3',
                'name': 'Bedroom',
                'center': [0.0, 5.0],
                'size': [3.5, 3.5],
                'color': [0.2, 0.2, 0.8, 0.3],
                'description': 'Main bedroom'
            }
        }
        self.motion_settings = {
            'cooldown_period': 30.0,
            'alert_duration': 300.0
        }
        self.doors = []
        self.environment = {}
    
    def get_room_by_id(self, room_id):
        """Get room configuration by ID"""
        for room_name, room_info in self.rooms.items():
            if room_info.get('id') == str(room_id):
                return room_name, room_info
        return None, None
    
    def get_room_by_name(self, room_name):
        """Get room configuration by name"""
        return self.rooms.get(room_name, None)
    
    def get_all_rooms(self):
        """Get all room configurations"""
        return self.rooms
    
    def get_room_center(self, room_name):
        """Get room center coordinates"""
        room = self.get_room_by_name(room_name)
        if room:
            return tuple(room['center'])
        return None
    
    def get_room_size(self, room_name):
        """Get room size"""
        room = self.get_room_by_name(room_name)
        if room:
            return tuple(room['size'])
        return None
    
    def get_room_color(self, room_name):
        """Get room color (RGBA)"""
        room = self.get_room_by_name(room_name)
        if room:
            return tuple(room['color'])
        return (0.5, 0.5, 0.5, 0.3)  # Default gray
    
    def get_investigation_height(self):
        """Get default investigation height"""
        return self.default_investigation_height
    
    def get_motion_settings(self):
        """Get motion detection settings"""
        return self.motion_settings
    
    def get_doors(self):
        """Get door configurations"""
        return self.doors
    
    def get_environment(self):
        """Get environment settings"""
        return self.environment
    
    def get_cooldown_period(self):
        """Get motion detection cooldown period"""
        return self.motion_settings.get('cooldown_period', 30.0)
    
    def list_room_ids(self):
        """Get list of all room IDs"""
        return [room['id'] for room in self.rooms.values()]
    
    def create_room_mapping(self):
        """Create ID to name mapping for compatibility"""
        mapping = {}
        for room_name, room_info in self.rooms.items():
            mapping[room_info['id']] = room_name
        return mapping
    
    def reload_config(self):
        """Reload configuration from file"""
        self.load_config()

# Global instance for easy access
_room_config_instance = None

def get_room_config():
    """Get global room configuration instance"""
    global _room_config_instance
    if _room_config_instance is None:
        _room_config_instance = RoomConfig()
    return _room_config_instance

def reload_room_config():
    """Reload room configuration"""
    global _room_config_instance
    if _room_config_instance is not None:
        _room_config_instance.reload_config()
    else:
        _room_config_instance = RoomConfig()
    return _room_config_instance
