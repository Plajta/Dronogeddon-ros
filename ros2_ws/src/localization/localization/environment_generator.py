#!/usr/bin/env python3

import numpy as np
import cv2
import json
import os
import argparse
from datetime import datetime

class EnvironmentGenerator:
    """Tool for creating simulation environments from room configurations or custom layouts"""
    
    def __init__(self, resolution=0.05, width=400, height=400):
        self.resolution = resolution  # meters per pixel
        self.width = width           # pixels
        self.height = height         # pixels
        self.origin_x = -10.0        # world coordinates of map origin
        self.origin_y = -10.0
        
    def create_apartment_layout(self, rooms_config=None):
        """Create apartment layout from room configuration"""
        # Initialize empty map (all free space)
        env_map = np.zeros((self.height, self.width), dtype=np.uint8)
        
        if rooms_config is None:
            # Create default apartment
            return self.create_default_apartment()
        
        # Load rooms from configuration
        if isinstance(rooms_config, str):
            with open(rooms_config, 'r') as f:
                import yaml
                config = yaml.safe_load(f)
                rooms = config.get('rooms', {})
        else:
            rooms = rooms_config
        
        # Add outer walls
        wall_thickness = 3
        env_map[0:wall_thickness, :] = 100      # Top wall
        env_map[-wall_thickness:, :] = 100     # Bottom wall
        env_map[:, 0:wall_thickness] = 100     # Left wall
        env_map[:, -wall_thickness:] = 100     # Right wall
        
        # Generate walls between rooms
        self.add_room_walls(env_map, rooms)
        
        # Add furniture and obstacles
        self.add_furniture(env_map, rooms)
        
        return env_map
    
    def create_default_apartment(self):
        """Create default apartment with 4 rooms"""
        env_map = np.zeros((self.height, self.width), dtype=np.uint8)
        
        # Outer walls
        wall_thickness = 5
        env_map[0:wall_thickness, :] = 100
        env_map[-wall_thickness:, :] = 100
        env_map[:, 0:wall_thickness] = 100
        env_map[:, -wall_thickness:] = 100
        
        # Internal walls
        # Vertical divider
        env_map[50:350, 195:205] = 100
        # Horizontal divider
        env_map[195:205, 50:350] = 100
        
        # Doors (gaps in walls)
        door_width = 20
        env_map[200-door_width//2:200+door_width//2, 195:205] = 0  # Vertical wall door
        env_map[195:205, 200-door_width//2:200+door_width//2] = 0  # Horizontal wall door
        env_map[100:140, 195:205] = 0  # Additional door
        
        # Add furniture
        # Living room (bottom-left)
        env_map[80:120, 80:140] = 100   # Sofa
        env_map[150:170, 120:160] = 100 # Coffee table
        
        # Kitchen (bottom-right)
        env_map[80:100, 250:350] = 100  # Kitchen counter
        env_map[120:140, 280:300] = 100 # Island
        
        # Bedroom (top-left)
        env_map[250:300, 80:150] = 100  # Bed
        env_map[320:340, 120:140] = 100 # Dresser
        
        # Bathroom (top-right)
        env_map[250:280, 250:280] = 100 # Toilet
        env_map[290:320, 280:320] = 100 # Bathtub
        
        return env_map
    
    def add_room_walls(self, env_map, rooms):
        """Add walls between rooms based on configuration"""
        # Simple wall generation - create walls around room boundaries
        wall_thickness = 3
        
        for room_name, room_info in rooms.items():
            center = room_info['center']
            size = room_info['size']
            
            # Convert to map coordinates
            map_x = int((center[0] - self.origin_x) / self.resolution)
            map_y = int((center[1] - self.origin_y) / self.resolution)
            map_w = int(size[0] / self.resolution)
            map_h = int(size[1] / self.resolution)
            
            # Room boundaries
            x1 = max(0, map_x - map_w//2)
            x2 = min(self.width, map_x + map_w//2)
            y1 = max(0, map_y - map_h//2)
            y2 = min(self.height, map_y + map_h//2)
            
            # Add walls around room (but not completely enclosed)
            # Top and bottom walls
            if y1 > wall_thickness:
                env_map[y1-wall_thickness:y1, x1:x2] = 100
            if y2 < self.height - wall_thickness:
                env_map[y2:y2+wall_thickness, x1:x2] = 100
            
            # Left and right walls (with doors)
            door_size = map_h // 4  # Door is 1/4 of room height
            door_start = (map_h - door_size) // 2
            
            if x1 > wall_thickness:
                env_map[y1:y1+door_start, x1-wall_thickness:x1] = 100
                env_map[y1+door_start+door_size:y2, x1-wall_thickness:x1] = 100
            if x2 < self.width - wall_thickness:
                env_map[y1:y1+door_start, x2:x2+wall_thickness] = 100
                env_map[y1+door_start+door_size:y2, x2:x2+wall_thickness] = 100
    
    def add_furniture(self, env_map, rooms):
        """Add furniture to rooms"""
        furniture_configs = {
            'living_room': [
                {'type': 'sofa', 'size': (1.5, 0.8), 'offset': (-1.0, -0.5)},
                {'type': 'table', 'size': (0.8, 0.8), 'offset': (0.5, 0.0)},
            ],
            'kitchen': [
                {'type': 'counter', 'size': (2.0, 0.6), 'offset': (0.0, -0.8)},
                {'type': 'island', 'size': (1.0, 0.6), 'offset': (0.0, 0.5)},
            ],
            'bedroom': [
                {'type': 'bed', 'size': (2.0, 1.4), 'offset': (0.0, -0.3)},
                {'type': 'dresser', 'size': (1.2, 0.5), 'offset': (0.8, 0.8)},
            ],
            'bathroom': [
                {'type': 'toilet', 'size': (0.6, 0.8), 'offset': (-0.5, -0.5)},
                {'type': 'sink', 'size': (0.8, 0.5), 'offset': (0.5, -0.5)},
            ]
        }
        
        for room_name, room_info in rooms.items():
            room_key = room_name.lower()
            if room_key not in furniture_configs:
                continue
            
            center = room_info['center']
            furniture_list = furniture_configs[room_key]
            
            for furniture in furniture_list:
                self.add_furniture_item(env_map, center, furniture)
    
    def add_furniture_item(self, env_map, room_center, furniture):
        """Add single furniture item to map"""
        # Calculate furniture position
        furn_x = room_center[0] + furniture['offset'][0]
        furn_y = room_center[1] + furniture['offset'][1]
        furn_w, furn_h = furniture['size']
        
        # Convert to map coordinates
        map_x = int((furn_x - self.origin_x) / self.resolution)
        map_y = int((furn_y - self.origin_y) / self.resolution)
        map_w = int(furn_w / self.resolution)
        map_h = int(furn_h / self.resolution)
        
        # Add furniture to map
        x1 = max(0, map_x - map_w//2)
        x2 = min(self.width, map_x + map_w//2)
        y1 = max(0, map_y - map_h//2)
        y2 = min(self.height, map_y + map_h//2)
        
        env_map[y1:y2, x1:x2] = 100
    
    def save_environment(self, env_map, filename_base):
        """Save environment in multiple formats"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create simulation_environments directory
        sim_dir = "simulation_environments"
        if not os.path.exists(sim_dir):
            os.makedirs(sim_dir)
        
        # Save as PNG (for visualization and loading)
        img = np.zeros_like(env_map)
        img[env_map == 0] = 255    # Free space = white
        img[env_map == 100] = 0    # Occupied = black
        img = np.flipud(img)       # Flip for correct orientation
        
        png_path = os.path.join(sim_dir, f"{filename_base}_{timestamp}.png")
        cv2.imwrite(png_path, img)
        
        # Save metadata
        metadata = {
            'filename': f"{filename_base}_{timestamp}",
            'resolution': self.resolution,
            'width': self.width,
            'height': self.height,
            'origin_x': self.origin_x,
            'origin_y': self.origin_y,
            'created': datetime.now().isoformat(),
            'type': 'simulation_environment'
        }
        
        metadata_path = os.path.join(sim_dir, f"{filename_base}_{timestamp}_metadata.json")
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        # Save raw data (for exact loading)
        raw_data = {
            'map_data': env_map.tolist(),
            'metadata': metadata
        }
        
        raw_path = os.path.join(sim_dir, f"{filename_base}_{timestamp}_raw.json")
        with open(raw_path, 'w') as f:
            json.dump(raw_data, f)
        
        print(f"Environment saved:")
        print(f"  Image: {png_path}")
        print(f"  Metadata: {metadata_path}")
        print(f"  Raw data: {raw_path}")
        
        return png_path, metadata_path, raw_path
    
    def create_maze_environment(self, complexity=0.3):
        """Create maze-like environment for testing"""
        env_map = np.zeros((self.height, self.width), dtype=np.uint8)
        
        # Outer walls
        env_map[0:5, :] = 100
        env_map[-5:, :] = 100
        env_map[:, 0:5] = 100
        env_map[:, -5:] = 100
        
        # Add random walls
        np.random.seed(42)  # For reproducible results
        
        # Vertical walls
        for i in range(10, self.width-10, 30):
            wall_height = int(self.height * complexity)
            start_y = np.random.randint(10, self.height - wall_height - 10)
            env_map[start_y:start_y+wall_height, i:i+3] = 100
            
            # Add gaps
            gap_size = 20
            gap_pos = start_y + wall_height//2
            env_map[gap_pos:gap_pos+gap_size, i:i+3] = 0
        
        # Horizontal walls
        for i in range(10, self.height-10, 30):
            wall_width = int(self.width * complexity)
            start_x = np.random.randint(10, self.width - wall_width - 10)
            env_map[i:i+3, start_x:start_x+wall_width] = 100
            
            # Add gaps
            gap_size = 20
            gap_pos = start_x + wall_width//2
            env_map[i:i+3, gap_pos:gap_pos+gap_size] = 0
        
        return env_map

def main():
    parser = argparse.ArgumentParser(description='Generate simulation environments')
    parser.add_argument('--type', choices=['apartment', 'maze', 'from_config'], 
                       default='apartment', help='Type of environment to create')
    parser.add_argument('--config', help='Room configuration file (for from_config type)')
    parser.add_argument('--output', default='sim_environment', help='Output filename base')
    parser.add_argument('--resolution', type=float, default=0.05, help='Map resolution (m/pixel)')
    parser.add_argument('--width', type=int, default=400, help='Map width (pixels)')
    parser.add_argument('--height', type=int, default=400, help='Map height (pixels)')
    
    args = parser.parse_args()
    
    generator = EnvironmentGenerator(args.resolution, args.width, args.height)
    
    if args.type == 'apartment':
        env_map = generator.create_default_apartment()
        print("Created default apartment environment")
    
    elif args.type == 'maze':
        env_map = generator.create_maze_environment()
        print("Created maze environment")
    
    elif args.type == 'from_config':
        if not args.config:
            print("Error: --config required for from_config type")
            return
        
        if not os.path.exists(args.config):
            print(f"Error: Config file not found: {args.config}")
            return
        
        env_map = generator.create_apartment_layout(args.config)
        print(f"Created environment from config: {args.config}")
    
    else:
        print(f"Unknown environment type: {args.type}")
        return
    
    # Save environment
    generator.save_environment(env_map, args.output)
    
    # Show statistics
    total_pixels = args.width * args.height
    occupied_pixels = np.sum(env_map == 100)
    free_pixels = np.sum(env_map == 0)
    
    print(f"\nEnvironment statistics:")
    print(f"  Total area: {total_pixels * args.resolution * args.resolution:.1f} m²")
    print(f"  Free space: {free_pixels * args.resolution * args.resolution:.1f} m² ({free_pixels/total_pixels*100:.1f}%)")
    print(f"  Occupied space: {occupied_pixels * args.resolution * args.resolution:.1f} m² ({occupied_pixels/total_pixels*100:.1f}%)")

if __name__ == '__main__':
    main()
