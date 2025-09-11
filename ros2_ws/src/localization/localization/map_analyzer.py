#!/usr/bin/env python3

import json
import numpy as np
import cv2
import os
import sys
from datetime import datetime
import argparse

class MapAnalyzer:
    """Tool for analyzing saved exploration maps and generating room configuration hints"""
    
    def __init__(self, maps_directory="saved_maps"):
        self.maps_directory = maps_directory
        
    def list_available_maps(self):
        """List all available saved maps"""
        if not os.path.exists(self.maps_directory):
            print(f"Maps directory '{self.maps_directory}' not found")
            return []
        
        map_files = []
        for file in os.listdir(self.maps_directory):
            if file.endswith('_analysis.json'):
                map_files.append(file.replace('_analysis.json', ''))
        
        return sorted(map_files)
    
    def analyze_map(self, map_name):
        """Analyze a specific map and generate room configuration"""
        analysis_file = os.path.join(self.maps_directory, f"{map_name}_analysis.json")
        metadata_file = os.path.join(self.maps_directory, f"{map_name}_metadata.json")
        image_file = os.path.join(self.maps_directory, f"{map_name}.png")
        
        if not os.path.exists(analysis_file):
            print(f"Analysis file not found: {analysis_file}")
            return None
        
        # Load analysis data
        with open(analysis_file, 'r') as f:
            analysis = json.load(f)
        
        # Load metadata if available
        metadata = {}
        if os.path.exists(metadata_file):
            with open(metadata_file, 'r') as f:
                metadata = json.load(f)
        
        print(f"\n=== Map Analysis: {map_name} ===")
        print(f"Session ID: {analysis.get('session_id', 'Unknown')}")
        print(f"Exploration Duration: {analysis.get('exploration_duration_seconds', 0):.1f} seconds")
        print(f"Map Updates: {analysis.get('map_updates_count', 0)}")
        print(f"Trajectory Points: {analysis.get('trajectory_points', 0)}")
        print(f"Explored Area: {analysis.get('map_area_explored', 0):.2f} m²")
        
        efficiency = analysis.get('exploration_efficiency', {})
        if efficiency:
            print(f"Total Distance: {efficiency.get('total_distance_meters', 0):.2f} m")
            print(f"Average Speed: {efficiency.get('average_speed', 0):.2f} m/s")
            print(f"Area per Distance: {efficiency.get('area_per_distance', 0):.2f} m²/m")
        
        # Display room detection hints
        hints = analysis.get('room_detection_hints', [])
        if hints:
            print(f"\n=== Room Detection Hints ({len(hints)} regions found) ===")
            for i, hint in enumerate(hints, 1):
                print(f"\nRegion {i}:")
                print(f"  Center: ({hint['center_x']}, {hint['center_y']})")
                print(f"  Size: {hint['width']} x {hint['height']} m")
                print(f"  Area: {hint['area_sqm']} m²")
        
        return analysis
    
    def generate_room_config(self, map_name, room_names=None):
        """Generate room configuration YAML from map analysis"""
        analysis_file = os.path.join(self.maps_directory, f"{map_name}_analysis.json")
        
        if not os.path.exists(analysis_file):
            print(f"Analysis file not found: {analysis_file}")
            return None
        
        with open(analysis_file, 'r') as f:
            analysis = json.load(f)
        
        hints = analysis.get('room_detection_hints', [])
        if not hints:
            print("No room detection hints found in analysis")
            return None
        
        # Generate room configuration
        config = {
            'rooms': {},
            'default_investigation_height': 1.5,
            'motion_settings': {
                'cooldown_period': 30.0,
                'alert_duration': 300.0
            }
        }
        
        # Default room names if not provided
        if room_names is None:
            room_names = [
                'living_room', 'kitchen', 'bedroom', 'bathroom', 
                'hallway', 'office', 'balcony', 'storage'
            ]
        
        # Default colors for rooms
        colors = [
            [0.2, 0.8, 0.2, 0.3],  # Green
            [0.8, 0.2, 0.2, 0.3],  # Red
            [0.2, 0.2, 0.8, 0.3],  # Blue
            [0.8, 0.8, 0.2, 0.3],  # Yellow
            [0.8, 0.2, 0.8, 0.3],  # Magenta
            [0.2, 0.8, 0.8, 0.3],  # Cyan
            [0.5, 0.8, 0.2, 0.3],  # Light Green
            [0.6, 0.4, 0.2, 0.3],  # Brown
        ]
        
        for i, hint in enumerate(hints):
            if i >= len(room_names):
                break
                
            room_key = room_names[i]
            room_name = room_names[i].replace('_', ' ').title()
            
            config['rooms'][room_key] = {
                'id': str(i + 1),
                'name': room_name,
                'center': [hint['center_x'], hint['center_y']],
                'size': [hint['width'], hint['height']],
                'color': colors[i % len(colors)],
                'description': f"Auto-detected room {i + 1}",
                'area_sqm': hint['area_sqm']
            }
        
        return config
    
    def save_room_config(self, map_name, output_file=None, room_names=None):
        """Save generated room configuration to YAML file"""
        config = self.generate_room_config(map_name, room_names)
        if config is None:
            return False
        
        if output_file is None:
            output_file = f"rooms_from_{map_name}.yaml"
        
        # Convert to YAML format manually (since we want specific formatting)
        yaml_content = "# Room configuration generated from exploration map\n"
        yaml_content += f"# Source map: {map_name}\n"
        yaml_content += f"# Generated: {datetime.now().isoformat()}\n\n"
        
        yaml_content += "rooms:\n"
        for room_key, room_data in config['rooms'].items():
            yaml_content += f"  {room_key}:\n"
            yaml_content += f"    id: \"{room_data['id']}\"\n"
            yaml_content += f"    name: \"{room_data['name']}\"\n"
            yaml_content += f"    center: {room_data['center']}\n"
            yaml_content += f"    size: {room_data['size']}\n"
            yaml_content += f"    color: {room_data['color']}\n"
            yaml_content += f"    description: \"{room_data['description']}\"\n"
            yaml_content += f"    # Detected area: {room_data['area_sqm']:.2f} m²\n\n"
        
        yaml_content += f"# Default investigation height for all rooms\n"
        yaml_content += f"default_investigation_height: {config['default_investigation_height']}\n\n"
        
        yaml_content += "# Motion detection settings\n"
        yaml_content += "motion_settings:\n"
        yaml_content += f"  cooldown_period: {config['motion_settings']['cooldown_period']}\n"
        yaml_content += f"  alert_duration: {config['motion_settings']['alert_duration']}\n"
        
        try:
            with open(output_file, 'w') as f:
                f.write(yaml_content)
            print(f"Room configuration saved to: {output_file}")
            return True
        except Exception as e:
            print(f"Error saving configuration: {e}")
            return False
    
    def interactive_room_config(self, map_name):
        """Interactive room configuration generator"""
        analysis = self.analyze_map(map_name)
        if analysis is None:
            return
        
        hints = analysis.get('room_detection_hints', [])
        if not hints:
            print("No room regions detected in this map")
            return
        
        print(f"\n=== Interactive Room Configuration ===")
        print(f"Found {len(hints)} potential room regions")
        print("For each region, specify a room name or press Enter to skip")
        
        room_assignments = {}
        room_id = 1
        
        for i, hint in enumerate(hints):
            print(f"\nRegion {i + 1}:")
            print(f"  Center: ({hint['center_x']:.2f}, {hint['center_y']:.2f})")
            print(f"  Size: {hint['width']:.2f} x {hint['height']:.2f} m")
            print(f"  Area: {hint['area_sqm']:.2f} m²")
            
            room_name = input(f"Room name for region {i + 1} (or Enter to skip): ").strip()
            
            if room_name:
                room_key = room_name.lower().replace(' ', '_')
                room_assignments[room_key] = {
                    'id': str(room_id),
                    'name': room_name,
                    'center': [hint['center_x'], hint['center_y']],
                    'size': [hint['width'], hint['height']],
                    'area_sqm': hint['area_sqm']
                }
                room_id += 1
                print(f"  → Assigned as '{room_name}'")
        
        if room_assignments:
            output_file = input(f"\nSave configuration to file (default: rooms_from_{map_name}.yaml): ").strip()
            if not output_file:
                output_file = f"rooms_from_{map_name}.yaml"
            
            # Create full config
            config = {
                'rooms': room_assignments,
                'default_investigation_height': 1.5,
                'motion_settings': {
                    'cooldown_period': 30.0,
                    'alert_duration': 300.0
                }
            }
            
            # Save with assigned names
            self._save_config_with_assignments(config, output_file, map_name)
        else:
            print("No rooms assigned")
    
    def _save_config_with_assignments(self, config, output_file, map_name):
        """Save configuration with user assignments"""
        colors = [
            [0.2, 0.8, 0.2, 0.3],  # Green
            [0.8, 0.2, 0.2, 0.3],  # Red
            [0.2, 0.2, 0.8, 0.3],  # Blue
            [0.8, 0.8, 0.2, 0.3],  # Yellow
            [0.8, 0.2, 0.8, 0.3],  # Magenta
            [0.2, 0.8, 0.8, 0.3],  # Cyan
            [0.5, 0.8, 0.2, 0.3],  # Light Green
            [0.6, 0.4, 0.2, 0.3],  # Brown
        ]
        
        yaml_content = "# Room configuration generated from exploration map\n"
        yaml_content += f"# Source map: {map_name}\n"
        yaml_content += f"# Generated: {datetime.now().isoformat()}\n\n"
        
        yaml_content += "rooms:\n"
        for i, (room_key, room_data) in enumerate(config['rooms'].items()):
            color = colors[i % len(colors)]
            yaml_content += f"  {room_key}:\n"
            yaml_content += f"    id: \"{room_data['id']}\"\n"
            yaml_content += f"    name: \"{room_data['name']}\"\n"
            yaml_content += f"    center: {room_data['center']}\n"
            yaml_content += f"    size: {room_data['size']}\n"
            yaml_content += f"    color: {color}\n"
            yaml_content += f"    description: \"User-assigned room from exploration\"\n"
            yaml_content += f"    # Detected area: {room_data['area_sqm']:.2f} m²\n\n"
        
        yaml_content += f"# Default investigation height for all rooms\n"
        yaml_content += f"default_investigation_height: {config['default_investigation_height']}\n\n"
        
        yaml_content += "# Motion detection settings\n"
        yaml_content += "motion_settings:\n"
        yaml_content += f"  cooldown_period: {config['motion_settings']['cooldown_period']}\n"
        yaml_content += f"  alert_duration: {config['motion_settings']['alert_duration']}\n"
        
        try:
            with open(output_file, 'w') as f:
                f.write(yaml_content)
            print(f"Room configuration saved to: {output_file}")
            return True
        except Exception as e:
            print(f"Error saving configuration: {e}")
            return False

def main():
    parser = argparse.ArgumentParser(description='Analyze exploration maps and generate room configurations')
    parser.add_argument('--maps-dir', default='saved_maps', help='Directory containing saved maps')
    parser.add_argument('--list', action='store_true', help='List available maps')
    parser.add_argument('--analyze', help='Analyze specific map')
    parser.add_argument('--generate', help='Generate room config from map')
    parser.add_argument('--interactive', help='Interactive room configuration')
    parser.add_argument('--output', help='Output file for generated configuration')
    
    args = parser.parse_args()
    
    analyzer = MapAnalyzer(args.maps_dir)
    
    if args.list:
        maps = analyzer.list_available_maps()
        if maps:
            print("Available maps:")
            for map_name in maps:
                print(f"  - {map_name}")
        else:
            print("No maps found")
    
    elif args.analyze:
        analyzer.analyze_map(args.analyze)
    
    elif args.generate:
        analyzer.save_room_config(args.generate, args.output)
    
    elif args.interactive:
        analyzer.interactive_room_config(args.interactive)
    
    else:
        # Interactive mode
        maps = analyzer.list_available_maps()
        if not maps:
            print("No maps found in directory:", args.maps_dir)
            return
        
        print("Available maps:")
        for i, map_name in enumerate(maps, 1):
            print(f"  {i}. {map_name}")
        
        try:
            choice = input("\nSelect map number for analysis (or 'q' to quit): ").strip()
            if choice.lower() == 'q':
                return
            
            map_idx = int(choice) - 1
            if 0 <= map_idx < len(maps):
                selected_map = maps[map_idx]
                
                print(f"\nSelected: {selected_map}")
                print("Options:")
                print("  1. Analyze map")
                print("  2. Generate room config (auto)")
                print("  3. Interactive room assignment")
                
                action = input("Choose action (1-3): ").strip()
                
                if action == '1':
                    analyzer.analyze_map(selected_map)
                elif action == '2':
                    analyzer.save_room_config(selected_map)
                elif action == '3':
                    analyzer.interactive_room_config(selected_map)
                else:
                    print("Invalid choice")
            else:
                print("Invalid map number")
                
        except (ValueError, KeyboardInterrupt):
            print("\nExiting...")

if __name__ == '__main__':
    main()
