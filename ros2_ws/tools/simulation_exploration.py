#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    localization_dir = get_package_share_directory('localization')
    
    # RViz config file path
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('localization'),
        'config',
        'drone_visualization.rviz'
    ])
    
    return LaunchDescription([
        # Drone Simulator (replaces real drone)
        Node(
            package='localization',
            executable='drone_simulator',
            name='drone_simulator',
            output='screen'
        ),
        
        # SLAM and autonomous exploration
        Node(
            package='localization',
            executable='slam_mapper',
            name='slam_mapper',
            output='screen'
        ),
        
        Node(
            package='localization',
            executable='autonomous_explorer',
            name='autonomous_explorer',
            output='screen'
        ),
        
        Node(
            package='localization',
            executable='compass_mapping',
            name='compass_mapping',
            output='screen'
        ),
        
        Node(
            package='localization',
            executable='rviz_visualizer',
            name='rviz_visualizer',
            output='screen'
        ),
        
        Node(
            package='localization',
            executable='map_saver',
            name='map_saver',
            output='screen'
        ),
        
        # RViz Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
