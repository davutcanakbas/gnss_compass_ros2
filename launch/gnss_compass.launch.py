#!/usr/bin/env python3
# Copyright (c) 2026 Davut Can Akbas
# SPDX-License-Identifier: Apache-2.0

"""
Launch file for GNSS Compass driver.

This launch file starts the GNSS driver node with configurable parameters.
The node auto-activates on startup.

Usage:
    ros2 launch gnss_compass_driver gnss_compass.launch.py
    ros2 launch gnss_compass_driver gnss_compass.launch.py port:=/dev/ttyACM0
    ros2 launch gnss_compass_driver gnss_compass.launch.py baudrate:=460800
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('gnss_compass_driver')
    
    # Default config file path
    default_config = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])
    
    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port device path'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Serial port baud rate'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='gnss_link',
            description='TF frame ID for GNSS messages'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to parameter configuration file'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Node namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
    ]
    
    # Parameter overrides from launch arguments
    param_overrides = {
        'port': LaunchConfiguration('port'),
        'baudrate': LaunchConfiguration('baudrate'),
        'frame_id': LaunchConfiguration('frame_id'),
    }
    
    # GNSS Compass Driver Node
    # Note: Node auto-configures and activates on startup (see main.cpp)
    gnss_driver_node = Node(
        package='gnss_compass_driver',
        executable='gnss_compass_driver',
        name='gnss_compass_driver',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config_file'),
            param_overrides,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription(
        declared_arguments + [
            gnss_driver_node,
        ]
    )
