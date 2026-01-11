#!/usr/bin/env python3
# Copyright (c) 2026 Davut Can Akbas
# SPDX-License-Identifier: MIT

"""
Launch file for GNSS Compass driver as a composable node.

This launch file loads the driver as a component in a component container,
allowing for zero-copy message passing with other components.

Usage:
    ros2 launch gnss_compass_driver gnss_compass_component.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
            'container_name',
            default_value='gnss_container',
            description='Name of the component container'
        ),
    ]
    
    # Parameter overrides from launch arguments
    param_overrides = {
        'port': LaunchConfiguration('port'),
        'baudrate': LaunchConfiguration('baudrate'),
        'frame_id': LaunchConfiguration('frame_id'),
    }
    
    # Component container with GNSS Compass driver
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gnss_compass_driver',
                plugin='gnss_compass::GnssCompassDriverNode',
                name='gnss_compass_driver',
                parameters=[
                    LaunchConfiguration('config_file'),
                    param_overrides,
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription(declared_arguments + [container])

