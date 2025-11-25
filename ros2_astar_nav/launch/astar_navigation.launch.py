#!/usr/bin/env python3
# Copyright 2024 The Authors
# Licensed under the Apache License, Version 2.0

"""Launch file for A* Navigation Node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for A* navigation."""
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_astar_nav'),
            'config',
            'astar_params.yaml'
        ]),
        description='Full path to the parameter file to use'
    )

    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/map',
        description='Topic for map subscription'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Topic for odometry subscription'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic for velocity command publishing'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_topic = LaunchConfiguration('map_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    # A* Navigation Node
    navigation_node = Node(
        package='ros2_astar_nav',
        executable='navigation_node',
        name='astar_navigation',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/map', map_topic),
            ('/odom', odom_topic),
            ('/cmd_vel', cmd_vel_topic)
        ]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        params_file_arg,
        map_topic_arg,
        odom_topic_arg,
        cmd_vel_topic_arg,
        # Nodes
        navigation_node
    ])
