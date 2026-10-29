#!/usr/bin/env python3
# -*-coding:utf8-*-
# Launch file for the GraspNet to PosCmd converter node

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for graspnet_to_poscmd node"""
    
    graspnet_to_poscmd_node = Node(
        package='piper',
        executable='graspnet_to_poscmd',
        name='graspnet_to_poscmd_node',
        output='screen',
        parameters=[],
    )
    
    return LaunchDescription([
        graspnet_to_poscmd_node,
    ])

