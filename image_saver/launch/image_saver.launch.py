#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明launch参数
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Image topic to subscribe to'
    )
    
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='~/turtlebot3_images',
        description='Directory to save images'
    )
    
    save_interval_arg = DeclareLaunchArgument(
        'save_interval',
        default_value='1.0',
        description='Time interval between saves (seconds)'
    )
    
    max_images_arg = DeclareLaunchArgument(
        'max_images',
        default_value='100',
        description='Maximum number of images to save'
    )
    
    # 创建image_saver节点
    image_saver_node = Node(
        package='image_saver',
        executable='image_saver_node',
        name='image_saver_node',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'save_path': LaunchConfiguration('save_path'),
            'save_interval': LaunchConfiguration('save_interval'),
            'max_images': LaunchConfiguration('max_images'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        image_topic_arg,
        save_path_arg,
        save_interval_arg,
        max_images_arg,
        image_saver_node,
    ])
