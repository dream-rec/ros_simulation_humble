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
    
    show_image_arg = DeclareLaunchArgument(
        'show_image',
        default_value='true',
        description='Whether to show the camera feed window'
    )
    
    # 创建interactive image saver节点
    interactive_saver_node = Node(
        package='image_saver',
        executable='interactive_image_saver',
        name='interactive_image_saver',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'save_path': LaunchConfiguration('save_path'),
            'show_image': LaunchConfiguration('show_image'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        image_topic_arg,
        save_path_arg,
        show_image_arg,
        interactive_saver_node,
    ])
