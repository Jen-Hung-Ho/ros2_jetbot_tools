# Author: Addison Sears-Collins
# Date: March 29, 2023
# Description: Use OpenCV to convert sensor_msgs.msg.Image to small size to publish to host machine

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Publish jetbot camera image via OpenCV
    camera_publisher_cmd = Node(
        package= 'jetbot_tools',
        executable='img_subscriber',
        name='img_subscriber',
        output = "screen",
        emulate_tty=True,
        parameters=[
            {'sub_topic': 'video_source/raw'},
            {'pub_topic': 'jetbot_img'},
            {'scale_percent': 50}
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any actions
    ld.add_action(camera_publisher_cmd)

    return ld