#
# Copyright (c) 2026, Jen-Hung Ho
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

#
# ros2 launch jetbot_tools pointcloud_avoidance.launch.py param_file:=/ros2_ws/src/jetbot_tools/param/pointcloud_avoidance_params.yaml
#

def generate_launch_description():

    param_file_cmd = DeclareLaunchArgument(
        'param_file', default_value='/ros2_ws/src/jetbot_tools/param/pointcloud_avoidance_params.yaml')


    # Start the ROS2 node for the Yahboom ROSMASTER X3/Jetbot two wheel bot.
    start_pointcloud_avoidance_node_cmd = Node(
        package='jetbot_tools',
        executable='pointcloud_avoidance',
        output="screen",
        parameters=[LaunchConfiguration('param_file')],
        name='pointcloud_avoidance')

    ld = LaunchDescription()

    # Add any actions
    ld.add_action(param_file_cmd)


    ld.add_action(start_pointcloud_avoidance_node_cmd)

    return ld