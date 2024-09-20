import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

#
# ros2 launch jetbot_voice jetbot_TTS.launch.py param_file:=./jetbot_voice/param/jetbot_TTS_params.yaml
#

def generate_launch_description():

    param_file_cmd = DeclareLaunchArgument(
        'param_file', default_value='./jetbot_tools/param/jetbot_voice_copilot_params.yaml')

    # Start the ROS2 node receive Jetson ASR transcript 
    # filter with keyword list then set parameter to the target two wheel bot.
    start_jetbot_tools_voice_node_cmd = Node(
        package='jetbot_tools',
        executable='voice_copilot',
        output="screen",
        parameters=[LaunchConfiguration('param_file')]
        )

    ld = LaunchDescription()

    # Add any actions
    ld.add_action(param_file_cmd)

    ld.add_action(start_jetbot_tools_voice_node_cmd)

    return ld