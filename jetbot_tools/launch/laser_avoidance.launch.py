import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

#
# ros2 launch jetbot_tools laser_avoidance.launch.py param_file:=./jetbot_tools/param/laser_avoidance_params.yaml
#

def generate_launch_description():

    param_file_cmd = DeclareLaunchArgument(
        'param_file', default_value='./jetbot_tools/param/laser_avoidance_params.yaml')


    # Start the ROS2 node for the GoPiGo3/Jetbot two wheel bot.
    start_laser_avoidance_node_cmd = Node(
        package='jetbot_tools',
        executable='laser_avoidance',
        output="screen",
        parameters=[LaunchConfiguration('param_file')],
        name='laser_avoidance')
    
    ld = LaunchDescription()

    # Add any actions
    ld.add_action(param_file_cmd)
    

    ld.add_action(start_laser_avoidance_node_cmd)

    return ld