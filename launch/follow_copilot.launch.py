import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#
# ros2 launch jetbot_tools follow_copilot.launch.py param_file:=./jetbot_tools/param/person_detect_copilot_params.yaml
#

def generate_launch_description():

    param_file_cmd = DeclareLaunchArgument(
        'param_file', default_value='./jetbot_tools/param/person_follow_copilot_params.yaml')

    # Set the path to different files and folders.
    # pkg_share = FindPackageShare(package='jetbot_tools').find('jetbot_tools')
    # default_launch_dir = os.path.join(pkg_share, 'launch')
    # detect_copilot_parameter_file_path = os.path.join(pkg_share, 'param', 'detect_copilot_params.yaml')

    # Publish the joint state values for the non-fixed joints in GoPiGo3 two wheel bot.
    start_detect_copilot_node_cmd = Node(
        package='jetbot_tools',
        executable='follow_copilot',
        output="screen",
        # parameters=[detect_copilot_parameter_file_path],
        parameters=[LaunchConfiguration('param_file')],
        name='follow_copilot')
    
    ld = LaunchDescription()

    # Add any actions
    ld.add_action(param_file_cmd)
    

    ld.add_action(start_detect_copilot_node_cmd)

    return ld