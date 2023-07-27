import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution

def generate_launch_description():

    use_viewer = LaunchConfiguration('use_viewer')

    # Declare the launch arguments
    input = DeclareLaunchArgument(
        'input', default_value='csi://0')
    input_width = DeclareLaunchArgument(
        'input_width', default_value=TextSubstitution(text='640'))
    input_height = DeclareLaunchArgument(
        'input_height', default_value=TextSubstitution(text='480'))
    input_flip = DeclareLaunchArgument(
        'input_flip', default_value='rotate-180')
    publish_topic = DeclareLaunchArgument(
        'topic', default_value='/video_source/raw')
    output_topic = DeclareLaunchArgument(
        'pub_topic', default_value='jebot_img')
    scale_percent = DeclareLaunchArgument(
        'scale_percent', default_value=TextSubstitution(text='50'))
    declare_use_viewer = DeclareLaunchArgument(
        name='use_viewer',
        default_value='False',
        description='Whether to start the viewer')

    # Launch the DNN interface video viewer
    jetbot_camera_viewer_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('jetbot_tools'), 'launch'),
         '/DNN_video_viewer.launch.py']),
      condition=IfCondition(use_viewer),
      )

    # Launch the DNN interface video source
    jetbot_image_resize_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('jetbot_tools'), 'launch'),
         '/DNN_video_source.launch.py']),
      condition=IfCondition(PythonExpression(['not ',use_viewer])),
      )
    
    ld = LaunchDescription()

    # Add any actions
    ld.add_action(input)
    ld.add_action(input_width)
    ld.add_action(input_height)
    ld.add_action(input_flip)
    ld.add_action(publish_topic)
    ld.add_action(output_topic)
    ld.add_action(scale_percent)
    ld.add_action(declare_use_viewer)

    ld.add_action(jetbot_camera_viewer_cmd)
    ld.add_action(jetbot_image_resize_cmd)

    return ld