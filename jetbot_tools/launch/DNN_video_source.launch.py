# Author: Addison Sears-Collins
# Date: March 29, 2023
# Description: Use OpenCV to convert sensor_msgs.msg.Image to small size to publish to host machine

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Declare the launch arguments
  input = DeclareLaunchArgument(
    'input', default_value='csi://0')
  input_width = DeclareLaunchArgument(
    'input_width', default_value=TextSubstitution(text='640'))
  input_height = DeclareLaunchArgument(
    'input_height', default_value=TextSubstitution(text='360'))
  input_flip = DeclareLaunchArgument(
    'input_flip', default_value='rotate-180')
  publish_topic = DeclareLaunchArgument(
    'topic', default_value='/video_source/raw')
  output_topic = DeclareLaunchArgument(
    'pub_topic', default_value='jebot_img')
  scale_percent = DeclareLaunchArgument(
    'scale_percent', default_value=TextSubstitution(text='50'))

  # Launch condition configuration
  use_img_subscriber = LaunchConfiguration('use_img_subscriber')
  declare_use_img_subscriber = DeclareLaunchArgument(
    name='use_img_subscriber',
    default_value='True')


  
  # Include video_source.ros2 xmal launch file
  video_source_node_cmd = IncludeLaunchDescription(
    XMLLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory("ros_deep_learning"),
        "launch/video_source.ros2.launch"
      )
    )
    # launch_arguments={
    #  'input':input, 
    #  'input_width': input_width,
    #  'input_heitht': input_height,
    #  'input_flip': input_flip}
  )
   
  # Publish jetbot camera image via OpenCV
  camera_publisher_cmd = Node(
      package= 'jetbot_tools',
      executable='img_subscriber',
      name='img_subscriber',
      output = "screen",
      emulate_tty=True,
      parameters=[
          {'sub_topic': LaunchConfiguration('topic')},
          {'pub_topic': LaunchConfiguration('pub_topic')},
          {'scale_percent': LaunchConfiguration('scale_percent')}
        ],
      condition=IfCondition(use_img_subscriber)
    )

  # Create the launch description and populate
  ld = LaunchDescription()

  # Add any actions
  ld.add_action(input)
  ld.add_action(input_width)
  ld.add_action(input_height)
  ld.add_action(input_flip)
  ld.add_action(publish_topic)
  ld.add_action(output_topic)
  ld.add_action(scale_percent)
  # camera_publisher_cmd
  ld.add_action(declare_use_img_subscriber)

  ld.add_action(video_source_node_cmd)
  ld.add_action(camera_publisher_cmd)

  return ld