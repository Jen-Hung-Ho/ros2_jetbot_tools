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

  # Launch configuration variables specific to vidio source 
  launch_video_source = LaunchConfiguration('launch_video_source')
  
  # Declare the launch arguments
  # video_source.ros2.launch arguments
  declare_launch_video_source = DeclareLaunchArgument(
   'launch_video_source', default_value='False')
  input = DeclareLaunchArgument(
    'input', default_value='csi://0')
  input_width = DeclareLaunchArgument(
    'input_width', default_value=TextSubstitution(text='640'))
  input_height = DeclareLaunchArgument(
    'input_height', default_value=TextSubstitution(text='480'))
  input_flip = DeclareLaunchArgument(
    'input_flip', default_value='rotate-180')
  # video_source.ros2.launch arguments oput topic -> /video_source/raw

  # node img_subscriber arguments
  overlay_topic = DeclareLaunchArgument(  
    'ovelay_topic', default_value='/detectnet/overlay')
  output_topic = DeclareLaunchArgument(
    'pub_topic', default_value='jebot_img')
  scale_percent = DeclareLaunchArgument(
    'scale_percent', default_value=TextSubstitution(text='50'))

  
  # Declare the detectnet arguments
  publish_topic = DeclareLaunchArgument(
    'topic', default_value='/video_source/raw')
  # remappings = [('/detectnet/image_in','/video_source/raw')]
  remappings = [('/detectnet/image_in', LaunchConfiguration('topic'))]
  model_name = DeclareLaunchArgument(
    'model_name', default_value='ssd-mobilenet-v2')
  model_path = DeclareLaunchArgument(
    'model_path', default_value='')
  class_labels_path = DeclareLaunchArgument(
    'class_labels_path', default_value='')
  input_blob = DeclareLaunchArgument(
    'input_blob', default_value='input_0')
  output_cvg = DeclareLaunchArgument(
    'output_cvg', default_value='scores')
  output_bbox = DeclareLaunchArgument(
    'output_bbox', default_value='boxes')
  threshold = DeclareLaunchArgument(
    'threshold', default_value='0.5')
  
  # Include video_source.ros2 xmal launch file
  video_source_node_cmd = IncludeLaunchDescription(
    XMLLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory("ros_deep_learning"),
        "launch/video_source.ros2.launch"
      )
    ),
    condition=IfCondition(launch_video_source)
  )

  # Launch the detectnet
  detectnet_node_cmd = Node(
      package = 'ros_deep_learning',
      executable = 'detectnet',
      name = 'detectnet',
      output = 'screen',
      emulate_tty = True,
      remappings = remappings,
      parameters = [
          {'model_name': LaunchConfiguration('model_name')},
          {'model_path': LaunchConfiguration('model_path')},
          {'class_labels_path': LaunchConfiguration('class_labels_path')},
          {'input_blob': LaunchConfiguration('input_blob')},
          {'output_cvg': LaunchConfiguration('output_cvg')},
          {'output_bbox': LaunchConfiguration('output_bbox')},
          {'threshold': LaunchConfiguration('threshold')}
        ]
    )
   
  # Publish jetbot camera image via OpenCV
  camera_publisher_cmd = Node(
      package = 'jetbot_tools',
      executable = 'img_subscriber',
      name = 'img_subscriber',
      output = 'screen',
      emulate_tty = True,
      parameters = [
          {'sub_topic': LaunchConfiguration('ovelay_topic')},
          {'pub_topic': LaunchConfiguration('pub_topic')},
          {'scale_percent': LaunchConfiguration('scale_percent')}
        ]
    )

  # Create the launch description and populate
  ld = LaunchDescription()

  # Add any actions
  ld.add_action(declare_launch_video_source)
  ld.add_action(input)
  ld.add_action(input_width)
  ld.add_action(input_height)
  ld.add_action(input_flip)
  ld.add_action(publish_topic)
  ld.add_action(overlay_topic)
  ld.add_action(output_topic)
  ld.add_action(scale_percent)

  # DetectNet configuration actions
  ld.add_action(model_name)
  ld.add_action(model_path)
  ld.add_action(class_labels_path)
  ld.add_action(input_blob)
  ld.add_action(output_cvg)
  ld.add_action(output_bbox)
  ld.add_action(threshold)

  ld.add_action(video_source_node_cmd)
  ld.add_action(detectnet_node_cmd)
  ld.add_action(camera_publisher_cmd)

  return ld