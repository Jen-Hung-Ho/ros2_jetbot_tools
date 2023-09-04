# jetbot_tools
Jetbot tools is a set of ROS2 nodes that uses the Jetson inference DNN vision library for NVIDIA Jetson. With Jetbot tools, you can build your own low-cost 2-wheel robot with a camera and a lidar sensor and make it do the following amazing things:

- Lidar-assisted object avoidance self-driving: Your robot can navigate autonomously and avoid obstacles using the lidar sensor.
- Real-time object detection and tracking: Your robot can detect objects using the SSD Mobilenet V2 model. You can also make your robot follow a specific object that it detects.
- Real-time object detection and distance measurement: Your robot can detect and measure the distance of objects using the SSD Mobilenet V2 model and the lidar sensor. You can also make your robot follow a specific object that it detects and stop when it is too close to the object.
- NAV2 TF2 position tracking and following: Your robot can track its own position and follow another Jetbot robot using the NAV2 TF2 framework.

#### Here is a brief overview of the jetbot tools design diagram/architecture
<img src="docs/JetBot_tool_design.png" width="700" />

### Jetbot tools video demos:
---
- **Lidar-assisted object avoidance self-driving:**
  - Source code:
    - [launch file: laser_avoidance.launch.py](launch/laser_avoidance.launch.py) <br>
    - [ROS2 node: laser_avoidance.py](jetbot_tools/script/laser_avoidance.py) <br>
  - Usage:
    - ros2 launch jetbot_tools laser_avoidance.launch.py param_file:=./jetbot_tools/param/laser_avoidance_params.yaml
    - ros2 param get /laser_avoidance start
    - ros2 param set /laser_avoidance start true <br>
  [<img src="https://img.youtube.com/vi/wy3AIB81d3M/hqdefault.jpg" width="300" height="200"
/>](https://www.youtube.com/shorts/wy3AIB81d3M)
- **Real-time object detection and tracking:**
  - Source code:
    - [launch file: detect_copilot.launch.py](launch/detect_copilot.launch.py) 
    - [ROS2 node: detect_copilot.py](jetbot_tools/script/detect_copilot.py)
  - Usage:
    - ros2 launch jetbot_tools DNN_SSD_source.launch.py model_path:=/home/jetbot/dev_ws/pytorch-ssd/models/toy/ssd-mobilenet.onnx class_labels_path:=/home/jetbot/dev_ws/pytorch-ssd/models/toy/labels.txt launch_video_source:=false topic:=/video_source/raw
    - ros2 launch jetbot_tools detect_copilot.launch.py param_file:=./jetbot_tools/param/detect_toys_copilot_params.yaml
    - ros2 param get /detect_copilot follow_detect
    - ros2 param set /detect_copilot follow_detect true <br>
  [<img src="https://img.youtube.com/vi/KeckfQseZ7E/hqdefault.jpg" width="250" height="170"
/>](https://www.youtube.com/embed/KeckfQseZ7E)
  [<img src="https://img.youtube.com/vi/qFJGvR46Qic/hqdefault.jpg" width="250" height="170"
/>](https://www.youtube.com/embed/qFJGvR46Qic)
- **Real-time object detection and distance measurement:**
  - Source code:
    - [launch file: follow_copilot.launch.py](launch/follow_copilot.launch.py)
    - [ROS2 node: follow_copilot.py](jetbot_tools/script/follow_copilot.py)
  - Usage:
    - ros2 launch jetbot_tools DNN_SSD_source.launch.py model_name:=ssd-mobilenet-v2 launch_video_source:=false topic:=/video_source/raw
    - ros2 launch jetbot_tools follow_copilot.launch.py param_file:=./jetbot_tools/param/follow_copilot_params.yaml
    - ros2 param get /follow_copilot follow_detect
    - ros2 param set /follow_copilot follow_detect false <br>
  [<img src="https://img.youtube.com/vi/tyB0vQvJUOY/hqdefault.jpg" width="300" height="200"
/>](https://www.youtube.com/embed/tyB0vQvJUOY)
- **NAV2 TF2 position tracking and following:**
  - Source code:
    - [launch file: tf2_follow_copilot.launch.py](launch/tf2_follow_copilot.launch.py)
    - [ROS2 node: tf2_listener_copilot.py](jetbot_tools/script/tf2_listener_copilot.py)
  - Usage:
    - Pre requirements: ros2 launch <follow_copilot.launch.py> or <detect_copilot.launch.py>
    - ros2 launch jetbot_tools tf2_follow_copilot.launch.py param_file:=./jetbot_tools/param/tf2_follow_copilot_params.yaml
    - ros2 param set /tf2_follow start_follow true<br>
    <img src="docs/TF2_04.png" width="300"/> [<img src="https://img.youtube.com/vi/jliHl-B6Ivo/hqdefault.jpg" width="300" height="200"/>](https://www.youtube.com/shorts/jliHl-B6Ivo)
    
### Requirements:
- Jetson Nano:
  - Ububnu 20.04: https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html
  - ROS2 foxy: https://docs.ros.org/en/foxy/index.html
  - Jetson inference and realtime vision DNN library: https://github.com/dusty-nv/jetson-inference
  - Jetson Inference Nodes for ROS2: https://github.com/dusty-nv/ros_deep_learning <br>
    <img src="docs/Ubuntu_20_04_JTOP.jpg" width="500" />
- Host Virtual Machine:
  - Ubuntu 20.04 LTS (Focal Fossa):https://www.releases.ubuntu.com/focal/ 
  - ROS2 foxy: https://docs.ros.org/en/foxy/index.html
  - NAV2 : https://navigation.ros.org/ <br>
    <p float="left">
      <img src="docs/JetBot_NAV2_2023-04-21.png" width="330" />
      <img src="docs/JetBot_DetectNet_toys.png" width="290" />
    </p>
- Robot:
  - Jetson Nano Jetbot: https://www.waveshare.com/wiki/JetBot_ROS_AI_Kit
    - https://github.com/waveshare/jetbot_pro  
  - GoPiGo3: https://www.dexterindustries.com/gopigo3/
    - https://github.com/slowrunner/ROS2-GoPiGo3
    <p float="left">
      <img src="docs/JetBot_1.jpg" width="200" height="200"/>
      <img src="docs/GoPiGo3_1.jpg" width="200" height="200"/>
    </p>
 
    
### References
- https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html
- https://docs.ros.org/en/foxy/index.html
- https://navigation.ros.org/
- https://github.com/dusty-nv/jetson-inference
- https://github.com/dusty-nv/ros_deep_learning
- https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/
