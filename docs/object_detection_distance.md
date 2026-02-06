## Real-time object detection and distance measurement (NEW in v2.1):
  - Code logic explanation:
    - Detection Modes (V1 & V2):
      - V1 (Legacy): Uses Jetson DNN inference ROS2 detectnet node to detect the target object position from camera images, then fuses with LIDAR 360° LaserScan data to calculate distance
        - https://github.com/dusty-nv/ros_deep_learning#detectnet-node-1
      - V2 (Current): Leverages YOLOv11n via jetbot_vision_perception. The YOLOv11n detection mode identifies the target object position and estimates distance directly using a depth camera, enabling precise real-time tracking and safe following
        - [Ultralytics YOLO tutorial](https://www.jetson-ai-lab.com/tutorial_ultralytics.html)
        - [Jetbot vesion perception](https://github.com/Jen-Hung-Ho/jetbot_vision_perception)
    - The LIDAR sensor collected 360-degree ROS2 LaserScan raw data
      - http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
    - Calculate the rotation angle by measuring the difference between the camera’s field of view (FOV) and the detection image position
    - Use lidar data on rotation angle to calculate the distance of an object
    - Send a ROS2 Twist message to move the robot follow the detection object
    - Stop the robot if it is too close to the target
  - Source code:
    - [launch file: follow_copilot.launch.py](/jetbot_tools/launch/follow_copilot.launch.py)
    - [ROS2 node: follow_copilot.py](/jetbot_tools/script/follow_copilot.py)
  - Usage:
    - V1 (DetectNet + LIDAR)
      - ros2 launch jetbot_tools DNN_SSD_source.launch.py model_name:=ssd-mobilenet-v2 launch_video_source:=false topic:=/video_source/raw
      - ros2 launch jetbot_tools follow_copilot.launch.py param_file:=./jetbot_tools/param/follow_copilot_params.yaml
    - V2 (YOLOv11n + Depth Camera)
      - ros2 launch jetbot_tools follow_copilot.launch.py param_file:=./jetbot_tools/param/follow_copilot_params_v2.yaml
    - ros2 param get /follow_copilot follow_detect
    - ros2 param set /follow_copilot follow_detect true <br>
  [<img src="https://img.youtube.com/vi/tyB0vQvJUOY/hqdefault.jpg" width="300" height="200"
/>](https://www.youtube.com/embed/tyB0vQvJUOY)