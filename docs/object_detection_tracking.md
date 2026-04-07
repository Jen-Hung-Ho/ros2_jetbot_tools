## Real-time object detection and tracking:
  - Code logic explanation:
    - Use Jetson DNN inference ROS2 detectnet node to detect the targeting object position of the image capture from camera
      - https://github.com/dusty-nv/ros_deep_learning#detectnet-node-1
    - Calculate the angle between the image center and the targeting position
    - Use the size of the detected image to determine the distance between robot to the target
    - Send a ROS2 Twist message to move the robot follow the detection object
    - Stop the robot if it is too close to the target
  - Source code:
    - [launch file: detect_copilot.launch.py](/jetbot_tools/launch/detect_copilot.launch.py) 
    - [ROS2 node: detect_copilot.py](/jetbot_tools/script/detect_copilot.py)
  - Usage:
    - ros2 launch jetbot_tools DNN_SSD_source.launch.py model_path:=/home/jetbot/dev_ws/pytorch-ssd/models/toy/ssd-mobilenet.onnx class_labels_path:=/home/jetbot/dev_ws/pytorch-ssd/models/toy/labels.txt launch_video_source:=false topic:=/video_source/raw
    - ros2 launch jetbot_tools detect_copilot.launch.py param_file:=./jetbot_tools/param/detect_toys_copilot_params.yaml
    - ros2 param get /detect_copilot follow_detect
    - ros2 param set /detect_copilot follow_detect true <br>
  [<img src="https://img.youtube.com/vi/KeckfQseZ7E/hqdefault.jpg" width="250" height="170"
/>](https://www.youtube.com/embed/KeckfQseZ7E)
  [<img src="https://img.youtube.com/vi/qFJGvR46Qic/hqdefault.jpg" width="250" height="170"
/>](https://www.youtube.com/embed/qFJGvR46Qic)