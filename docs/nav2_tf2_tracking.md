## NAV2 TF2 position tracking and following:
  - Code logic explanation:
    - To run this tf2_follow_copilot program, you need two robots that can use tf2 broadcaster to publish their coordinate frames.
    - The tf2_follow_copilot program uses a tf2 listener to calculate the difference between the robot frames and determine the direction and distance to follow.
    - The program publishes a ROS2 Twist message to control the GoPiGo3 robot's speed and steering, so that it can follow the jetbot robot.
  - Source code:
    - [launch file: tf2_follow_copilot.launch.py](/jetbot_tools/launch/tf2_follow_copilot.launch.py)
    - [ROS2 node: tf2_listener_copilot.py](/jetbot_tools/script/tf2_listener_copilot.py)
  - Usage:
    - Pre requirements: ros2 launch <follow_copilot.launch.py> or <detect_copilot.launch.py>
    - ros2 launch jetbot_tools tf2_follow_copilot.launch.py param_file:=./jetbot_tools/param/tf2_follow_copilot_params.yaml
    - ros2 param set /tf2_follow start_follow true <br>
    <img src="/docs/TF2_04.png" width="300"/> [<img src="https://img.youtube.com/vi/jliHl-B6Ivo/hqdefault.jpg" width="300" height="200"/>](https://www.youtube.com/shorts/jliHl-B6Ivo)
