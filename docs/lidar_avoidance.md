## Lidar-assisted object avoidance self-driving:
  - Code logic explanation:
    - Use the LIDAR sensor to collect data from all directions and divide it into 12 segments of 30 degrees each
    - Compare the distances of the objects in the first three segments (front 90 degrees) and select the segment with the farthest open area
    - If the object in the selected segment is closer than a threshold distance to the target object
      - Repeat the comparison for the first six segments (front 180 degrees) and select the segment with the farthest object
      - If the object in the selected segment is still closer than the threshold distance to the target object
        - Repeat the comparison for all 12 segments (360 degrees) and select the segment with the farthest open area
          - Rotate the robot to face the selected segment
    - Publish a ROS2 Twist message to move the robot towards the open area 
  - Source code:
    - [launch file: laser_avoidance.launch.py](/jetbot_tools/launch/laser_avoidance.launch.py) <br>
    - [ROS2 node: laser_avoidance.py](/jetbot_tools/script/laser_avoidance.py) <br>
  - Usage:
    - ros2 launch jetbot_tools laser_avoidance.launch.py param_file:=./jetbot_tools/param/laser_avoidance_params.yaml
    - ros2 param get /laser_avoidance start
    - ros2 param set /laser_avoidance start true <br>
  <img src="/docs/JetBot_lidar.png" width="300"/> [<img src="https://img.youtube.com/vi/wy3AIB81d3M/hqdefault.jpg" width="300" height="200"
/>](https://www.youtube.com/shorts/wy3AIB81d3M)