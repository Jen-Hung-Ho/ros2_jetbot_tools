# jetbot_tools
Jetbot tools is a set of ROS2 nodes that uses the Jetson inference DNN vision library for NVIDIA Jetson. With Jetbot tools, you can build your own low-cost 2-wheel robot with a camera and a lidar sensor and make it do the following amazing things:

- Lidar-assisted object avoidance self-driving: Your robot can navigate autonomously and avoid obstacles using the lidar sensor.
- Real-time object detection and tracking: Your robot can detect objects using the SSD Mobilenet V2 model. You can also make your robot follow a specific object that it detects.
- Real-time object detection and distance measurement: Your robot can detect and measure the distance of objects using the SSD Mobilenet V2 model and the lidar sensor. You can also make your robot follow a specific object that it detects and stop when it is too close to the object.
- NAV2 TF2 position tracking and following: Your robot can track its own position and follow another Jetbot robot using the NAV2 TF2 framework.

#### Here is a brief overview of the jetbot tools design diagram/architecture.
![jetbot_tools](docs/JetBot_tool_design.png)

### Requirements:
- Jetson Nano:
  - Ububnu 20.04: https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html
  - ROS2 foxy: https://docs.ros.org/en/foxy/index.html
  - ![jetbot_tools](docs/Ubuntu_20_04_JTOP.jpg)
- Host Virtual Machine:
  - Ubuntu 20.04: https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html
  - NAV2 : https://navigation.ros.org/
  - ![jetbot_tools](docs/JetBot_Screenshot 2023-04-21.png)
### References
- https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html
- https://docs.ros.org/en/foxy/index.html
- https://navigation.ros.org/
- https://github.com/dusty-nv/jetson-inference
- https://github.com/dusty-nv/ros_deep_learning
- https://github.com/waveshare/jetbot_pro
- https://github.com/slowrunner/ROS2-GoPiGo3
