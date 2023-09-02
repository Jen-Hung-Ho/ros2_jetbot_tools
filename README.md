# jetbot_tools
Jetbot Tools is a set up ROS2 nodes powered by Jetson Nano, to perform real-time object detection and distance estimation. Jetbot will use the Jetson DNN Inference ROS2 node to run the SSD MobileNet V2 model, which can detect people and other objects in the camera feed. Jetbot will also use a lidar sensor to measure the distance to the detected objects and fuse the lidar input with the camera input. This way, Jetbot can not only see what is in front of it, but also how far away it is.

![jetbot_sools](docs/JetBot_tool_design.png)
