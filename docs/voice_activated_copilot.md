## Empower your robot with Voice-Activated Copilot Tool:

 - Unleash the power of voice control for your ROS2 robot with the [Jetbot Voice-Activated Copilot Tool!](https://github.com/Jen-Hung-Ho/ros2_jetbot_voice)
  - The Jetbot Voice-Activated Copilot Tool integrates the [Nvidia RIVA (ASR-TTS) service](https://docs.nvidia.com/deeplearning/riva/user-guide/docs/overview.html) and a simple 1D convolutional neural network (CNN) model for text classification, empowering your robot to understand and respond to spoken commands. Enhance interactions with features such as natural chat greetings, conversational capabilities via Large Language Models (LLM), Visual Language Models (VLM) for vision processing, object avoidance, autonomous self-driving, real-time person following, and fundamental robot navigation movements.
  - In Jetbot Tools v2.1, the Voice‑Activated Copilot is now connected to the Jetbot Tools Task Copilot and the Jetbot Action Server, providing a unified ROS2 Action interface for managing robot behaviors. When a voice command is received, the Task Copilot can stop any currently running task and transition the robot into the newly requested action, ensuring smooth coordination between modules.
    - Enhance interactions with features such as natural chat greetings, conversational capabilities via Large Language Models (LLM), Visual Language Models (VLM) for vision processing, object avoidance, autonomous self‑driving, real‑time person following, and fundamental robot navigation movements
  
  - Source code:
    - [ROS2 node: llm_chat_agent.py](/jetbot_tools/script/llm_chat_agent.py)
    - [ROS2 node: llm_vision_agent.py](/jetbot_tools/script/llm_vision_agent.py)
    - [param file: jetbot_voice_copilot_params.yaml](/jetbot_tools/param/jetbot_voice_copilot_params.yaml)
    - [ROS2 action server: jetbot_action_server.py](/jetbot_action_server/script/jetbot_action_server.py)
    - [launch file: jetbot_tools_voice.launch.py](/jetbot_tools/launch/jetbot_tools_voice.launch.py)
    - [ROS2 node: jetbot_tools_copilot.py](/jetbot_tools/script/jetbot_tools_copilot.py)
    <br>
   The Jetbot Action Server must be running before starting any Copilot or agent nodes.
  - Usage:
    - ros2 run jetbot_action_server robot_command_action_server --ros-args --params-file ./jetbot_action_server/param/jetbot_action_server_params_v2.yaml
    - ros2 launch jetbot_tools jetbot_tools_voice.launch.py 
    - ros2 run jetbot_tools llm_chat_agent
    - ros2 run jetbot_tools llm_vision_agent
    - ros2 launch jetbot_tools jetbot_tools_voice.launch.py param_file:=./jetbot_tools/param/jetbot_voice_copilot_params.yaml
  [<img src="https://img.youtube.com/vi/SqDqO-KfWUs/hqdefault.jpg" width="300" height="200"
/>](https://youtu.be/SqDqO-KfWUs)
