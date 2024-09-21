## Jetbot tools setup

1. **Configure Docker Engine**:
   Follow these [setup steps](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md) to configure your Docker engine.

2. **Set Up ROS2 Development Environment**:
   Set up your ROS2 development environment by following the instructions [here](https://docs.ros.org/en/humble/Installation.html).

3. **Clone the Repository**:
   Open your terminal and run the following command to clone the repository:
   ```bash
   https://github.com/Jen-Hung-Ho/ros2_jetbot_tools
   ```

4. **Navigate to the Repository Directory**:
   Change to the directory of the cloned repository:
   ```bash
   cd ros2_jetbot_tools/docker
   ```

5. **Build the Docker Image**:
   Run the `build.sh` script to build the Docker image:
   ```bash
   ./build.sh
   ```

6. **Start Docker in user mode and Build the jetbot tools ros2 package**:

   Execute the following commands to run the Docker container under user mode and build the jetbot tools ROS2 packages:

   ```bash
   . run.sh user
   colcon build # Run this command only the first time after building the Docker image or when changes are made to the Jetbot tools ROS2 code.
   ```

   **More Info**: Run nodes inside the container with the same user ID as ROS2 nodes on the host. For more details, you can refer to the following resources:
   - [ROS2 Template GitHub](https://github.com/rosblox/ros-template)
   - [Stack Overflow Discussion](https://stackoverflow.com/questions/65900201/troubles-communicating-with-ros2-node-in-docker-container)
   
7. **Attach to an Existing Running Docker Container**:
   To attach to an existing running Docker container, use the following commands:
   ```bash
   docker ps 
   ```

   Identify the `CONTAINER ID` of the running container (e.g., `422fc05b7655`), then run:
   ```bash
   . start_ros2_shell.sh <CONTAINER_ID>
   ```

   For example:
   ```bash
   . start_ros2_shell.sh 422fc05b7655
   ```
