#!/bin/bash

# Check if a container ID is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <container_id>"
else
    CONTAINER_ID=$1
    # Execute the command inside the specified container
    # export ROS_DOMAIN_ID=7
    # source /opt/ros/install/setup.bash
    # source /ros2_workspace/install/setup.bash
    docker exec -it $CONTAINER_ID /bin/bash -c "export ROS_DOMAIN_ID=7 && source /opt/ros/install/setup.bash && source /ros2_workspace/install/setup.bash && exec /bin/bash"
fi