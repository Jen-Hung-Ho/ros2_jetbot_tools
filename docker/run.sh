#!/bin/bash
# xhost +

# Get the user id and group id
ROOT="$(dirname "$(readlink -f "$0")")"
USER_ID=$(id -u)
GROUP_ID=$(id -g)

# Set the environment variables
DISPLAY_VAR=$DISPLAY
ROS_DOMAIN_ID=7

# Set the volume mappings
VOLUME_X11=/tmp/.X11-unix/:/tmp/.X11-unix:rw
VOLUME_ROS_LOG=$HOME/.ros/log:/.ros/log


# Define Docker volumes and environment variables
ROOT=$(dirname "$0")
DOCKER_VOLUMES="
--volume=$VOLUME_X11 \
--volume /tmp/argus_socket:/tmp/argus_socket \
--volume=$ROOT/../../jetbot_tools:/ros2_ws/src/jetbot_tools \
--volume=$ROOT/../../jetbot_tools/data:/data \
--volume=$ROOT/../../test:/ros2_ws/src/test \
--volume=$VOLUME_ROS_LOG \
--volume=/ros2_ws:/ros2_ws \
"


DOCKER_ENV_VARS="
--env DISPLAY=$DISPLAY_VAR \
--env QT_X11_NO_MITSHM=1 \
--env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
"
# check for V4L2 devices
V4L2_DEVICES=""

for i in {0..9}
do
    if [ -a "/dev/video$i" ]; then
        V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
    fi
done

# check for I2C devices
I2C_DEVICES=""

for i in {0..9}
do
    if [ -a "/dev/i2c-$i" ]; then
        I2C_DEVICES="$I2C_DEVICES --device /dev/i2c-$i "
    fi
done

DOCKER_DEVICES="
--device /dev/snd \
--device /dev/bus/usb \
--device=/dev/input \
"

# extra flags
EXTRA_FLAGS=""

if [ -n "$HUGGINGFACE_TOKEN" ]; then
    EXTRA_FLAGS="$EXTRA_FLAGS --env HUGGINGFACE_TOKEN=$HUGGINGFACE_TOKEN"
fi

DOCKER_ARGS="${DOCKER_VOLUMES} ${DOCKER_ENV_VARS} ${DOCKER_DEVICES} ${V4L2_DEVICES} ${I2C_DEVICES} ${EXTRA_FLAGS}"

# Set the docker image
DOCKER_IMAGE=${DOCKER_IMAGE:-jetbot_nano_llm:latest}

# check if sudo is needed
if [ $(id -u) -eq 0 ] || id -nG "$USER" | grep -qw "docker"; then
    SUDO=""
else
    echo "run as ROOT user"
    SUDO="sudo"
fi

# Run the docker command
# Check if the first input parameter is 'admin'
if [ "$1" == "user" ]; then
    $SUDO docker run --runtime nvidia -it --user $USER_ID:$GROUP_ID --rm --net host --ipc host \
    ${DOCKER_ARGS} \
    $DOCKER_IMAGE /bin/bash -c "source install/setup.bash && /bin/bash"
else
    $SUDO docker run --runtime nvidia -it --rm --net host --ipc host\
    ${DOCKER_ARGS} \
    $DOCKER_IMAGE /bin/bash -c "source install/setup.bash && /bin/bash"
fi
