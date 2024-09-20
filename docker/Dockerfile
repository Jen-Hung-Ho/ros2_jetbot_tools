# Use an argument for the base image
ARG BASE_IMAGE=dustynv/nano_llm:humble-r36.3.0
FROM ${BASE_IMAGE}

# Use arguments for user, group IDs, and username
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=jetbot

# Create a new user with specified user id, group id, and username
RUN groupadd -g ${GROUP_ID} ${USERNAME} && \
    useradd -u ${USER_ID} -g ${USERNAME} -m -s /bin/bash ${USERNAME} && \
    echo "${USERNAME}:${USERNAME}" | chpasswd && \
    adduser ${USERNAME} sudo && \
    (getent group video || groupadd -r video) && \
    usermod -aG video ${USERNAME}


# Change the ownership of the home directory to the new user
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# Create the necessary directory
RUN mkdir -p /home/${USERNAME}/.cache/clip_trt && \
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.cache

# Change the ownership of /opt/jetson-inference, /ros2_workspace, /opt/NanoLLM, /opt/clip_trt, and /opt/NanoDB to the new user
RUN chown -R ${USERNAME}:${USERNAME} /opt/jetson-inference /ros2_workspace /opt/NanoLLM /opt/clip_trt /opt/NanoDB


# Create the working directory
RUN mkdir -p /test

# Set the working directory in the Docker image
WORKDIR /test

# Copy requirements.txt to the working directory
COPY requirements.txt /test/requirements.txt

# Update the package list and install PyKDL
RUN apt-get update && apt-get install -y \
    liborocos-kdl-dev \
    python3-pykdl

# Update the package list and install vi
RUN apt-get install -y vim

# Install any necessary dependencies
RUN pip install --no-cache-dir -r /test/requirements.txt

# Set PYTHONPATH environment variable for both users
ENV PYTHONPATH=/opt/NanoLLM:/opt/clip_trt:/opt/NanoDB:$PYTHONPATH


# Set up the ROS2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# Set the default shell
ENV SHELL=/bin/bash

# Set the default command
CMD ["/bin/bash"]
