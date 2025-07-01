FROM ros:jazzy-ros-base AS micro-ros-agent-builder

ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /svea_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
&&  git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_msgs src/micro_ros_msgs \
&&  git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ROS-Agent src/micro-ROS-Agent \
&&  colcon build \
&&  rm -rf log/ build/ src/

# Use the official ROS 2 Jazzy image as the base image
FROM ros:jazzy-ros-base

# Set the ROS distribution as a build argument
ARG ROSDISTRO="jazzy"
ENV ROSDISTRO=$ROSDISTRO

COPY --from=micro-ros-agent-builder /svea_ws /svea_ws

# Install bootstrap tools and ROS 2 packages
RUN apt-get update  -y && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
    build-essential \
    apt-utils lsb-release \
    cmake \
    git vim nano curl iputils-ping \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-$ROSDISTRO-ament-cmake-python \
    ros-$ROSDISTRO-ros-base=0.11.0-1* \
    # ros-$ROSDISTRO-rmw-zenoh-cpp\
    ros-$ROSDISTRO-teleop-twist-keyboard\
    && rm -rf /var/lib/apt/lists/*

# ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp


# Bootstrap rosdep and setup colcon mixin and metadata
# RUN rosdep init && \
RUN rosdep update --rosdistro $ROSDISTRO

# Create a Python virtual environment
# ENV VIRTUAL_ENV=/opt/venv
# RUN python3 -m venv $VIRTUAL_ENV
# ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Set up the working directory
WORKDIR /svea_ws

# Copy the ROS 2 workspace and utility files into the container
COPY src ./src
COPY util ./util
COPY entrypoint requirements.txt ./

# Update rosdep and install dependencies for the ROS 2 workspace
RUN apt-get update -y && \
    rosdep update --rosdistro $ROSDISTRO && \
    rosdep install --rosdistro $ROSDISTRO --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies in the virtual environment
# RUN pip install --break-system-packages --upgrade pip && \
RUN pip install --break-system-packages -r requirements.txt

# Ensure proper environment setup before build
ENV CMAKE_PREFIX_PATH="/opt/ros/$ROSDISTRO"

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/$ROSDISTRO/setup.bash && \
    colcon build --symlink-install"

COPY disable_fastdds_shm.xml disable_fastdds_shm_localhost_only.xml /tmp/
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV MICROROS_DISABLE_SHM=1

# Source the setup file in the .bashrc for convenience
RUN echo "source /opt/ros/$ROSDISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /svea_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /svea_ws/install/local_setup.bash" >> ~/.bashrc

# For fixing WSL error
RUN chmod +x ./entrypoint

# Set the entrypoint to the custom entrypoint script
ENTRYPOINT ["./entrypoint"]

# Set the default command to run when the container starts
CMD ["bash"]
