<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
# Dockerfile for SVEA images
#
# Author: Kaj Munhoz Arfvidsson
=======
FROM ros:jazzy-ros-base AS micro-ros-agent-builder
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
=======
FROM ros:jazzy AS micro-ros-agent-builder
>>>>>>> c1156cb (add micro-ros agent in docker)
=======
# Dockerfile for SVEA images
#
# Author: Kaj Munhoz Arfvidsson
>>>>>>> 25eba17 (update docker build)

ARG BUILD_TAG
ARG ROSDISTRO

FROM ${BUILD_TAG:-ghcr.io/kth-sml/svea:latest}

ARG ROSDISTRO
ARG DEBIAN_FRONTEND=noninteractive

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 28d5445 (ooops)
=======
>>>>>>> c1156cb (add micro-ros agent in docker)
# Use the official ROS 2 Jazzy image as the base image
FROM ros:jazzy-ros-base-noble
=======
## Install dependencies using apt-get
>>>>>>> 25eba17 (update docker build)

RUN apt-get update -y

# # You can add apt dependencies like this...
# RUN apt-get install --no-install-recommends -y \
#         git curl

# This is for any added ROS packages
COPY src ./src
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 28d5445 (ooops)
=======
>>>>>>> d0ce68c (Added mocap4ros2_qualisys dep import to build process)
=======
=======
>>>>>>> 68e62af (update docker build)
>>>>>>> 25eba17 (update docker build)
COPY requirements.txt ./
RUN rosdep install \
        --rosdistro $ROSDISTRO \
        --from-paths src \
        --ignore-src \
        -qry \
        && \
<<<<<<< HEAD
<<<<<<< HEAD
    pip install \
        --break-system-packages \
        -r requirements.txt

# Build the ROS 2 workspace (improves build speeds later on).
RUN /bin/bash -c "source /opt/ros/$ROSDISTRO/setup.bash && \
    colcon build --symlink-install"
=======
COPY util ./util
COPY entrypoint requirements.txt ./
=======
=======
>>>>>>> 25eba17 (update docker build)
    pip install -r requirements.txt
<<<<<<< HEAD
>>>>>>> 28d5445 (ooops)
=======
=======
COPY util ./util
COPY entrypoint requirements.txt ./

# Automatically import extra dependencies if dependency_repos.repos exists
RUN if [ -f src/mocap4ros2_qualisys/dependency_repos.repos ]; then \
      vcs import src < src/mocap4ros2_qualisys/dependency_repos.repos; \
    fi
    
# Update rosdep and install dependencies for the ROS 2 workspace
RUN apt-get update -y && \
    rosdep update --rosdistro $ROSDISTRO && \
    rosdep install --rosdistro $ROSDISTRO --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies in the virtual environment
# RUN pip install --break-system-packages --upgrade pip && \
RUN pip install --break-system-packages -r requirements.txt
>>>>>>> 739744d (Added mocap4ros2_qualisys dep import to build process)
>>>>>>> d0ce68c (Added mocap4ros2_qualisys dep import to build process)

# Ensure proper environment setup before build
ENV CMAKE_PREFIX_PATH="/opt/ros/$ROSDISTRO"

<<<<<<< HEAD
<<<<<<< HEAD
# Install Python dependencies in the virtual environment
# RUN pip install --break-system-packages --upgrade pip && \
RUN pip install --break-system-packages -r requirements.txt

# Ensure proper environment setup before build
ENV CMAKE_PREFIX_PATH="/opt/ros/$ROSDISTRO"

=======
>>>>>>> a6d4456 (add svea_localization,modifying to ros2 in progress)
# Build the ROS 2 workspace
=======
    pip install \
        --break-system-packages \
        -r requirements.txt

# Build the ROS 2 workspace (improves build speeds later on).
>>>>>>> 68e62af (update docker build)
RUN /bin/bash -c "source /opt/ros/$ROSDISTRO/setup.bash && \
    colcon build --symlink-install"
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
COPY firmware/disable_fastdds_shm.xml \
     firmware/disable_fastdds_shm_localhost_only.xml \
     /tmp/
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV MICROROS_DISABLE_SHM=1

# Source the setup file in the .bashrc for convenience
RUN echo "source /opt/ros/$ROSDISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /svea_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /svea_ws/install/local_setup.bash" >> ~/.bashrc
=======
# Source the setup file in the .bashrc for convenience
RUN echo "source /opt/ros/$ROSDISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /svea_ws/install/setup.bash" >> ~/.bashrc
>>>>>>> a6d4456 (add svea_localization,modifying to ros2 in progress)
=======
COPY disable_fastdds_shm.xml disable_fastdds_shm_localhost_only.xml /tmp/
=======
COPY firmware/disable_fastdds_shm.xml \
     firmware/disable_fastdds_shm_localhost_only.xml \
     /tmp/
>>>>>>> 8d5fdd9 (Add configuration files to disable Fast-DDS SHM transport for UDPv4)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV MICROROS_DISABLE_SHM=1

# Source the setup file in the .bashrc for convenience
RUN echo "source /opt/ros/$ROSDISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /svea_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /svea_ws/install/local_setup.bash" >> ~/.bashrc
>>>>>>> c1156cb (add micro-ros agent in docker)

# For fixing WSL error
RUN chmod +x ./entrypoint

# Set the entrypoint to the custom entrypoint script
ENTRYPOINT ["./entrypoint"]

# Set the default command to run when the container starts
CMD ["bash"]
<<<<<<< HEAD
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
=======
>>>>>>> 28d5445 (ooops)
=======
>>>>>>> a6d4456 (add svea_localization,modifying to ros2 in progress)
=======
>>>>>>> 25eba17 (update docker build)
