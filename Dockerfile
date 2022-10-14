# Dockerfile for SVEA image
#
# Author: Kaj Munhoz Arfvidsson

ARG BUILD_IMAGE
ARG BUILD_TAG
ARG ROSDISTRO
ARG WORKSPACE

#####################
## SVEA BASE IMAGE ##
#####################

FROM $BUILD_IMAGE:$BUILD_TAG

ARG ROSDISTRO
ARG WORKSPACE
ARG DEBIAN_FRONTEND=noninteractive

## Install dependencies from apt-get

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
        apt-utils \
        git vim nano curl iputils-ping \
        python3-tk \
        python3-pip \
        python3-numpy \
        python3-matplotlib \
        python3-catkin-tools \
        && \
    python3 -m pip install -U pip && \
    rm -rf /var/lib/apt/lists/*

## Create svea workspace

COPY . $WORKSPACE
WORKDIR $WORKSPACE

RUN cp -f entrypoint /ros_entrypoint.sh && \
    apt-get update -y && \
    rosdep update \
        --rosdistro $ROSDISTRO \
        && \
    rosdep install \
        --rosdistro $ROSDISTRO \
        --from-paths src \
        --ignore-src \
        -qry \
        && \
    pip install -r requirements.txt && \
    rm -rf /var/lib/apt/lists/*

# Run catkin build on workspace (improves build speeds later on).
# Ending on true is a hacky way to ensure the docker build continues
# even if the catkin build fail

RUN catkin config \
        --init \
        --mkdirs \
        --extend /opt/ros/$ROSDISTRO \
        > /dev/null \
        && \
    catkin build || \
    true

# Container entrypoint (executes util/entrypoint)
# bash is run by default when user starts

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
