#! /bin/sh
#
# This dot-script can be sourced for environment variables describing this
# project.
#
# The following variables can be override:
# * BUILD_FILE
# * BUILD_IMAGE
# * BUILD_TAG
#
# There are examples of user configs for our normal hardware below. You must
# choose one of these for e.g. GPU support.
#
# Author: Kaj Munhoz Arfvidsson

## ------------------ ##
## User Configuration ##
## ------------------ ##

## General x86/arm (prebuilt SVEA)
#BUILD_IMAGE="ghcr.io/kth-sml/svea"
#BUILD_TAG="latest"

## TODO
## General x86/arm + NVIDIA GPU (prebuilt, CUDA enabled)
#BUILD_IMAGE="ghcr.io/kth-sml/svea"
#BUILD_TAG="cuda"

## TODO
## Jetson TX2 (L4T 4.x)
#BUILD_IMAGE="ghcr.io/kth-sml/svea"
#BUILD_TAG="tx2"

## TODO
## Jetson Xavier AGX (L4T 5.x)
#BUILD_TAG="xavier-agx"

## --------------------------------- ##
## Automatic Configuration Variables ##
## --------------------------------- ##

. "$(dirname "$0")/snippets.sh"

REPOSITORY_PATH="$(climb .git)"
REPOSITORY_NAME="$(basename "$REPOSITORY_PATH")"

BUILD_CONTEXT="$REPOSITORY_PATH"
BUILD_FILE="${BUILD_FILE:-"$REPOSITORY_PATH/docker/Dockerfile"}"
BUILD_IMAGE="${BUILD_IMAGE:-"ros"}"
BUILD_TAG="${BUILD_TAG:-"noetic"}"
IMAGE_TAG="$(basename "$BUILD_CONTEXT")"
IMAGE_TAG="${IMAGE_TAG%%.*}"

ROSDISTRO="noetic"
WORKSPACE="/$IMAGE_TAG"
SHARED_VOLUME_SRC="$REPOSITORY_PATH/src"
SHARED_VOLUME_DST="$WORKSPACE/src"

if [ -n "$DEBUG" ]; then
    echo "REPOSITORY_PATH=$REPOSITORY_PATH"
    echo "REPOSITORY_NAME=$REPOSITORY_NAME"
    echo "BUILD_CONTEXT=$BUILD_CONTEXT"
    echo "BUILD_FILE=$BUILD_FILE"
    echo "BUILD_IMAGE=$BUILD_IMAGE"
    echo "BUILD_TAG=$BUILD_TAG"
    echo "IMAGE_TAG=$IMAGE_TAG"
    echo "ROSDISTRO=$ROSDISTRO"
    echo "WORKSPACE=$WORKSPACE"
    echo "SHARED_VOLUME_SRC=$SHARED_VOLUME_SRC"
    echo "SHARED_VOLUME_DST=$SHARED_VOLUME_DST"
    echo
fi

