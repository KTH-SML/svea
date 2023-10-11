#! /bin/sh
#
# This dot-script can be sourced for environment variables describing this
# project.
#
# Author: Kaj Munhoz Arfvidsson

ROSDISTRO="noetic"

main() {
    if [ "$BUILD_CONFIG" = "base" ]; then
        BUILD_FILE="Dockerfile.base"
        BUILD_TAG="ros:$ROSDISTRO"
        IMAGE_TAG="ghcr.io/kth-sml/svea:$(uname -m)-base"
    elif [ "$BUILD_CONFIG" = "desktop" ]; then
        BUILD_FILE="Dockerfile.base"
        BUILD_TAG="osrf/ros:$ROSDISTRO-desktop"
        IMAGE_TAG="ghcr.io/kth-sml/svea:$(uname -m)-desktop"
    fi

    REPOSITORY_PATH="$(climb entrypoint)"
    REPOSITORY_NAME="$(basename "$REPOSITORY_PATH")"

    BUILD_CONTEXT="$REPOSITORY_PATH"
    BUILD_FILE="$BUILD_CONTEXT/docker/${BUILD_FILE:-"Dockerfile"}"
    BUILD_TAG="${BUILD_TAG:-"ghcr.io/kth-sml/svea"}"
    IMAGE_TAG="${IMAGE_TAG:-$REPOSITORY_NAME}"
    CONTAINER_NAME="$REPOSITORY_NAME"

    WORKSPACE="/svea_ws"
    SHRVOL_SRC="$BUILD_CONTEXT/src"
    SHRVOL_DST="$WORKSPACE/src"

    if istrue DESKTOP; then
        BUILD_TAG="ghcr.io/kth-sml/svea:$(uname -m)-desktop"
    fi

    if [ -n "$DEBUG" ]; then
        if [ -n "$BUILD_CONFIG" ]; then
            echo ""
            echo "BUILD_CONFIG=$BUILD_CONFIG"
        fi
        echo ""
        echo "REPOSITORY_PATH=$REPOSITORY_PATH"
        echo "REPOSITORY_NAME=$REPOSITORY_NAME"
        echo "BUILD_CONTEXT=$BUILD_CONTEXT"
        echo "BUILD_TAG=$BUILD_TAG"
        echo "IMAGE_TAG=$IMAGE_TAG"
        echo "CONTAINER_NAME=$CONTAINER_NAME"
        echo "ROSDISTRO=$ROSDISTRO"
        echo "WORKSPACE=$WORKSPACE"
        echo "SHRVOL_SRC=$SHRVOL_SRC"
        echo "SHRVOL_DST=$SHRVOL_DST"
        echo
    fi
}

################################################################################
################################################################################

##
## https://gist.github.com/kaarmu/123f536abd46fc86b0d720c8137a13a7
##

# Print error and exit
# > panic [CODE] MESSAGE
panic() {
    [ $# -gt 1 ] && CODE=$1 && shift || CODE=1
    echo "Error ($CODE): $1"
    usage
    exit $CODE
}

# Assert there exist commands
# > assert_command [COMMANDS...]
assert_command() {
    for cmd in "$@"; do
        [ ! "$(command -v "$cmd")" ] && panic "Missing command \"$cmd\". Is it installed?"
    done
}

# Return the argument at index
# > index [-]NUM [ARGS...]
index() {
    [ $# -lt 2 ] && return 1
    [ $1 -gt 0 ] && i=$1 || i=$(($# + $1))
    [ $i -le 0 ] && echo "" || (shift $i && echo "$1")
}

# Replace first substring with something else
# > replace SUB NEW TEXT
replace() {
    case "$3" in
    *"$1"*) echo "${3%%$1*}$2${3#*$1}" ;;
    "$3") echo "$3" ;;
    esac
}

# Climb directories to find an existing path CHILD
# > climb CHILD [ PARENT ]
climb() {
    CHILD="$1"
    PARENT="$(realpath "${2:-$PWD}")"
    [ -e "$PARENT/$CHILD" ] && echo "$PARENT" && return 0
    [ "$PARENT" = "/" ] && return 1
    climb "$CHILD" "$PARENT/.."
    return $?
}

# Assert shell variable with name NAME is the number one
# > istrue NAME
istrue() {
    VALUE="$(eval echo "\$$1")"
    test "${VALUE:-0}" -eq 1
    return $?
}

################################################################################
################################################################################

main
