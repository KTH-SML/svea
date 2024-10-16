#! /bin/sh
#
# This dot-script can be sourced for environment variables describing this
# project.
#
# Author: Kaj Munhoz Arfvidsson

## Uncomment to build base image for amd64 (x86)/arm64.
# CONFIG="base-amd64"
# CONFIG="base-arm64"

main() {

    CONFIG="${CONFIG:-"deafult"}"
    DEFAULT_ROSDISTRO="noetic"
    DEFAULT_WORKSPACE="/svea_ws"
    DEFAULT_BUILD_PLATFORM="linux/amd64"
    DEFAULT_BUILD_FILE="docker/Dockerfile"
    DEFAULT_BUILD_TAG="latest"
    DEFAULT_IMAGE_PUSH="0"

    if [ "$CONFIG" = "base" ]; then
        DEFAULT_BUILD_PLATFORM="linux/amd64,linux/arm64/v8"
        DEFAULT_BUILD_FILE="docker/Dockerfile.base"
        DEFAULT_BUILD_TAG="$DEFAULT_ROSDISTRO"
	IMAGE_PUSH="1"
        IMAGE_TAG="ghcr.io/kth-sml/svea"
    elif [ "$CONFIG" = "base-amd64" ]; then
        DEFAULT_BUILD_PLATFORM="linux/amd64"
        DEFAULT_BUILD_FILE="docker/Dockerfile.base"
        DEFAULT_BUILD_TAG="$DEFAULT_ROSDISTRO"
        IMAGE_TAG="ghcr.io/kth-sml/svea"
    elif [ "$CONFIG" = "base-arm64" ]; then
        DEFAULT_BUILD_PLATFORM="linux/arm64/v8"
        DEFAULT_BUILD_FILE="docker/Dockerfile.base"
        DEFAULT_BUILD_TAG="$DEFAULT_ROSDISTRO"
        IMAGE_TAG="ghcr.io/kth-sml/svea"
    fi

    ROSDISTRO="${ROSDISTRO:-"$DEFAULT_ROSDISTRO"}"
    WORKSPACE="${WORKSPACE:-"$DEFAULT_WORKSPACE"}"

    REPOSITORY_PATH="$(climb entrypoint)"
    REPOSITORY_NAME="$(basename "$REPOSITORY_PATH")"

    BUILD_PLATFORM="${BUILD_PLATFORM:-"$DEFAULT_BUILD_PLATFORM"}"
    BUILD_CONTEXT="${BUILD_CONTEXT:-"$REPOSITORY_PATH"}"
    BUILD_FILE="$BUILD_CONTEXT/${BUILD_FILE:-"$DEFAULT_BUILD_FILE"}"
    BUILD_TAG="${BUILD_TAG:-"$DEFAULT_BUILD_TAG"}"

    IMAGE_PUSH="${IMAGE_PUSH:-"$DEFAULT_IMAGE_PUSH"}"
    IMAGE_TAG="${IMAGE_TAG:-"$REPOSITORY_NAME"}"

    CONTAINER_NAME="$REPOSITORY_NAME"
    SHARED_VOLUME="$BUILD_CONTEXT/src:$WORKSPACE/src"

    if [ -n "$DEBUG" ]; then
        echo ""
        echo "ROSDISTRO=$ROSDISTRO"
        echo "WORKSPACE=$WORKSPACE"
        echo "REPOSITORY_PATH=$REPOSITORY_PATH"
        echo "REPOSITORY_NAME=$REPOSITORY_NAME"
        echo "BUILD_PLATFORM=$BUILD_PLATFORM"
        echo "BUILD_CONTEXT=$BUILD_CONTEXT"
        echo "BUILD_FILE=$BUILD_FILE"
        echo "BUILD_TAG=$BUILD_TAG"
        echo "IMAGE_PUSH=$IMAGE_PUSH"
        echo "IMAGE_TAG=$IMAGE_TAG"
        echo "CONTAINER_NAME=$CONTAINER_NAME"
        echo "SHARED_VOLUME=$SHARED_VOLUME"
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

# Echo TRUTHY if `istrue NAME` else FALSY
# > ifelse NAME TRUTHY FALSY
ifelse() {
    if istrue "$1"
    then echo "$2"
    else echo "$3"
    fi
}

# Append ARGS to shell variable with name NAME
# > append LIST_NAME [ARGS...]
append() {
    LIST_NAME="$1"
    shift
    eval "$LIST_NAME=\"\${$LIST_NAME} $*\""
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
