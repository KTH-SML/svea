#! /bin/sh
#
# This dot-script can be sourced for environment variables describing this
# project.
#
# Author: Kaj Munhoz Arfvidsson

## Uncomment to build base image for amd64 (x86)/arm64.
# CONFIG="base-amd64"
CONFIG="base-arm64"

main() {

    withdefault DEBUG "0"

    withdefault ROSDISTRO "noetic"
    withdefault WORKSPACE "/svea_ws"
    withdefault REPOSITORY_PATH "$(climb entrypoint)"
    withdefault REPOSITORY_NAME "$(basename "$REPOSITORY_PATH")"

    withdefault BUILD_CONFIG "default"
    if [ "$BUILD_CONFIG" = "default" ]; then
        # building for host platform
        withdefault BUILD_PLATFORM "$(uname -m)"
        withdefault BUILD_CONTEXT "$REPOSITORY_PATH"
        withdefault BUILD_FILE "docker/Dockerfile"
        withdefault BUILD_TAG "ghcr.io/kth-sml/svea:latest"
        withdefault IMAGE_TAG "$REPOSITORY_NAME"
        withdefault IMAGE_PUSH "0"
    elif [ "$BUILD_CONFIG" = "base" ]; then
        # building for host platform
        withdefault BUILD_PLATFORM "$(uname -m)"
        withdefault BUILD_CONTEXT "$REPOSITORY_PATH"
        withdefault BUILD_FILE "docker/Dockerfile.base"
        withdefault BUILD_TAG "ros:$ROSDISTRO"
        withdefault IMAGE_TAG "ghcr.io/kth-sml/svea:latest"
        withdefault IMAGE_PUSH "0"
    elif [ "$BUILD_CONFIG" = "base-amd64" ]; then
        # building for x86_64
        withdefault BUILD_PLATFORM "linux/amd64"
        withdefault BUILD_CONTEXT "$REPOSITORY_PATH"
        withdefault BUILD_FILE "docker/Dockerfile.base"
        withdefault BUILD_TAG "ros:$ROSDISTRO"
        withdefault IMAGE_TAG "ghcr.io/kth-sml/svea:latest"
        withdefault IMAGE_PUSH "0"
    elif [ "$BUILD_CONFIG" = "base-arm64" ]; then
        # building for arm64/aarch64/jetson
        withdefault BUILD_PLATFORM "linux/arm64"
        withdefault BUILD_CONTEXT "$REPOSITORY_PATH"
        withdefault BUILD_FILE "docker/Dockerfile.base"
        withdefault BUILD_TAG "ros:$ROSDISTRO"
        withdefault IMAGE_TAG "ghcr.io/kth-sml/svea:latest"
        withdefault IMAGE_PUSH "0"
    elif [ "$BUILD_CONFIG" = "ghcr" ]; then
        # building for both amd64 and arm64
        withdefault BUILD_PLATFORM "linux/arm64,linux/amd64"
        withdefault BUILD_CONTEXT "$REPOSITORY_PATH"
        withdefault BUILD_FILE "docker/Dockerfile.base"
        withdefault BUILD_TAG "ros:$ROSDISTRO"
        withdefault IMAGE_TAG "ghcr.io/kth-sml/svea:latest"
        withdefault IMAGE_PUSH "1"
    fi

    withdefault CONTAINER_NAME "$REPOSITORY_NAME"
    withdefault SHARED_VOLUME "$BUILD_CONTEXT/src:$WORKSPACE/src"

    if istrue DEBUG; then
        echo ""
        echovar BUILD_CONFIG
        echo
        echovar ROSDISTRO
        echovar WORKSPACE
        echovar REPOSITORY_PATH
        echovar REPOSITORY_NAME
        echovar BUILD_PLATFORM
        echovar BUILD_CONTEXT
        echovar BUILD_FILE
        echovar BUILD_TAG
        echovar IMAGE_TAG
        echovar IMAGE_PUSH
        echovar CONTAINER_NAME
        echovar SHARED_VOLUME
        echo
    fi
}

call() {
    if istrue DEBUG; then
        echo "$@"
    else
        exec "$@"
    fi
}

jetson_release() {
    file="/etc/nv_tegra_release"
    if [ -f "$file" ]; then
        echo "$(index 2 $(cat "$file"))"
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

# Echo shell variable
# > echovar NAME
echovar() {
    eval "echo \"$1=\$$1\""
}

# Set shell variable with default
# > withdefault NAME DEFAULT
withdefault() {
    eval "$1=\"\${$1:-\"$2\"}\""
}

# Echo TRUTHY if `istrue NAME` else FALSY
# > ifelse NAME TRUTHY FALSY
ifelse() {
    if istrue "$1"; then
        echo "$2"
    else
        echo "$3"
    fi
}

# If VALUE equals COND then echo RET, otherwise shift
# and continue with following arguments
# > switch VALUE [[COND RET]...]
switch() {
    VALUE="$1"
    shift
    if [ "$VALUE" = "$1" ]; then
        echo "$2"
    elif [ "$#" -ge 2 ]; then
        shift 2
        switch "$VALUE" "$@"
    else
        echo "$@" # echo nothing or remaining arg
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
