#!/bin/bash

set -e

# --- ROS DISTRO ---
if [ $# -ge 1 ]; then
    DISTRO="$1"
elif [ -n "${ROS_DISTRO:-}" ]; then
    DISTRO="$ROS_DISTRO"
    echo "Using currently sourced ROS distro: $DISTRO"
else
    echo "Error: No ROS distribution specified and ROS_DISTRO not set."
    echo "Usage: $0 <ROS_DISTRO> [build_type]"
    echo "Example: $0 humble release"
    exit 1
fi

# Source ROS
source /opt/ros/$DISTRO/setup.bash

# --- CMAKE BUILD TYPE ---
if [ $# -lt 2 ]; then
    CMAKE_BUILD_TYPE="Debug"
else
    arg=$(echo "$2" | tr '[:upper:]' '[:lower:]')
    if [ "$arg" == "release" ]; then
        CMAKE_BUILD_TYPE="Release"
    elif [ "$arg" == "debug" ]; then
        CMAKE_BUILD_TYPE="Debug"
    else
        echo "Unknown build type: $2"
        exit 1
    fi
fi

# --- BUILD ---
colcon build --executor parallel \
    --packages-select bernard_hw_if \
    --symlink-install \
    --cmake-clean-cache \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
