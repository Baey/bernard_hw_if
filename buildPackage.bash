#!/bin/bash

set -e

# --- ROS DISTRO ---
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

# Source ROS
source /opt/ros/$DISTRO/setup.bash

# --- CMAKE BUILD TYPE ---
if [ $# -lt 2 ]; then
    CMAKE_BUILD_TYPE="Debug"
elif [ "$2" == "release" ]; then
    CMAKE_BUILD_TYPE="Release"
else
    echo "Unknown build type: $2"
    exit 1
fi

# --- BUILD ---
colcon build --executor parallel \
    --packages-select bernard_hw_if \
    --symlink-install \
    --cmake-clean-cache \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
