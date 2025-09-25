#!/bin/bash

readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

pushd "$(pwd)" > /dev/null
cd "$(dirname "$0")"
echo "Working Path: $(pwd)"

ROS_VERSION=""
ROS_HUMBLE=""

# Set working ROS version
if [ "$1" = "humble" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_HUMBLE=${VERSION_HUMBLE}
else
    echo "Invalid Argument. Usage: ./build.sh humble"
    exit 1
fi

echo "ROS version is: $ROS_VERSION"

# clean only the livox_ros_driver2 build/install/log folders
cd ../..
rm -rf build/livox_ros_driver2 install/livox_ros_driver2 log

# switch to package source dir
cd src/livox_ros_driver2

# Replace package.xml for ROS 2
rm -f package.xml
cp -f package_ROS2.xml package.xml
rm -rf launch/
cp -rf launch_ROS2/ launch/

# Build only livox_ros_driver2
cd ../..
colcon build --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=${ROS_VERSION} -DHUMBLE_ROS=${ROS_HUMBLE}

# Clean up temp launch folder
rm -rf src/livox_ros_driver2/launch/

popd > /dev/null

