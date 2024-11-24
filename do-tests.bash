#!/bin/bash

set -e

# Source ROS 2 setup file
source /opt/ros/humble/setup.bash

# Install necessary ROS 2 and Gazebo packages
sudo apt update
sudo apt install -y ros-humble-ament-cmake python3-colcon-common-extensions ros-humble-gazebo-ros-pkgs

# Create a workspace directory
mkdir -p ~/ros2_ws/src

# Copy the project files to the workspace
cp -r . ~/ros2_ws/src/project

# Change to the workspace directory
cd ~/ros2_ws

# Set gazebo_msgs_DIR if needed
export gazebo_msgs_DIR=/opt/ros/humble/share/gazebo_msgs/cmake

# Build the project with coverage flags
colcon build --cmake-args -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_C_FLAGS="--coverage" -DCMAKE_BUILD_TYPE=Debug

# Source the workspace
source install/setup.bash

# Run the tests
colcon test

# Generate coverage report
lcov --capture --directory build --output-file coverage.info
lcov --remove coverage.info '/opt/*' '/usr/*' '*/test/*' --output-file coverage_filtered.info
genhtml coverage_filtered.info --output-directory coverage_report

# Copy the coverage info file to the GitHub workspace
cp coverage_filtered.info $GITHUB_WORKSPACE/build/test_coverage.info

# Print the coverage report
lcov --list coverage_filtered.info
