#!/bin/bash

set -e

# Source ROS 2 setup file
source /opt/ros/humble/setup.bash

# Create a workspace directory
mkdir -p ~/ros2_ws/src

# Copy the project files to the workspace
cp -r . ~/ros2_ws/src/project_clearpath

# Change to the workspace directory
cd ~/ros2_ws

# Set gazebo_msgs_DIR if needed
export gazebo_msgs_DIR=/opt/ros/humble/share/gazebo_msgs/cmake

# Clean up any previous builds
rm -rf install log build

# Build the project with coverage flags using merged layout
colcon build --cmake-args -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_C_FLAGS="--coverage" -DCMAKE_BUILD_TYPE=Debug

# Source the workspace
source install/setup.bash

colcon test --event-handlers console_direct+
colcon test-result --test-result-base build/clearpath


# # Run the tests and generate coverage report only if build was successful
# if [ $? -eq 0 ]; then
#     # Run tests and capture the return code
#     colcon test --event-handlers console_direct+
#     TEST_RESULT=$?

#     if [ $TEST_RESULT -ne 0 ]; then
#         echo "Tests failed, stopping execution."
#         exit 1
#     fi

#     # Generate coverage report if tests pass
#     lcov --capture --directory build --output-file coverage.info
#     lcov --remove coverage.info '/opt/*' '/usr/*' '*/test/*' --output-file coverage_filtered.info
#     genhtml coverage_filtered.info --output-directory coverage_report

#     # Copy the coverage info file to the GitHub workspace
#     cp coverage_filtered.info $GITHUB_WORKSPACE/build/test_coverage.info
# else
#     echo "Build failed, skipping tests and coverage report generation."
#     exit 1
# fi
