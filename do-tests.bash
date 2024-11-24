#!/bin/bash

set -e  # Exit on error

# Print system info
echo "System information:"
uname -a
lsb_release -a

# Set up ROS workspace
echo "Setting up ROS workspace..."
source /opt/ros/humble/setup.bash
source $PWD/install/setup.bash  # Correct the path to your setup.bash

# Install dependencies (if any)
echo "Installing dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo "Building workspace..."
colcon build --symlink-install

# Run unit tests with coverage
echo "Running tests and generating coverage report..."
# Run tests using colcon test
colcon test --event-handler console_cohesion+ --output-dir /tmp/test_results

# Generate a combined coverage report
lcov --capture --directory . --output-file coverage.info
lcov --remove coverage.info '*/test/*' --output-file coverage.info  # Exclude test directories
lcov --list coverage.info

# Optionally, you can use `gcovr` to generate reports in different formats (e.g., HTML, XML)
gcovr -r . --html --html-details -o coverage_report.html

# Upload coverage result to Codecov
echo "Uploading coverage results to Codecov..."
# Codecov will automatically use the environment variable CODECOV_TOKEN, which should be set in your GitHub repository secrets
if [[ -n "$CODECOV_TOKEN" ]]; then
    curl -s https://codecov.io/bash > codecov
    bash codecov -t $CODECOV_TOKEN
else
    echo "CODECOV_TOKEN not set. Skipping Codecov upload."
fi

echo "Test run and coverage generation complete!"
