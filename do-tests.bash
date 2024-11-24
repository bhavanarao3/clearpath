# #!/bin/bash

# set -e  # Exit on error

# # Print system info
# echo "System information:"
# uname -a
# lsb_release -a

# # Clone your repository to make sure it's available
# echo "Cloning the clearpath repository..."
# git clone https://github.com/bhavanarao3/clearpath.git
# cd clearpath

# # Set up ROS workspace
# echo "Setting up ROS workspace..."
# source /opt/ros/humble/setup.bash

# # Initialize and update rosdep (required for installing dependencies)
# echo "Initializing rosdep..."
# # Remove the existing sources list file
# sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
# sudo rosdep init
# rosdep update

# # Source the correct setup file from the workspace
# source $PWD/install/setup.bash  # Correct the path to your setup.bash

# # Install dependencies
# echo "Installing dependencies..."
# rosdep install --from-paths src --ignore-src -r -y

# # Build the workspace
# echo "Building workspace..."
# colcon build --symlink-install

# # Run unit tests with coverage
# echo "Running tests and generating coverage report..."
# colcon test --event-handler console_cohesion+ --output-dir /tmp/test_results

# # Generate a combined coverage report
# lcov --capture --directory . --output-file coverage.info
# lcov --remove coverage.info '*/test/*' --output-file coverage.info  # Exclude test directories
# lcov --list coverage.info

# # Optionally, you can use `gcovr` to generate reports in different formats (e.g., HTML, XML)
# gcovr -r . --html --html-details -o coverage_report.html

# # Upload coverage result to Codecov
# echo "Uploading coverage results to Codecov..."
# # Codecov will automatically use the environment variable CODECOV_TOKEN, which should be set in your GitHub repository secrets
# if [[ -n "$CODECOV_TOKEN" ]]; then
#     curl -s https://codecov.io/bash > codecov
#     bash codecov -t $CODECOV_TOKEN
# else
#     echo "CODECOV_TOKEN not set. Skipping Codecov upload."
# fi

# echo "Test run and coverage generation complete!"

#!/bin/bash
#
# A convenient script to run level 2 unit test (eg. integration test)
#
set -xue -o pipefail

##############################
# 0. start from scratch
##############################
rm -rf build/ install/
set +u                          # stop checking undefined variable  
source /opt/ros/humble/setup.bash
# sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
set -u                          # re-enable undefined variable check

##############################
# 1. Build for test coverage
##############################
colcon build --cmake-args -DCOVERAGE=1
set +u                          # stop checking undefined variable  
source install/setup.bash
set -u                          # re-enable undefined variable check

##############################
# 2. run all tests
##############################
colcon test

##############################
# 3. get return status  (none-zero will cause the script to exit)
##############################
colcon test-result --test-result-base build/clearpath

##############################
# 4. generate individual coverage reports:
##############################
## 4.1 my_model:
# colcon build \
#        --event-handlers console_cohesion+ \
#        --packages-select 808x-final-project \
#        --cmake-target "test_coverage" \
#        --cmake-arg -DUNIT_TEST_ALREADY_RAN=1
# MY_MODEL_COVERAGE_INFO=./build/808x-final-project/test_coverage.info
# ## 4.2 my_controller:
# ros2 run 808x-final-project generate_coverage_report.bash
# MY_CONTROLLER_COVERAGE_INFO=./build/808x-final-project/test_coverage.info

# ##############################
# # 5. Combine coverage reports
# ##############################
# ## create output directory
# COMBINED_TEST_COVERAGE=combined_test_coverage
# if [[ -d $COMBINED_TEST_COVERAGE ]]; then
#    rm -rf $COMBINED_TEST_COVERAGE
# fi
# mkdir $COMBINED_TEST_COVERAGE
# ## combine the reports
# ALL_COVERAGE_INFO=./build/test_coverage_merged.info
# lcov -a $MY_MODEL_COVERAGE_INFO -a \
#      $MY_CONTROLLER_COVERAGE_INFO -o \
#      $ALL_COVERAGE_INFO

# genhtml --output-dir $COMBINED_TEST_COVERAGE $ALL_COVERAGE_INFO

# ##############################
# # 6. show the combined coverage report
# ##############################
# open $COMBINED_TEST_COVERAGE/index.html || true
