
# Project Clearpath
ENPM700-Final Project

# C++ Boilerplate v2 Badges
![CICD Workflow status](https://github.com/bhavanarao3/clearpath/actions/workflows/run-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/bhavanarao3/clearpath/graph/badge.svg?token=cV6vdN6x4L)](https://codecov.io/gh/bhavanarao3/clearpath)[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

| Role / Part   | Phase 1                   | 
|---------------|---------------------------|
|Driver         |Sri ram Prasad Kothapalli  |
|Navigator      |Bhavana B Rao              |
|Design Keeper  |Tathya Bhatt               |



"Project Clearpath" is an autonomous robot designed for disaster site cleanup, capable of identifying, localizing, and prioritizing hazardous debris. It autonomously navigates sites, detecting debris types, estimating 3D locations, and assigning priority based on hazard level.
This repository contains deliverables for midterm project of Bhavana B Rao, Sriramprasad Kothapalli and Tathya Bhatt as a part of the course ENPM700: Software Development for Robotics at the University of Maryland.

## AIP

This project was developed using the Agile Development Process (AIP) along with pair programming (with a driver, navigator, and design keeper), with a focus on test-driven development (TDD). This [sheet](https://docs.google.com/spreadsheets/d/124zPjeAy8mCFpvf6AqBxNpdo9hfX9Y0hR05A0yxhxY0/edit?gid=0#gid=0) has the product backlog, iteration backlogs, and work log for each task done. The link to the sprint planning and review meeting notes devised to overview each iteration sprint is attached [here](https://docs.google.com/document/d/1AVcXsBQ32G4zNruy3SwoopeVLuxpyloBtEbVzfRSro4/edit?tab=t.0#heading=h.mw18pfbxwsdy).

## UML

The initial UML Diagram designed during proposal phase is listed under UML/initial. The current UML diagram for the Phase-1 implementation is under UML/revised.

## Debris Detection System

This repository contains the `DebrisDetector` node implementation, which uses ROS2 and OpenCV to detect and navigate towards debris in a robot's field of view. The system integrates image processing, odometry handling, and navigation for autonomous operation.

## Features
- Detect debris using color-based thresholds in the HSV color space.
- Navigate towards detected debris while avoiding obstacles.
- Publish debug images for visualization.
- Modular design with ROS2 publishers, subscribers, and services.


## Dependencies
Ensure the following dependencies are installed before building the package:

### General Dependencies
- **ROS2 (Humble/Other compatible distribution)**
- **OpenCV** (>=4.x)
- **cv_bridge**
- **image_transport**
- **rclcpp**

### Testing and Coverage Dependencies
- **lcov**
- **genhtml**

Install the required dependencies using your system package manager or ROS2 tools.


## Build Instructions

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

2. Navigate to the root of your workspace and build the package with coverage flags:
   ```bash
   cd ~/ros2_ws
   colcon build --cmake-args -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_C_FLAGS="--coverage" -DCMAKE_BUILD_TYPE=Debug
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### Testing

Run the unit and integration tests for the package:
```bash
colcon test --event-handlers console_direct+
```

If all tests pass, proceed to generate the coverage report.


### Generate Coverage Report

1. Capture the coverage data:
   ```bash
   lcov --capture --directory build --output-file coverage.info
   ```

2. Filter out irrelevant files:
   ```bash
   lcov --remove coverage.info '/opt/*' '/usr/*' '*/test/*' --output-file coverage_filtered.info
   ```

3. Generate an HTML report:
   ```bash
   genhtml coverage_filtered.info --output-directory coverage_report
   ```

4. Open the coverage report in your browser:
   ```bash
   firefox coverage_report/index.html
   ```

### Launch 

To launch the node, run the following command:
```bash
ros2 launch project_clearpath collector_robot.launch.py
```

A demo video is as follows: 
[![Watch the video](https://drive.google.com/file/d/1Fyi_XzjUzuUIOucaIdy7Z6o0Lr2dtfaS/view?usp=sharing/0.jpg)](https://drive.google.com/file/d/1Fyi_XzjUzuUIOucaIdy7Z6o0Lr2dtfaS/view?usp=sharing)
[Watch the demo video](https://drive.google.com/file/d/1Fyi_XzjUzuUIOucaIdy7Z6o0Lr2dtfaS/view?usp=sharing)

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.







