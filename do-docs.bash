#!/bin/bash

set -e  # Exit on error

# Print system info
echo "System information:"
uname -a
lsb_release -a

# Set up ROS workspace
echo "Setting up ROS workspace..."
source /opt/ros/humble/setup.bash
source ~/your_workspace/install/setup.bash  # Adjust path to your workspace

# Generate documentation using Doxygen
echo "Generating documentation using Doxygen..."
doxygen Doxyfile  # Ensure you have a Doxyfile in your project root for configuration

# Optionally, you can also generate markdown, pdf, or other formats with pandoc
echo "Generating markdown documentation..."
pandoc README.md -o documentation.pdf  # Example of converting markdown to PDF

echo "Documentation generation complete!"
