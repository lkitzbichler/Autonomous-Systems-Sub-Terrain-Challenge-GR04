#!/bin/bash
set -e

echo "Installing missing ROS2 Jazzy dependencies..."

# Install tf2_geometry_msgs and other required packages
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-octomap-server \
  ros-jazzy-pcl-ros \
  ros-jazzy-depth-image-proc

echo "✅ Dependencies installed successfully"
