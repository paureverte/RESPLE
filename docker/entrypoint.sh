#!/bin/bash
set -e

# Source ROS
source /opt/ros/jazzy/setup.bash

# Hand off to the requested command (default: bash)
exec "$@"
