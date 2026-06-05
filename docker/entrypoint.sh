#!/bin/bash
set -e

# Source ROS 2 Humble setup
source "/opt/ros/humble/setup.bash"

# Source workspace setup if it exists
if [ -f "/workspace/ros2_ws/install/setup.bash" ]; then
  source "/workspace/ros2_ws/install/setup.bash"
fi

exec "$@"
