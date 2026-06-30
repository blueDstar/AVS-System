#!/bin/bash
# =============================================================================
# AVS Dashboard System — Docker Container Entrypoint
# Builds the avs_dashboard_system ROS 2 package inside the container,
# then launches all dashboard nodes (API on port 8080).
# =============================================================================
set -e

echo "[AVS Dashboard] Waiting for ROS 2 workspace build to complete..."
# Wait until the main avs_perception build is done (so we have a valid workspace)
while [ ! -f /workspace/ros2_ws/install/setup.bash ]; do
    sleep 2
    echo "[AVS Dashboard] Still waiting for /workspace/ros2_ws/install/setup.bash ..."
done
echo "[AVS Dashboard] Main workspace ready."

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash

# Install Python dependencies that may not be in the base image
pip3 install --quiet --no-cache-dir aiohttp psutil pyyaml matplotlib 2>/dev/null || true

# Build avs_dashboard_system as a standalone colcon package
echo "[AVS Dashboard] Building avs_dashboard_system..."
mkdir -p /workspace/avs_dashboard_ws/src
ln -sf /workspace/avs_dashboard_system /workspace/avs_dashboard_ws/src/avs_dashboard_system
cd /workspace/avs_dashboard_ws
colcon build --symlink-install --packages-select avs_dashboard_system 2>&1
source install/setup.bash
echo "[AVS Dashboard] Build complete!"

# Launch all dashboard nodes
echo "[AVS Dashboard] Starting dashboard nodes on port 8080..."
ros2 launch avs_dashboard_system avs_dashboard.launch.py \
    runtime:=real_robot \
    web_port:=8080 \
    web_host:=0.0.0.0
