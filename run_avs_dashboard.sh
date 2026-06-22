#!/bin/bash

set -e

cd /home/pi/SimpleSysIDV

docker compose -f docker-compose.prod.yml up -d avs_perception video_publisher web_dashboard

echo "[INFO] Starting AVS dashboard at http://localhost:8060/dashboard/vision"

docker exec -it avs_perception_container bash -lc '
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 /workspace/ros2_ws/web_dashboard_avs/app.py
'
