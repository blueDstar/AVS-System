#!/bin/bash
# =============================================================================
# AVS Robot Control Center — Start Script
# File: scripts/start_avs_dashboard.sh
#
# Usage:
#   ./start_avs_dashboard.sh [real_robot|gazebo] [port]
#   ./start_avs_dashboard.sh                    # → real_robot, port 8080
#   ./start_avs_dashboard.sh gazebo 9090        # → gazebo mode, port 9090
# =============================================================================

set -e

RUNTIME="${1:-real_robot}"
PORT="${2:-8080}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"

# ---- Colors ----
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════╗"
echo "║         AVS Robot Control Center                ║"
echo "║     ROS 2 Humble Dashboard Backend              ║"
echo "╚══════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${GREEN}Runtime  : ${RUNTIME}${NC}"
echo -e "${GREEN}Port     : ${PORT}${NC}"
echo -e "${GREEN}DOMAIN_ID: ${ROS_DOMAIN_ID}${NC}"
echo ""

# ---- Validate runtime ----
if [[ "$RUNTIME" != "real_robot" && "$RUNTIME" != "gazebo" ]]; then
    echo -e "${RED}Error: runtime must be 'real_robot' or 'gazebo'${NC}"
    exit 1
fi

# ---- Source ROS 2 ----
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✅ ROS 2 Humble sourced${NC}"
else
    echo -e "${RED}ERROR: ROS 2 Humble not found at /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

# ---- Source workspace (try common locations) ----
WS_FOUND=0
for WS_PATH in \
    "${HOME}/ros2_ws/install/setup.bash" \
    "/workspace/ros2_ws/install/setup.bash" \
    "${HOME}/avs_ws/install/setup.bash" \
    "${HOME}/colcon_ws/install/setup.bash"
do
    if [[ -f "$WS_PATH" ]]; then
        source "$WS_PATH"
        echo -e "${GREEN}✅ Workspace sourced: ${WS_PATH}${NC}"
        WS_FOUND=1
        break
    fi
done

if [[ $WS_FOUND -eq 0 ]]; then
    echo -e "${YELLOW}⚠ Workspace install not found. Try building first:${NC}"
    echo -e "${YELLOW}  colcon build --symlink-install --packages-select avs_dashboard_system${NC}"
fi

# ---- Set environment ----
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
echo -e "${GREEN}✅ ROS_DOMAIN_ID=${ROS_DOMAIN_ID}${NC}"

# ---- Check Python deps ----
echo -e "\n${CYAN}Checking Python dependencies...${NC}"
MISSING_DEPS=()
python3 -c "import aiohttp" 2>/dev/null || MISSING_DEPS+=("aiohttp")
python3 -c "import psutil" 2>/dev/null || MISSING_DEPS+=("psutil")

if [[ ${#MISSING_DEPS[@]} -gt 0 ]]; then
    echo -e "${YELLOW}⚠ Missing Python packages: ${MISSING_DEPS[*]}${NC}"
    echo -e "${YELLOW}Installing...${NC}"
    pip install "${MISSING_DEPS[@]}" --quiet
fi

# ---- Launch dashboard ----
echo ""
echo -e "${CYAN}Starting AVS Dashboard System...${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  🌐 Dashboard URL: ${CYAN}http://localhost:${PORT}${NC}"

# Get local IP for remote access
LOCAL_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
if [[ -n "$LOCAL_IP" ]]; then
    echo -e "  🌐 Remote access: ${CYAN}http://${LOCAL_IP}:${PORT}${NC}"
fi

echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

exec ros2 launch avs_dashboard_system avs_dashboard.launch.py \
    runtime:="${RUNTIME}" \
    web_port:="${PORT}" \
    ros_domain_id:="${ROS_DOMAIN_ID}"
