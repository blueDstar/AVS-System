#!/bin/bash
# =============================================================================
# AVS Robot Control Center — Full Installation & Build Script
# File: scripts/install_and_build.sh
#
# Run this ONCE to:
#   1. Install Node.js (via nvm, no sudo needed)
#   2. Install Python dependencies
#   3. Build the React frontend
#   4. Build the ROS 2 colcon package
#
# Usage: bash scripts/install_and_build.sh
# =============================================================================

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; YELLOW='\033[1;33m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WEB_DIR="$PKG_DIR/web"

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════╗"
echo "║    AVS Robot Control Center — Install & Build   ║"
echo "╚══════════════════════════════════════════════════╝"
echo -e "${NC}"

# ============================================================================
# 1. Node.js via nvm (no root needed)
# ============================================================================
echo -e "\n${CYAN}[1/4] Checking Node.js...${NC}"

if command -v node &>/dev/null; then
    echo -e "${GREEN}✅ Node.js found: $(node --version)${NC}"
else
    echo -e "${YELLOW}⚠ Node.js not found. Installing via nvm...${NC}"
    
    # Install nvm
    export NVM_DIR="$HOME/.nvm"
    if [ ! -d "$NVM_DIR" ]; then
        curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
    fi
    
    # Load nvm
    source "$NVM_DIR/nvm.sh" 2>/dev/null || true
    
    # Install Node 20 LTS
    nvm install 20
    nvm use 20
    
    echo -e "${GREEN}✅ Node.js installed: $(node --version)${NC}"
fi

# Ensure npm is available
if ! command -v npm &>/dev/null; then
    echo -e "${RED}ERROR: npm still not found. Please install Node.js manually.${NC}"
    echo -e "${YELLOW}Suggestion: curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash${NC}"
    exit 1
fi
echo -e "${GREEN}✅ npm: $(npm --version)${NC}"

# ============================================================================
# 2. Python dependencies
# ============================================================================
echo -e "\n${CYAN}[2/4] Installing Python dependencies...${NC}"

pip install --quiet --upgrade \
    aiohttp \
    psutil \
    matplotlib \
    numpy

echo -e "${GREEN}✅ Python deps: aiohttp, psutil, matplotlib, numpy${NC}"

# ============================================================================
# 3. Frontend build
# ============================================================================
echo -e "\n${CYAN}[3/4] Building React frontend...${NC}"
cd "$WEB_DIR"

echo "  → npm install"
npm install --silent

echo "  → npm run build"
npm run build

echo -e "${GREEN}✅ Frontend built to: ${WEB_DIR}/dist${NC}"

# ============================================================================
# 4. ROS 2 colcon build
# ============================================================================
echo -e "\n${CYAN}[4/4] Building ROS 2 package...${NC}"

# Find ros2 workspace
ROS_WS=""
for ws in "$HOME/ros2_ws" "/workspace/ros2_ws" "$HOME/avs_ws" "$HOME/colcon_ws"; do
    if [ -f "$ws/src/avs_dashboard_system/package.xml" ] 2>/dev/null || \
       [ -f "$ws/src/avs_dashboard_system/setup.py" ] 2>/dev/null; then
        ROS_WS="$ws"
        break
    fi
done

if [ -z "$ROS_WS" ]; then
    echo -e "${YELLOW}⚠ No ROS 2 workspace with avs_dashboard_system found.${NC}"
    echo -e "${YELLOW}  Manual step: copy/symlink the package and run colcon build:${NC}"
    echo -e "  cd ~/ros2_ws"
    echo -e "  ln -sf $PKG_DIR src/avs_dashboard_system"
    echo -e "  colcon build --symlink-install --packages-select avs_dashboard_system"
else
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    cd "$ROS_WS"
    colcon build \
        --symlink-install \
        --packages-select avs_dashboard_system \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo -e "${GREEN}✅ ROS 2 package built in: $ROS_WS${NC}"
fi

# ============================================================================
# Done
# ============================================================================
echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}✅  Installation & Build Complete!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  Next step: ${CYAN}bash scripts/start_avs_dashboard.sh${NC}"
echo -e "  Then open: ${CYAN}http://localhost:8080${NC}"
echo ""
