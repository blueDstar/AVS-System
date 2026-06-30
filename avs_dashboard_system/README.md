# AVS Robot Control Center

A comprehensive, full-stack ROS 2 Humble control and monitoring dashboard for the AVS autonomous micro-ROS robot.

## Features

- **Real-time Telemetry:** Aggregates and streams ~15 topics (cmd_vel, odom, imu, lidar, perception) at 10Hz via WebSockets.
- **cmd_vel Multiplexer:** Safety-critical node that manages multiple control algorithms with emergency stop and connection timeout protection.
- **Process Supervisor:** Start, stop, and switch between autonomous driving controllers safely via the web UI.
- **Experiment Recorder:** Log high-frequency time-series data to CSV/JSONL and automatically generate analysis metrics (RMSE, max error, tracking ratio).
- **Gazebo Integration:** Isolate real-robot hardware and benchmark controllers seamlessly in simulation mode.
- **High-Performance UI:** React + TailwindCSS v4 with Apache ECharts for 10Hz realtime plotting and HTML5 Canvas for path visualization.

## Architecture

* **Backend (ROS 2 / Python):** 8 dedicated nodes managing specific domains (telemetry, mux, process, gazebo, api). Uses `aiohttp` in a background thread for seamless ROS-to-WebSocket bridging.
* **Frontend (React / Vite):** Custom-built Single Page Application (SPA). No dependencies on heavy UI libraries; uses raw TailwindCSS v4 with CSS variables for a dark, industrial theme.

## Installation

### Prerequisites
* ROS 2 Humble
* Python 3 (`aiohttp`, `psutil`)
* Node.js & npm (for building the frontend)

### 1. Build the ROS 2 Workspace

Ensure the package is inside your ROS 2 workspace (e.g., `~/ros2_ws/src/avs_dashboard_system`).

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select avs_dashboard_system
source install/setup.bash
```

### 2. Install Python Dependencies

```bash
pip install aiohttp psutil matplotlib
```

### 3. Build the Frontend Web App

You must have `npm` installed. Compile the React frontend so the ROS 2 `dashboard_api_control` node can serve it statically.

```bash
cd ~/ros2_ws/src/avs_dashboard_system/web
npm install
npm run build
```

*(Note: The build will output to `web/dist`. The API node will automatically look for this folder and serve `index.html` on port 8080).*

## Running the System

Use the provided scripts to start the system quickly.

```bash
# To start on the Real Robot (default port 8080):
cd ~/ros2_ws/src/avs_dashboard_system/scripts
bash start_avs_dashboard.sh

# To start in Simulation Mode (Gazebo):
bash start_avs_dashboard.sh gazebo

# Custom port:
bash start_avs_dashboard.sh real_robot 9090
```

Once running, open your browser and navigate to:
**http://localhost:8080** (or your Raspberry Pi's IP address, e.g., `http://192.168.1.100:8080`).

To stop all nodes:
```bash
bash stop_avs_dashboard.sh
```

## System Configuration

Configurations are stored in `config/*.yaml`:
- **`dashboard.yaml`**: Core settings (v_max, omega_max, timeouts).
- **`controllers.yaml`**: Define autonomous controllers and their launch commands.
- **`processes.yaml`**: Whitelist system processes (RViz, perception, micro-ROS agent).
- **`gazebo.yaml`**: Define simulation worlds.

## License

MIT License. Designed for the AVS Autonomous Robotics Project.
