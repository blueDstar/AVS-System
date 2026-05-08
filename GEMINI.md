#  Project: AI Autonomous Vision System (AVS)

## 1. System Context
A real-time autonomous vehicle vision system utilizing **ROS2 Humble**, **Docker**, and **YOLO11n-seg** or **DSUnet**. It focus on lane and vehicle segmentation for high-speed decision making.

## 2. Software Stack & Programming Languages

- **Primary Languages:**
  - **Python 3.10:** Main language for ROS2 high-level nodes, OpenCV image processing, and AI segmentation logic.
  - **C++ (C++17):** Used for performance-critical tasks, specifically NCNN inference and high-speed ROS2 nodes to prevent memory fragmentation on ARM64.
  - **C/C++:** For firmware development on ESP32 via the micro-ROS framework.
- **Middleware:** ROS2 (Humble Hawksbill).
- **AI Frameworks:** Ultralytics (YOLO11), DSUnet (pytorch), NCNN (Optimized for ARM CPU/Vulkan).
- **Embedded Integration:** micro-ROS (ESP32).
- **Containerization:** Docker & Docker Compose.

## 3. Hardware Environment
### Build Host (Development)
- **Model:** Acer Nitro 5 AN515-57
- **Spec:** 11th Gen Intel i5-11400H, 16GB RAM, RTX 3050 (4GB VRAM).
- **OS:** Ubuntu 22.04.5 LTS (GNOME 42.9).

### Deployment Target (Edge)
- **Model:** Raspberry Pi 5 (4GB RAM) + Active Cooler.
- **Storage:** 64GB MicroSD (OS via Raspberry Pi Imager).
- **Network:** Hostname: `goln-raspi5.local`, User: `goln-raspi5`.
- **Controllers:** ESP32 via **micro-ROS** for low-level actuation.

## 4. Development & Operation Principles
To ensure system stability and maintainability, the following guidelines are enforced:
- **Hardware Skills:** Device identification and udev mapping (See `skills/camera_SKILL/SKILL.md`).
- **Deployment Skills:** Performance-driven Docker orchestration (See `skills/docker_SKILL/SKILL.md`).
- **Coding Standards:** Surgical changes and simplicity-first approach based on Karpathy Guidelines (See `skills/karpathy-guidelines/SKILL.md`).
- **Communication Skill:** Distributed ROS2 setup for Pi-to-Laptop streaming (See `skills/data_transport_SKILL/SKILL.md`).
- **UI Standard:** Remote monitoring via Foxglove Studio on Acer Nitro 5.

## 5. Vision & Perception Logic
### Segmentation Classes:
- **Lanes:** `main-lane` (ego), `other-lane` (adjacent).
- **Markings:** `solid-white/yellow`, `dashed-white/yellow`.
- **Objects:** `vehicle` (all detectable traffic).

## 6. System Roadmap
- [x] Hardware identification & udev setup.
- [x] Dockerization & Inference optimization.
- [ ] Decision-making logic (Twist message generation).