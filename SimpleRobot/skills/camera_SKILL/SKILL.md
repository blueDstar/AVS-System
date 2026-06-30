# 🤖 Project: AI Autonomous Vision System (AVS)

## 1. System Context
A real-time autonomous vehicle vision system utilizing **ROS2 Humble**, **Docker**, and **YOLO11n-seg**. It focus on lane and vehicle segmentation for high-speed decision making.

## 2. Hardware Environment
### Build Host (Development)
- **Model:** Acer Nitro 5 AN515-57
- **Spec:** 11th Gen Intel i5-11400H, 16GB RAM, RTX 3050 (4GB VRAM).
- **OS:** Ubuntu 22.04.5 LTS (GNOME 42.9).

### Deployment Target (Edge)
- **Model:** Raspberry Pi 5 (4GB RAM) + Active Cooler.
- **Storage:** 64GB MicroSD (OS via Raspberry Pi Imager).
- **Network:** Hostname: `goln-raspi5.local`, User: `goln-raspi5`.
- **Controllers:** ESP32 via **micro-ROS** for low-level actuation.

## 3. Vision & Perception Logic
### Segmentation Classes:
- **Lanes:** `main-lane` (ego), `other-lane` (adjacent).
- **Markings:** `solid-white/yellow`, `dashed-white/yellow`.
- **Objects:** `vehicle` (all detectable traffic).

## 4. System Roadmap
- [x] Hardware identification & udev setup.
- [x] Dockerization & Inference optimization.
- [ ] Decision-making logic (Twist message generation).