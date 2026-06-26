# AVS-System

## Tổng quan dự án

AVS-System là một hệ thống nhận diện thị giác cho xe tự hành, được xây dựng trên nền tảng **ROS2 Humble**, **Docker**, và **NCNN**. Mục tiêu chính là chạy inference segmentation lane/vehicle trên CPU nhanh và ổn định, tối ưu cho **Raspberry Pi 5**.

Hệ thống bao gồm:
- Docker container orchestration cho ROS2, camera capture và dashboard.
- Mạng nơ-ron đã xuất sang NCNN/INT8 để chạy inference ARM CPU.
- Một workspace ROS2 chứa các package `avs_perception` và `avs_controlsystem`.
- Web dashboard để giám sát video và trạng thái hệ thống.
- Các script hỗ trợ deploy lên Raspberry Pi 5 và build image ARM64.

---

## Cấu trúc thư mục chính

- `docker/`
  - `Dockerfile` — base image và môi trường runtime cho ROS2, OpenCV, NCNN, FastAPI.
  - `entrypoint.sh` — entrypoint chỉ source ROS2 Humble base.
- `docker-compose.yml` — compose file dev để build image và khởi chạy các container trên máy host.
- `docker-compose.prod.yml` — compose file production cho Pi 5, sử dụng image ARM64 và host networking.
- `models/` — chứa mô hình NCNN đã xuất sẵn và metadata.
- `ncnn/` — cài đặt NCNN local (shared lib, CMake config) để ROS2 package liên kết.
- `ros2_ws/` — ROS2 workspace với source packages và các artefact build.
- `web_dashboard/` — frontend + backend dashboard giám sát.
- `config/` — cấu hình runtime, ví dụ `config.json`.
- `scripts/` — hỗ trợ deploy, export model NCNN.
- `docs/` — tài liệu triển khai, overview, kế hoạch kỹ thuật.
- `run_avs_dashboard.sh` — script khởi động dashboard nhanh.
- `scripts/deploy_to_pi.sh` — script rsync đồng bộ mã nguồn sang Raspberry Pi.

---

## Thành phần chính

### 1. Docker & deployment

- `docker-compose.yml` khởi chạy:
  - `avs_perception` container để build và giới thiệu ROS2 nodes.
  - `video_publisher` container để chờ build xong rồi chạy node camera.
  - `web_dashboard` container để host dashboard.
- `docker-compose.prod.yml` tương tự nhưng dùng image ARM64 sẵn và cấu hình production.
- `run_avs_dashboard.sh` bắt đầu các dịch vụ `avs_perception`, `video_publisher`, `web_dashboard`.
- `scripts/deploy_to_pi.sh` đồng bộ mã nguồn từ laptop sang Raspberry Pi, loại trừ các thư mục build và dữ liệu nặng.

### 2. Web dashboard

- `web_dashboard/backend/main.py` chạy server backend (FastAPI / Uvicorn) để hiển thị telemetry và video.
- `web_dashboard/frontend/` chứa JS/CSS/HTML hiển thị giao diện giám sát.

### 3. NCNN và model

- `ncnn/lib/cmake/ncnn/` chứa cấu hình CMake và import target để package ROS2 tìm `ncnn`.
- `models/` chứa nhiều phiên bản mô hình NCNN, bao gồm `best_ncnn_model/`, `yolo26-best_ncnn_model/`, `yolo26-best_ncnn_model_int8/`.
- `scripts/export_ncnn.py` xuất model PyTorch/Ultralytics sang định dạng NCNN.

### 4. ROS2 workspace

- `ros2_ws/src/avs_perception/` — package ROS2 cho perception và inference.
- `ros2_ws/src/avs_controlsystem/` — package cho control và tính toán hành vi điều khiển.
- `ros2_ws/src/avs_perception/CMakeLists.txt` và `package.xml` định nghĩa build với ROS2, OpenCV, cv_bridge, image_transport và `ncnn`.

### 5. Tài liệu và kỹ năng

- `GEMINI.md` — mô tả mục tiêu hệ thống, stack công nghệ, phần cứng, và chiến lược CPU-centric.
- `docs/OVERVIEW.md` — hướng dẫn từng bước setup hệ thống YOLO26-NCNN trên ROS2 Humble.
- `docs/deployment_guide.md` — hướng dẫn triển khai và vận hành trên Raspberry Pi 5.
- `skills/` — chứa các hướng dẫn chuyên sâu cho Docker, camera, data transport, Karpathy guidelines, labeling.

---

## Hướng dẫn nhanh chạy hệ thống

1. Build image và chạy Docker Compose dev:

```bash
cd /home/bluedstar/AVS-System
sudo docker compose up -d --build
```

2. Hoặc chạy production trên Pi 5:

```bash
sudo docker compose -f docker-compose.prod.yml up -d
```

3. Khởi chạy web dashboard:

```bash
./run_avs_dashboard.sh
```

4. Đồng bộ lên Pi bằng:

```bash
./scripts/deploy_to_pi.sh
```

---

## Ghi chú hiện trạng

- Dự án đã có cấu hình Docker và hệ thống tài liệu khá đầy đủ.
- `README.md` ban đầu chỉ chứa tiêu đề và đã được mở rộng thành tài liệu tổng quan.
- `docker-compose.yml` và `docker-compose.prod.yml` đặt các container chạy cùng `network_mode: host`, cho phép ROS2 DDS discovery.
- `ncnn/` đã cài đặt NCNN cục bộ với `VULKAN=OFF`, `SHARED_LIB=ON`, phù hợp cho CPU-only inference trên ARM.
- Hệ thống có hướng rõ ràng cho deployment Pi 5 và cho phép thay đổi bộ điều khiển giữa Pure Pursuit / PD.

---

## Tài nguyên tham khảo

- `GEMINI.md`
- `docs/deployment_guide.md`
- `docs/OVERVIEW.md`
- `skills/docker_SKILL/SKILL.md`
- `skills/camera_SKILL/SKILL.md`
- `run_avs_dashboard.sh`
- `scripts/deploy_to_pi.sh`
