# Hướng dẫn triển khai hệ thống lên Raspberry Pi 5 (Production Target)

Tài liệu này hướng dẫn từng bước để bạn đưa toàn bộ hệ thống xử lý ảnh thông minh (AVS) và bộ điều khiển (Pure Pursuit / PD) từ máy tính Laptop phát triển lên **Raspberry Pi 5** chạy trên xe thực tế.

---

## 1. Chuẩn bị tài nguyên trên Laptop

Đảm bảo thư mục dự án trên Laptop phát triển có đầy đủ:
1.  **File Docker Image ARM64:** `avs_perception_arm64.tar` (đã được lưu tại thư mục gốc `/home/goln/SimpleSysIDV/`).
2.  **Mã nguồn ROS2:** Thư mục `ros2_ws/`.
3.  **Model INT8:** Thư mục `models/yolo26-best_ncnn_model_int8/`.
4.  **Cấu hình Docker:** `docker-compose.prod.yml`.

---

## 2. Sao chép dự án sang Raspberry Pi 5

Bật Raspberry Pi 5 lên, kết nối cùng một mạng Wi-Fi với Laptop. 

### Bước 2.1: Lấy địa chỉ IP của Raspberry Pi 5
Bạn có thể quét IP hoặc ping tên máy trên Raspberry Pi để lấy IP chính xác:
```bash
hostname -I
```
*(Giả sử địa chỉ IP của Raspberry Pi 5 là `192.168.1.100`, user đăng nhập là `pi`)*

### Bước 2.2: Sao chép dự án bằng `rsync`
Chạy lệnh sau trên Terminal của Laptop (loại bỏ hoàn toàn các thư mục build tạm thời, môi trường ảo Python `.venv`, dữ liệu Git `.git` và các thư mục không cần thiết trên Pi như `test/`, `docs/`, `skills/` để quá trình sao chép cực kỳ nhanh và nhẹ):

```bash
rsync -avz \
  --exclude="ros2_ws/build" \
  --exclude="ros2_ws/install" \
  --exclude="ros2_ws/log" \
  --exclude="test" \
  --exclude="docs" \
  --exclude="skills" \
  --exclude=".venv" \
  --exclude=".git" \
  /home/goln/SimpleSysIDV/ pi@192.168.1.100:~/SimpleSysIDV/
```

---

## 3. Cài đặt và thiết lập trên Raspberry Pi 5

Đăng nhập vào Raspberry Pi 5 thông qua SSH:
```bash
ssh pi@192.168.1.100
cd ~/SimpleSysIDV
```

### Bước 3.1: Nạp Docker Image ARM64 vào Pi 5
Mô hình Docker Image cho kiến trúc ARM64 của Pi 5 đã được đóng gói sẵn trong file `.tar`. Bạn chỉ cần load nó vào Docker của Pi 5:

```bash
docker load -i avs_perception_arm64.tar
```
*Kiểm tra danh sách image bằng lệnh `docker images`, bạn sẽ thấy `avs_perception:arm64`.*

### Bước 3.2: Thiết lập quyền truy cập USB Camera
Vì container cần đọc hình ảnh trực tiếp từ USB camera thông qua `/dev/video*`, hãy đảm bảo user của Pi có quyền truy cập camera:
```bash
sudo usermod -aG video $USER
```

---

## 4. Chạy hệ thống trên Raspberry Pi 5

Hệ thống được khởi chạy dễ dàng thông qua **Docker Compose**:

### Bước 4.1: Khởi chạy bộ xử lý AI và Controller
Chạy file compose production:
```bash
docker compose -f docker-compose.prod.yml up -d
```
*   **Container `avs_perception_container`** sẽ tự động:
    1. Clean thư mục build cũ.
    2. Build biên dịch code C++ trực tiếp trên kiến trúc ARM64 của Pi 5 (tối ưu hóa phần cứng).
    3. Tự động chạy đồng thời 4 node: `ncnn_inference_node` (nhận diện INT8), `ipm_transform_node` (biến đổi tọa độ), `control_node` (tính sai số) và bộ điều khiển `pure_pursuit_node` (phát lệnh `/cmd_vel`).

*   **Container `video_publisher_container`**: Đọc luồng camera USB thực tế và publish lên ROS2.
*   **Container `web_dashboard_container`**: Khởi chạy dashboard giám sát qua trình duyệt tại địa chỉ `http://192.168.1.100:8000`.

### Bước 4.2: Tùy chỉnh bộ điều khiển (Pure Pursuit vs PD)
Nếu bạn muốn sử dụng bộ điều khiển **PD** (`cmdvel_from_control_error_node`) thay vì **Pure Pursuit** mặc định, chỉ cần mở file `docker-compose.prod.yml` trên Pi và thay đổi đoạn cuối lệnh `command` tại dòng 20:

*   **Đổi từ:** `& ros2 run avs_perception pure_pursuit_node`
*   **Thành:** `& ros2 run avs_perception cmdvel_from_control_error_node`

Sau đó restart container:
```bash
docker compose -f docker-compose.prod.yml up -d --force-recreate
```

---

## 5. Khởi chạy micro-ROS Agent kết nối với ESP32

Để các lệnh điều khiển `/cmd_vel` từ Pi 5 truyền được xuống ESP32 điều khiển động cơ, bạn cần chạy một container micro-ROS agent kết nối qua cổng Serial UART/USB:

Chạy lệnh sau trên terminal của Pi 5:
```bash
docker run -it --rm \
  --net=host \
  --privileged \
  -v /dev:/dev \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyUSB0 -b 921600
```
*(Thay thế `/dev/ttyUSB0` bằng cổng kết nối thực tế với ESP32 của bạn).*

---

## 6. Giám sát hệ thống từ xa

Từ Laptop hoặc điện thoại kết nối chung mạng Wi-Fi với xe, mở trình duyệt và truy cập:
```
http://192.168.1.100:8000
```
Bạn sẽ quan sát được:
1. Luồng video nhận diện vạch làn đường/vật cản thời gian thực.
2. Các thông số sai số làn, vận tốc góc, vận tốc dài xe đang phát xuống ESP32.
3. Giao diện trực quan hóa quỹ đạo di chuyển của xe.
