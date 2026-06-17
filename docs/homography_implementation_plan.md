# Kế Hoạch Triển Khai: Extract Line — Pixel → Tọa Độ Thực (Homography + Waypoints)

## 1. Tổng Quan Kiến Trúc Đã Triển Khai

```
+------------------------------------------------------------------+
|                    LAPTOP (Acer Nitro 5)                         |
|                                                                  |
|  +--------------------------------------+                        |
|  |  Web Dashboard (http://localhost:8000)|                       |
|  |                                      |                        |
|  |  [Stream Normal] [Settings]          |                        |
|  |  [BEV Canvas]  [Calibration Modal]   |                        |
|  |                                      |                        |
|  |  Calibration Modal:                  |                        |
|  |  - Hiển thị 1 frame camera tĩnh      |                        |
|  |  - Click 4 điểm trên ảnh            |                        |
|  |  - Nhập tọa độ thực (mm)            |                        |
|  |  - POST /api/calibration             |                        |
|  |  - Tính H → lưu calibration.json    |                        |
|  +------------------+-------------------+                        |
|                     |                                            |
|  +------------------v-------------------+                        |
|  |  FastAPI Backend (main.py)           |                        |
|  |  - Tính H = getPerspectiveTransform  |                        |
|  |  - Lưu H vào config/calibration.json|                        |
|  |  - Stream /api/stream (normal view)  |                        |
|  |  - WebSocket /ws → push telemetry   |                        |
|  +--------------------------------------+                        |
+------------------------------------------------------------------+
                        |
          config/calibration.json (volume-mounted)
                        |
+-------------------v----------------------------------------------+
|                  RASPBERRY PI 5                                  |
|                                                                  |
|  +---------------+   +-----------------+   +------------------+ |
|  |video_publisher|-->|ncnn_inference   |-->|ipm_transform_node| |
|  |_node          |   |_node            |   |(C++)             | |
|  |               |   |                 |   |                  | |
|  | /camera/      |   | /avs/telemetry  |   |Đọc calibration   | |
|  |  image_raw/   |   | (JSON + polygons|   |.json             | |
|  |  compressed   |   |  pixel coords)  |   |Transform pixel   | |
|  +---------------+   +-----------------+   |-> mm (H matrix)  | |
|                                            |Extract centerline| |
|                                            |Fit polynomial    | |
|                                            |Smooth (EMA)      | |
|                                            |                  | |
|                                            | /avs/            | |
|                                            |  telemetry_      | |
|                                            |  realworld       | |
|                                            +------------------+ |
+------------------------------------------------------------------+
```

---

## 2. Hệ Tọa Độ

```
      Y (mm) — huong truoc xe (doc / longitudinal)
      ^
      |
      O----------> X (mm) — sang phai (ngang / lateral)
    (0,0) = dau xe / diem chieu camera
```

| Trục | Phạm vi | Ý nghĩa |
|------|---------|---------|
| X | -1000mm ~ +1000mm | Chiều ngang 2m |
| Y | 0mm ~ 3500mm | Tầm nhìn phía trước 3.5m |

---

## 3. Các Thành Phần Đã Triển Khai

### 3.1 Frontend — Calibration Modal (Web Dashboard)

**File:** `web_dashboard/frontend/index.html` + `app.js` + `style.css`

**Chức năng đã hoàn thành:**
- Nút **[Calibrate]** trên thanh điều khiển → mở modal calibration
- Hiển thị **1 frame tĩnh** từ camera (API `GET /api/calibration/frame`)
- Người dùng **click 4 điểm** trên ảnh → vẽ dấu chấm màu + số thứ tự theo chiều kim đồng hồ từ trái-trên
- Mỗi điểm: nhập tọa độ thực **(X_mm, Y_mm)** qua ô input (giá trị mặc định hợp lý)
- Nút **[Save Calibration]** → gửi POST `/api/calibration` → hiển thị trạng thái "CALIBRATED"
- Nút **[Clear Points]** → reset các điểm đã chọn

**Tọa độ pixel được tính theo kích thước ảnh gốc:**
```javascript
const u = Math.round((e.clientX - rect.left) * (imgW / rect.width));
const v = Math.round((e.clientY - rect.top) * (imgH / rect.height));
```

**BEV Canvas (Real-World Bird's Eye View):**
- Canvas 280×350 px, hiển thị tọa độ thực theo thời gian thực qua WebSocket
- Mapping: `toCanvasX(X) = w/2 + X * (w/2000)`, `toCanvasY(Y) = h - Y * (h/3500)`
- Vẽ grid 500mm, vehicle shape ở gốc, polygon_real_world, waypoints, polyline curve
- Màu sắc theo class (main-lane: xanh lá, other-lane: đỏ, turn-lane: tím...)

**Lưu ý:** IPM Warp stream view đã bị **xóa** — chỉ còn Normal stream view. BEV canvas là cách xem tọa độ thực.

---

### 3.2 Backend — API Calibration (FastAPI)

**File:** `web_dashboard/backend/main.py`

**API đã hoàn thành:**

```
POST /api/calibration
Body: {
    "pixel_points": [[u1,v1], [u2,v2], [u3,v3], [u4,v4]],
    "world_points": [[x1,y1], [x2,y2], [x3,y3], [x4,y4]],
    "image_size": [640, 480]
}
Response: {
    "status": "success",
    "homography_matrix": [[h11,h12,h13],[h21,h22,h23],[h31,h32,h33]]
}

GET /api/calibration          -> Trả về calibration.json hiện tại
GET /api/calibration/frame    -> Trả về 1 frame JPEG tĩnh để calibrate
```

**Logic tính H:**
```python
src = np.float32(pixel_points)
dst = np.float32(world_points)
H = cv2.getPerspectiveTransform(src, dst)  # 3x3 matrix
```

---

### 3.3 Config File — `config/calibration.json`

```json
{
    "homography_matrix": [
        [h11, h12, h13],
        [h21, h22, h23],
        [h31, h32, h33]
    ],
    "pixel_points": [[u1,v1], [u2,v2], [u3,v3], [u4,v4]],
    "world_points": [[x1,y1], [x2,y2], [x3,y3], [x4,y4]],
    "image_size": [640, 480],
    "calibrated_at": "2026-06-17T..."
}
```

File nằm trong folder `config/` được volume-mount trong Docker → `ipm_transform_node` trên Pi đọc trực tiếp. Node tự động **reload** khi file thay đổi (so sánh `last_write_time`).

---

### 3.4 ROS2 Node — `ipm_transform_node` (C++)

**File:** `ros2_ws/src/avs_perception/src/ipm_transform_node.cpp`

**Subscribe:** `/avs/telemetry` (std_msgs/String — JSON chứa pixel polygons từ ncnn_inference_node)

**Publish:** `/avs/telemetry_realworld` (std_msgs/String — JSON bổ sung real-world coords + waypoints + polynomial)

#### Pipeline xử lý mỗi frame:

**Bước 1 — Load & auto-reload calibration:**
```cpp
void check_calibration_update() {
    auto current_write_time = filesystem::last_write_time(calibration_file_path_);
    if (!has_calibration_ || current_write_time != last_write_time_)
        load_calibration();
}
```

**Bước 2 — Transform pixel polygon → real-world (mm):**
```cpp
// Với mỗi điểm (u, v) trong polygon:
double w = H_[2][0]*u + H_[2][1]*v + H_[2][2];
if (abs(w) > 1e-6) {
    double X = (H_[0][0]*u + H_[0][1]*v + H_[0][2]) / w;
    double Y = (H_[1][0]*u + H_[1][1]*v + H_[1][2]) / w;
    // làm tròn 0.1mm: round(X*10)/10
}
```
Kết quả: `obj["polygons_real_world"]` — thêm vào JSON telemetry.

**Bước 3 — Extract centerline waypoints:**

Chỉ xử lý lane labels: `main-lane (3)`, `other-lane (4)`, `turn-lane (10)`.

| Label | Phương pháp | Sweep axis |
|-------|------------|-----------|
| 3 (main-lane) | `extract_centerline_waypoints_y()` | Y-sweep (bước 100mm) |
| 4 (other-lane) | `extract_centerline_waypoints_y()` | Y-sweep (bước 100mm) |
| 10 (turn-lane) | `extract_centerline_waypoints_x()` | X-sweep (bước 100mm) |

```cpp
// Y-sweep (cho main/other-lane):
for (double y = start_y; y <= max_y; y += step_mm) {
    // Ray-casting: tìm X giao của đường y = const với cạnh polygon
    X_center = (X_min_intersection + X_max_intersection) / 2.0;
    waypoints.push_back({X_center, y});
}
```

**Bước 4 — Dedup & Spatial Smoothing:**
```cpp
// Dedup: merge waypoints có cùng t (trung bình)
// 3-point moving average:
smoothed[i].s = (raw[i-1].s + raw[i].s + raw[i+1].s) / 3.0;
```

**Bước 5 — Fit polynomial bậc 3 (SVD):**
```cpp
// N >= 4: cubic polynomial
cv::Mat A(n, 4, CV_64F);  // [t^3, t^2, t, 1]
cv::Mat B(n, 1, CV_64F);  // [s values]
cv::solve(A, B, C, cv::DECOMP_SVD);
// coeffs = [a3, a2, a1, a0]

// N < 4: linear fallback
cv::Mat A(n, 2, CV_64F);  // [t, 1]
```

**Bước 6 — Temporal Smoothing (EMA, alpha=0.25):**
```cpp
// Áp dụng cho tất cả label đã từng xuất hiện:
double alpha = 0.25;
for (size_t c = 0; c < 4; ++c)
    coeffs[c] = alpha * coeffs[c] + (1.0 - alpha) * prev_coeffs_[label][c];
```

**Bước 7 — Regenerate smooth waypoints từ polynomial đã smooth:**
```cpp
// main-lane / other-lane:
for (double y_val = y_min; y_val <= y_max; y_val += 100.0) {
    double x_val = a3*pow(y,3) + a2*pow(y,2) + a1*y + a0;
    smooth_wps.push_back({x_val, y_val});
}

// turn-lane:
for (double x_val = x_min; x_val <= x_max; x_val += 100.0) {
    double y_val = a3*pow(x,3) + a2*pow(x,2) + a1*x + a0;
    smooth_wps.push_back({x_val, y_val});
}
```

**Bước 8 — Tính control metrics (smooth EMA):**

| Metric | Công thức | Label áp dụng |
|--------|-----------|---------------|
| `lateral_offset_mm` | `a0` của x(y) | main-lane, other-lane |
| `longitudinal_offset_mm` | `a0` của y(x) | turn-lane |
| `heading_angle_rad` | `atan(a1)` | tất cả lane |
| `curvature_inv_mm` | `2·a2` | tất cả lane |

**Output JSON mẫu (per object):**
```json
{
  "label": 3,
  "polygons_real_world": [[[X1,Y1], [X2,Y2], ...]],
  "waypoints": [[X_c1, Y1], [X_c2, Y2], ...],
  "polynomial": {"a3": 1e-7, "a2": -0.0003, "a1": 0.015, "a0": 85.2},
  "lateral_offset_mm": 85.2,
  "longitudinal_offset_mm": 0.0,
  "heading_angle_rad": 0.015,
  "curvature_inv_mm": -0.0006
}
```

---

### 3.5 CMakeLists.txt

```cmake
# IPM Transform Node
add_executable(ipm_transform_node src/ipm_transform_node.cpp)
target_link_libraries(ipm_transform_node ${OpenCV_LIBRARIES})
ament_target_dependencies(ipm_transform_node
  rclcpp std_msgs nlohmann_json_vendor)
install(TARGETS ipm_transform_node
  DESTINATION lib/${PROJECT_NAME})
```

---

### 3.6 Docker Compose

Node `ipm_transform_node` được khởi chạy song song trong `avs_perception_container`:

```yaml
command: bash -c "... colcon build --symlink-install && ... &&
  (ros2 run avs_perception ncnn_inference_node &
   ros2 run avs_perception ipm_transform_node)"
```

---

## 4. Luồng Dữ Liệu Đầy Đủ

```
[Camera USB] --> video_publisher_node
                     | /camera/image_raw/compressed
                     v
               ncnn_inference_node (NCNN YOLO)
                     | /avs/telemetry
                     | JSON: {objects: [{label, polygons(pixel), ...}]}
                     v
               ipm_transform_node (C++)
                     |
                     |-- Load H from calibration.json (auto-reload)
                     |-- Transform polygon pixel -> real-world mm
                     |-- Extract centerline waypoints (Y-sweep / X-sweep)
                     |-- Spatial smooth (3-point MA)
                     |-- Fit polynomial bac 3 (SVD)
                     |-- Temporal smooth (EMA alpha=0.25)
                     |-- Regenerate smooth waypoints
                     |-- Compute lateral_offset, heading, curvature
                     |
                     | /avs/telemetry_realworld
                     | JSON: {objects: [{..., polygons_real_world,
                     |                      waypoints, polynomial,
                     |                      lateral_offset_mm, ...}]}
                     v
               web_bridge_node (FastAPI)
                     | WebSocket /ws
                     v
               Browser BEV Canvas (JavaScript)
                     |-- Draw polygons_real_world
                     |-- Draw waypoints
                     |-- Draw smooth polyline curve
```

---

## 5. Quy Trình Sử Dụng

### Calibration (1 lần duy nhất):
```
1. Đặt vật tham chiếu (kích thước đã biết, đặt trước đầu xe)
2. Mở web dashboard: http://<hostname>:8000
3. Bấm nút [Calibrate] → modal mở, hiển thị frame camera
4. Click 4 góc của vật tham chiếu theo thứ tự: TL→TR→BR→BL
5. Nhập tọa độ thực (mm) cho 4 điểm (gốc 0,0 = đầu xe)
6. Bấm [Save Calibration]
7. H được lưu vào config/calibration.json
8. ipm_transform_node tự động reload (phát hiện file thay đổi)
9. BEV canvas chuyển badge sang "CALIBRATED"
```

### Vận hành (liên tục):
```
1. Camera stream → video_publisher_node
2. YOLO seg → ncnn_inference_node → /avs/telemetry (pixel)
3. ipm_transform_node → /avs/telemetry_realworld (real-world mm + waypoints)
4. web_bridge_node → WebSocket → BEV canvas hiển thị real-time
5. [TODO] Controller đọc lateral_offset_mm, heading_angle_rad, curvature_inv_mm
         → tính Twist → điều khiển xe
```

---

## 6. Trạng Thái Triển Khai

| Phase | Công việc | Trạng thái |
|-------|----------|-----------|
| **Phase 1** | Backend API calibration + config file | ✅ Hoàn thành |
| **Phase 2** | Frontend UI calibration modal + BEV canvas | ✅ Hoàn thành |
| **Phase 3** | `ipm_transform_node` C++ (core logic) | ✅ Hoàn thành |
| **Phase 4** | Docker Compose integration + end-to-end test | ✅ Hoàn thành |
| **Phase 5** | Kết nối output với bộ điều khiển PID/Twist | ⬜ Chưa triển khai |

---

## 7. Xác Minh & Kiểm Thử

| Bước kiểm tra | Cách thực hiện |
|---------------|----------------|
| Calibration API | POST 4 cặp điểm → kiểm tra `calibration.json` có H hợp lệ |
| Auto-reload | Lưu lại calibration mới → ipm_transform_node log "Reloading..." |
| Transform chính xác | Đặt vật ở vị trí đã biết, kiểm tra `polygons_real_world` output |
| Polynomial fit | BEV canvas hiển thị đường cong mượt trùng với vị trí làn |
| Temporal smooth | Quan sát BEV canvas — đường cong không bị giật khi detection thay đổi |
| Control metrics | `lateral_offset_mm` thay đổi khi xe lệch khỏi làn |
