Viewed perception.launch.py:75-105
Searched for "lookahead_d_max_mm"
Viewed ipm_transform_node.cpp:20-35
Edited ipm_transform_node.cpp
Edited perception.launch.py
Ran command: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
Viewed pure_pursuit_guide.md:110-157

Hoàn toàn có thể điều chỉnh giảm khoảng cách nhìn trước (look-ahead) xuống để phù hợp hơn với bản đồ nhỏ và hỗ trợ xe phản ứng nhạy hơn ở dải tốc độ cao (lên tới $2.5\text{ m/s}$).

Tôi đã thực hiện điều chỉnh cấu hình tham số look-ahead đồng bộ trên cả mã nguồn C++ và file launch như sau:

### 1. Các thông số mới được thiết lập:
*   **Thời gian xem trước (`lookahead_T_preview`):** Giảm từ $0.5\text{ s}$ xuống **$0.2\text{ s}$**.
*   **Khoảng cách nhìn trước tối thiểu (`lookahead_d_min_mm`):** Giảm từ $150\text{ mm}$ xuống **$100\text{ mm}$ ($10\text{ cm}$)**.
*   **Khoảng cách nhìn trước tối đa (`lookahead_d_max_mm`):** Giảm từ $600\text{ mm}$ xuống **$400\text{ mm}$ ($40\text{ cm}$)**.

---

### 2. Sự thay đổi thực tế khi xe chạy:
*   **Tại vận tốc tối đa $2.5\text{ m/s}$:** 
    *   Khoảng cách tính toán lý thuyết: $2500\text{ mm/s} \times 0.2\text{ s} = 500\text{ mm}$.
    *   Sau khi qua bộ kẹp giới hạn (clamp), điểm nhìn trước thực tế sẽ là **$400\text{ mm}$** ($40\text{ cm}$ trước mũi xe) thay vì bị đẩy đi quá xa.
*   **Tại vận tốc thấp $0.5\text{ m/s}$:**
    *   Khoảng cách tính toán lý thuyết: $500\text{ mm/s} \times 0.2\text{ s} = 100\text{ mm}$.
    *   Điểm nhìn trước thực tế sẽ là **$100\text{ mm}$** ($10\text{ cm}$ trước mũi xe), giúp bám cua cực sát và nhạy ở các góc cua ngặt.

---

### 3. Các file đã cập nhật trong workspace:
1.  **[ipm_transform_node.cpp](file:///home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ipm_transform_node.cpp):** Thay đổi giá trị tham số mặc định được khai báo ban đầu để áp dụng trực tiếp khi chạy Docker Compose.
2.  **[perception.launch.py](file:///home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/launch/perception.launch.py):** Cập nhật đồng bộ các tham số truyền vào khi chạy ở chế độ launch.

---

### Hướng dẫn triển khai và cập nhật lên Raspberry Pi:

Vì có sự thay đổi trong code C++ (`ipm_transform_node.cpp`), bạn cần đồng bộ và rebuild lại workspace trên Pi bằng các bước sau:

**Bước 1:** Đồng bộ code sang Pi:
```bash
rsync -avz \
  --exclude '.venv' --exclude 'build' --exclude 'install' \
  --exclude 'log' --exclude '.git' --exclude 'ncnn-src' \
  --exclude 'ncnn' --exclude 'test' \
  /home/goln/SimpleSysIDV/ pi@raspi5.local:/home/pi/SimpleSysIDV/
```

**Bước 2:** SSH vào Pi, xóa build cũ của node IPM và khởi động lại để Docker tự động compile bản mới:
```bash
cd /home/pi/SimpleSysIDV
docker compose -f docker-compose.prod.yml down

# Khởi chạy lại hệ thống (Docker tự colcon build tự động)
docker compose -f docker-compose.prod.yml up -d
```