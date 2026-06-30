Cách đo FPS và Latency (độ trễ) hiện tại trong hệ thống được thực hiện ngay bên trong file nguồn [ncnn_inference_node.cpp](file:///home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp) (từ dòng 60 đến 193). 

Dưới đây là chi tiết về **cách đo hiện tại**, đánh giá **mức độ chính xác**, và **các điểm hạn chế** của phương pháp này.

---

### 1. Cách đo hiện tại trong mã nguồn
Hệ thống sử dụng thư viện chuẩn C++ `std::chrono::high_resolution_clock` để bấm giờ các phân đoạn xử lý của hàm callback `image_callback` khi nhận được một khung hình:

* **Mốc bắt đầu (`start_time`)**: Lấy ngay khi bắt đầu hàm callback `image_callback`.
* **Độ trễ chuyển đổi ảnh (`bridge_latency`)**: Đo thời gian chuyển đổi từ định dạng ảnh của ROS2 (`sensor_msgs/msg/Image`) sang ảnh OpenCV `cv::Mat` thông qua `cv_bridge`.
* **Độ trễ suy luận (`inference_latency`)**: Đo thời gian chạy mô hình nhận diện thực tế bằng thư viện NCNN (`yolo_->detect()`).
* **Độ trễ hậu xử lý (`post_latency`)**: Đo thời gian tìm đường bao contour (`cv::findContours()`) từ mặt nạ phân vùng và tuần tự hóa kết quả thành chuỗi JSON.
* **Tổng độ trễ xử lý (`full_latency`)**: Tính bằng:
  $$\text{full\_latency} = t_{\text{hoàn thành hậu xử lý}} - t_{\text{bắt đầu callback}}$$
* **Chỉ số FPS**: Được tính toán tức thời (instantaneous) theo công thức:
  $$\text{FPS} = \frac{1000.0}{\text{full\_latency (ms)}}$$
* **Thông tin ghi log (Debug)**: Node còn tính thêm `total_latency_with_publish` (bao gồm cả thời gian thực hiện hàm `telemetry_pub_->publish()`) và ghi ra dưới dạng log debug để giám sát.

---

### 2. Đánh giá: Cách đo hiện tại có đang ĐÚNG không?

Cách đo hiện tại **ĐÚNG về mặt cục bộ (Local Profiling)** để tối ưu hóa thuật toán, nhưng **CHƯA ĐỦ và CHƯA HOÀN TOÀN CHÍNH XÁC về mặt hệ thống (End-to-End System Latency & Actual Throughput)**. 

Cụ thể, phương pháp này có 3 điểm hạn chế quan trọng sau:

#### Hạn chế 1: Đo FPS tức thời thay vì FPS thực tế (Instantaneous vs. Actual Throughput)
* **Vấn đề**: Cách tính $\text{FPS} = 1000 / \text{latency}$ chỉ phản ánh *khả năng xử lý lý thuyết của một khung hình đơn lẻ*.
* **Thực tế**: Trong một hệ thống ROS2 chạy trên Raspberry Pi 5, FPS thực tế truyền đến các node điều khiển phụ thuộc vào việc lập lịch của CPU (CPU scheduling), hàng đợi tin nhắn (message queue) và việc rớt khung hình (frame drop). 
* **Giải pháp chuẩn hơn**: Nên đo FPS thực tế bằng cách đếm số lượng tin nhắn được publish thành công trong một khoảng thời gian trượt (sliding window, ví dụ 1 giây) hoặc đo khoảng thời gian giữa 2 lần callback liên tiếp ($t_{\text{hiện tại}} - t_{\text{trước đó}}$).

#### Hạn chế 2: Chưa tính độ trễ truyền dẫn từ Camera (Transport Latency)
* **Vấn đề**: `full_latency` chỉ bắt đầu đo từ khi hàm callback chạy. Nó bỏ qua thời gian truyền ảnh từ driver camera qua mạng/bộ nhớ dùng chung ROS2 tới node xử lý.
* **Thực tế**: Đối với xe tự hành chạy tốc độ cao, độ trễ từ lúc mắt camera thực tế chụp ảnh (photon ghi nhận) cho đến khi ra quyết định mới là quan trọng nhất.
* **Giải pháp chuẩn hơn**: Nên tận dụng mốc thời gian phần cứng của camera trong header tin nhắn (`msg->header.stamp`) để tính độ trễ thực tế:
  $$\text{Latency}_{\text{End-to-End}} = \text{Thời gian hiện tại} - \text{msg->header.stamp}$$

#### Hạn chế 3: Bỏ qua thời gian publish gói tin lớn trong chỉ số gửi Dashboard
* **Vấn đề**: Chỉ số `full_latency` gửi lên Web Dashboard được tính toán *trước* khi gọi hàm `publish()`. 
* **Thực tế**: Chuỗi JSON chứa rất nhiều tọa độ contour của làn đường và vật cản là một gói dữ liệu khá lớn. Việc gọi hàm `publish()` gửi gói tin này qua ROS2 chiếm một khoảng thời gian không hề nhỏ trên CPU của Pi 5 (đã được ghi nhận trong biến debug `publish_latency`). Việc bỏ qua nó khiến độ trễ hiển thị trên Dashboard có vẻ "đẹp" hơn thực tế.

---

### Tóm lại
* **Đo thời gian chạy mô hình (NCNN Inference)** và **từng phân đoạn**: **Đúng hoàn toàn** và rất hữu ích cho việc debug hiệu năng thuật toán.
* **Đo FPS và Latency hệ thống**: **Chưa hoàn toàn chính xác** vì mới chỉ đo trễ nội bộ của node xử lý (processing latency), chưa phải độ trễ toàn hệ thống (system latency) và FPS thực tế (actual throughput).