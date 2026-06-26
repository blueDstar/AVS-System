# AVS Control System Documentation

## 1. Giới thiệu

`avs_controlsystem` là package ROS2 chịu trách nhiệm chuyển đổi sai số làn đường, dữ liệu telemetry và an toàn từ perception thành lệnh điều khiển xe `/cmd_vel`.

Nội dung file này mô tả:
- Kiến trúc điều khiển trong `avs_controlsystem`
- Các node chính và cách hoạt động
- Các topic ROS2 quan trọng
- Thuật toán điều khiển và quy ước dữ liệu
- Cách chạy và kiểm tra

## 2. Kiến trúc tổng quan

Các node điều khiển chính trong package:
- `mainlane_following_controlerror`
- `pur_persuit_mainlane_following`
- `safe_error_cmdvel_node`
- `avs_lane_cmdvel_node`
- `lane_lidar_follower_node`

Ngoài ra còn có các node hỗ trợ như:
- `lane_parser_node`
- `lane_target_from_telemetry_node`
- `lane_follow_controller_node`
- `path_visualizer_node`
- `run_report_node`

### Luồng dữ liệu chính

1. Perception publish `/avs/control_error` hoặc `/avs/telemetry_realworld`.
2. Control node đọc dữ liệu lỗi làn, tạo lệnh vận tốc.
3. Control node publish `/cmd_vel`.
4. `/cmd_vel` được gửi xuống ECU/ESP32 hoặc các cơ cấu chấp hành.

## 3. Các topic ROS2 quan trọng

| Topic | Direction | Message | Vai trò |
|---|---|---|---|
| `/avs/control_error` | input | `std_msgs/String` | JSON sai số làn đường từ perception
| `/avs/telemetry_realworld` | input | `std_msgs/String` | Dữ liệu telemetry fallback / lane target
| `/scan` | input | `sensor_msgs/LaserScan` | LiDAR cho an toàn và stop
| `/cmd_vel` | output | `geometry_msgs/Twist` | Lệnh vận tốc cuối cùng cho xe
| `/avs/safe_control_error` | input | `std_msgs/String` | Sai số an toàn có tính đến armed/disarmed
| `/avs/lane_state` | input | `std_msgs/String` | Trạng thái làn đường (trong node `avs_lane_cmdvel_node`)
| `/avs/lane_cmdvel_debug` | output | `std_msgs/String` | Debug payload của `avs_lane_cmdvel_node`
| `/avs/mainlane_control_debug` | output | `std_msgs/String` | Debug payload của `mainlane_following_controlerror`
| `/avs/safe_error_cmdvel_debug` | output | `std_msgs/String` | Debug payload của `safe_error_cmdvel_node`
| `/avs/pur_persuit_mainlane_debug` | output | `std_msgs/String` | Debug payload của `pur_persuit_mainlane_following`

## 4. Quy ước dữ liệu và hướng điều khiển

### Quy ước sai số

- `epsilon_x_mm` / `x_mm` / `lookahead_x_mm`
  - dương nếu mục tiêu nằm bên phải xe.
- `epsilon_y_mm` / `y_mm` / `lookahead_d_mm`
  - khoảng cách lookahead theo mm.
- `theta_rad` / `heading_error_rad`
  - sai số góc giữa hướng xe và hướng mục tiêu.
- `curvature_inv_mm`
  - độ cong nghịch đảo của đường đi, đơn vị mm^-1.

### Quy ước ROS angular

- `geometry_msgs/Twist.angular.z > 0` nghĩa là rẽ trái.
- Vì sai số ngang dương nghĩa là mục tiêu bên phải,
  nên các node điều khiển sử dụng `omega = -...` để rẽ phải đúng chiều.

## 5. Các node kiểm soát chính

### 5.1 `mainlane_following_controlerror`

Node này đọc `/avs/control_error` và xuất lệnh `/cmd_vel` trực tiếp.

#### Chức năng chính
- Nhận JSON từ `/avs/control_error`
- Lọc và bình ổn `x`, `theta`
- Tính `curvature = -2*x / L_d^2`
- Tính `v_target` theo độ cong và điều kiện cua
- Tính `omega_target` dùng Pure Pursuit + PID-like correction
- Giới hạn omega theo cơ học differential-drive
- Rate-limit `v` và `omega`
- Publish `/cmd_vel`

#### Tham số quan trọng
- `enable_motion` (default `True`)
- `control_rate_hz` (default `30.0`)
- `error_timeout_s` (default `1.5`)
- `v_max` (default `0.10`)
- `v_min` (default `0.025`)
- `v_turn_max` (default `0.055`)
- `omega_max` (default `0.32`)
- `k_pursuit` (default `1.00`)
- `k_heading` (default `0.28`)
- `kd_lateral` (default `0.006`)
- `kd_heading` (default `0.010`)
- `filter_alpha`, `v_rate_limit`, `omega_rate_limit`
- `x_deadband_m`, `theta_deadband_rad`, `omega_deadband`
- `lookahead_min_m`, `lookahead_max_m`, `default_lookahead_m`
- `wheel_separation_m`, `inner_wheel_min_fraction`, `allow_pivot_turn`
- `slow_lateral_gain`, `slow_heading_gain`, `slow_curvature_gain`
- `invert_angular`

#### Cơ chế an toàn
- Nếu dữ liệu lỗi quá cũ (`> error_timeout_s`) thì giảm tốc dần bằng `ramp_stop`
- Nếu `valid=false` thì `hard_stop`
- Khi nhận signal dừng (`SIGINT`/`SIGTERM`) thì xuất nhiều `/cmd_vel` bằng 0

#### Đầu ra debug
- `mode`: trạng thái hiện tại
- `v_cmd`, `omega_cmd`
- `error_age_s`
- `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`, `curvature_inv_mm`
- `v_left_est`, `v_right_est`

### 5.2 `pur_persuit_mainlane_following`

Node này cũng đọc `/avs/control_error` và dùng công thức Pure Pursuit để tạo `/cmd_vel`.

#### Điểm khác biệt chính so với `mainlane_following_controlerror`
- Tính `v_target = v_max * cos(theta) / (1 + k_c*|kappa_m|)`
- Tính `omega_target` từ công thức Pure Pursuit thô: `omega = v * (-2*x/L_d^2)`
- Thêm điều khiển chênh lệch bánh trong/ngoài
- Rate-limit thêm cả `delta_v` và chuyển thành `omega_cmd`

#### Tham số quan trọng
- `v_max` (default `0.10`)
- `v_min` (default `0.025`)
- `v_turn_min` (default `0.035`)
- `k_c` (default `0.20`)
- `omega_max` (default `0.35`)
- `wheel_separation_m` (default `0.36`)
- `max_delta_v` (default `0.085`)
- `delta_v_rate_limit` (default `0.16`)
- `inner_wheel_min_fraction` (default `0.45`)
- `x_deadband_m`, `theta_deadband_rad`, `omega_deadband`
- `filter_alpha`, `v_rate_limit`
- `max_abs_x_m`, `max_abs_theta_rad`
- `invert_angular`

#### Công thức quan trọng
- `e_x = epsilon_x_mm / 1000`
- `L_d = epsilon_y_mm / 1000`
- `kappa_m = curvature_inv_mm * 1000`
- `v_target = clamp(v_max*cos(theta)/(1+k_c*|kappa_m|), v_min, v_max)`
- `gamma = -2*e_x / L_d^2`
- `omega_target = v_target * gamma`
- `delta_v_target = omega_target * wheel_separation`

### 5.3 `safe_error_cmdvel_node`

Node này tạo lệnh an toàn từ cả `/avs/safe_control_error` và `/avs/control_error`.

#### Luồng chọn lỗi
1. Dùng `safe_control_error` nếu còn mới và hợp lệ
2. Nếu không, dùng `control_error` nếu `allow_raw_control_error_fallback=True`
3. Nếu không có lỗi hợp lệ, dừng xe hoặc publish debug trạng thái timeout

#### An toàn LiDAR
- Nếu `use_lidar_safety=True` thì node đọc thêm `/scan`
- Dừng khẩn cấp khi `front_dist < emergency_distance`
- Dừng khi `front_dist < stop_distance`
- Có chế độ giảm tốc khi `front_dist < slow_distance`

#### Tham số quan trọng
- `safe_error_timeout_s` (default `0.8`)
- `control_error_timeout_s` (default `0.8`)
- `allow_raw_control_error_fallback` (default `True`)
- `stop_on_timeout` (default `True`)
- `v_max`, `v_min`, `v_turn_max`, `omega_max`
- `k_pursuit`, `heading_gain`, `kd_lateral`, `kd_heading`
- `use_lidar_safety`, `emergency_distance`, `stop_distance`, `slow_distance`
- `front_angle_deg`, `side_min_angle_deg`, `side_max_angle_deg`
- `inner_min_fraction`, `allow_pivot_turn`

#### Debug output
- `front_dist`, `left_dist`, `right_dist`
- `safe_age_s`, `raw_age_s`
- `armed`, `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`, `curvature_inv_mm`

### 5.4 `avs_lane_cmdvel_node`

Node này là một bộ điều khiển kết hợp dành cho xe AVS mang hiệu năng an toàn cao.

#### Chức năng
- Ưu tiên `/avs/control_error`
- Fallback sang `/avs/telemetry_realworld` khi cần
- Dùng dữ liệu `objects[]` để chọn mục tiêu phù hợp với `target_label`
- Hỗ trợ `lane_state` và `scan` cho tránh chướng ngại vật
- Publish `/cmd_vel` và trạng thái debug

#### Tham số quan trọng
- `control_error_topic`, `telemetry_topic`, `lane_state_topic`
- `allow_telemetry_fallback`, `allow_label_fallback`
- `v_max`, `v_min`, `v_turn_max`, `omega_max`
- `k_pursuit`, `heading_gain`, `kd_lateral`, `kd_heading`
- `x_deadband_m`, `theta_deadband_rad`, `omega_deadband`
- `min_lookahead_m`, `max_lookahead_m`
- `use_lidar_safety`, `emergency_distance`, `stop_distance`, `slow_distance`
- `slow_x_gain`, `slow_theta_gain`, `slow_curvature_gain`

### 5.5 `lane_lidar_follower_node`

Node này dùng `Point` lane target và `LaserScan` để điều khiển xe bằng quy tắc PD kết hợp an toàn LiDAR.

#### Input / Output
- Subscribe `/lane_target` (`geometry_msgs/Point`)
- Subscribe `/scan` (`sensor_msgs/LaserScan`)
- Publish `/cmd_vel` (`geometry_msgs/Twist`)
- Publish obstacle marker `/simplerobot_obstacle_points`

#### Logic điều khiển
- `e_y = msg.x` là lateral error
- `e_heading = msg.y`
- `lane_valid = msg.z`
- Dùng PID đơn giản:
  - `linear.x = f(e_y)`
  - `angular.z = kp_heading*e_heading + kd_heading*e_heading_dot`
- Dừng nếu lane mất quá lâu hoặc LiDAR thấy chướng ngại vật.

## 6. Chạy package và node

### Build package

```bash
cd ros2_ws
colcon build --symlink-install --packages-select avs_controlsystem
source install/setup.bash
```

### Kiểm tra executables

```bash
ros2 pkg executables avs_controlsystem
```

Expected executables:
- `avs_lane_cmdvel_node`
- `lane_lidar_follower_node`
- `lane_parser_node`
- `mainlane_following_controlerror`
- `pur_persuit_mainlane_following`
- `pur_persuit_pd_mainlane_following`
- `safe_error_cmdvel_node`
- `smooth_lane_lidar_follower_node`
- `start_turn_pur_persuit_pd_following`
- ...

### Chạy node thủ công

Ví dụ chạy điều khiển main lane error:

```bash
ros2 run avs_controlsystem mainlane_following_controlerror
```

Ví dụ chạy node an toàn kết hợp LiDAR:

```bash
ros2 run avs_controlsystem safe_error_cmdvel_node
```

### Launch file mẫu

Các launch file hiện có trong `ros2_ws/src/avs_controlsystem/launch`:
- `avs_lane_follow.launch.py`
- `simplerobot_lane_lidar_demo.launch.py`
- `simplerobot_lidar_demo.launch.py`
- `lidar_avoidance_rviz.launch.py`

Ví dụ khởi chạy `avs_lane_follow.launch.py`:

```bash
ros2 launch avs_controlsystem avs_lane_follow.launch.py
```

## 7. Tích hợp với Web Dashboard

Web dashboard hiện hỗ trợ điều khiển qua topics:
- Publish `/avs/control_mode` từ dashboard để chuyển `auto` / `manual`
- Publish `/avs/control_params` từ dashboard để điều chỉnh thông số
- Publish `/cmd_vel` khi ở chế độ manual

Để xe nhận lệnh tự động, perception cần publish `/avs/control_error`.

## 8. Debug và kiểm tra

### Kiểm tra topic

```bash
ros2 topic echo /avs/control_error
ros2 topic echo /cmd_vel
ros2 topic echo /avs/safe_error_cmdvel_debug
ros2 topic echo /avs/mainlane_control_debug
```

### Lỗi thường gặp

- `No executable found`:
  - Chưa build lại package hoặc chưa source `install/setup.bash`.
- `publisher's context is invalid` khi dừng node:
  - Node đã shutdown trước khi xuất lệnh STOP. Thường do `rclpy.shutdown()` được gọi ngay sau khi stop.
- `rviz2: command not found` trong container:
  - Image Docker không cài ROS desktop. Dùng `rviz2` trên host với cùng ROS_DOMAIN_ID hoặc cài thêm gói `ros-humble-rviz2`.

### Kiểm tra build

```bash
python3 -m py_compile ros2_ws/src/avs_controlsystem/avs_controlsystem/*.py
```

## 9. Ghi chú quan trọng

- `cmd_vel` là lệnh cuối cùng cho xe; mọi node điều khiển đều có thể publish lên topic này.
- Hệ thống sử dụng `network_mode: host` trong Docker để host và container share mạng ROS.
- `enable_motion` cần bật để node publish lệnh. Nếu tắt, node luôn giữ trạng thái dừng.
- An toàn LiDAR và timeout cần được test thực tế trước khi chạy xe trên sân.

## 10. Tóm tắt chiến lược điều khiển

| Node | Dữ liệu vào | Chiến lược | Output |
|---|---|---|---|
| `mainlane_following_controlerror` | `/avs/control_error` | Pure Pursuit + PID + rate-limit | `/cmd_vel` |
| `pur_persuit_mainlane_following` | `/avs/control_error` | Pure Pursuit + differential speed | `/cmd_vel` |
| `safe_error_cmdvel_node` | `/avs/safe_control_error`, `/avs/control_error`, `/scan` | Chọn lỗi an toàn + LiDAR stop | `/cmd_vel` |
| `avs_lane_cmdvel_node` | `/avs/control_error`, `/avs/telemetry_realworld`, `/scan` | Fallback telemetry + label selection | `/cmd_vel` |
| `lane_lidar_follower_node` | `/lane_target`, `/scan` | PD lane following + obstacle avoidance | `/cmd_vel` |

---

File này được tạo để làm tài liệu tham khảo đầy đủ cho phần điều khiển của `avs_controlsystem` trong dự án SimpleSysIDV.