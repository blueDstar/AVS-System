# 🤖 Robot Differential Drive — PID Pose Controller Simulation

Mô phỏng robot tự hành 4 bánh vi sai (skid-steer) chạy qua các waypoint bằng thuật toán **Pose Controller** (dựa trên hệ tọa độ cực).

---

## 📁 Cấu trúc thư mục

```
coordinatesPID/
├── main.py          ← Điểm khởi động
├── robot.py         ← Robot class (state + kinematics)
├── controller.py    ← PoseController + WaypointManager
├── simulator.py     ← Pygame loop + Renderer + Dialog
├── config.py        ← Thông số cấu hình
└── README.md        ← Tài liệu này
```

---

## ⚙️ Cài đặt

```bash
pip install pygame numpy
```

## 🚀 Chạy

```bash
cd coordinatesPID
python main.py
```

---

## 🎮 Phím điều khiển

| Phím  | Chức năng                     |
|-------|-------------------------------|
| `R`   | Reset / nhập lại waypoint     |
| `P`   | Tạm dừng / tiếp tục           |
| `T`   | Bật/tắt trail đường đi        |
| `ESC` | Thoát                         |

---

## 📐 Mô hình xe

```
    [ZQ]───────[YQ]
      |    →    |      → = hướng đầu xe (trục X robot)
      |   [ ]   |      ZQ = bánh trái trước
    [ZH]───────[YH]    YH = bánh phải sau
```

| Thông số     | Giá trị    | Đơn vị |
|-------------|------------|--------|
| Wheelbase L | 0.091      | m      |
| Track B     | 0.135      | m      |
| Wheel r     | 0.0325     | m      |
| v_max       | 0.5        | m/s    |
| ω_max       | 1.5        | rad/s  |

---

## 🧮 Thuật toán điều khiển

### Biến sai số (hệ tọa độ cực)

```
dx    = x_goal - x
dy    = y_goal - y
rho   = sqrt(dx² + dy²)             ← khoảng cách đến mục tiêu
alpha = wrap_to_pi(atan2(dy,dx) - theta)  ← sai số hướng đến mục tiêu
θ_err = wrap_to_pi(theta_goal - theta)    ← sai số góc đích cuối
```

### Luật điều khiển

```
v     = k_rho   * rho                      → tiến về phía mục tiêu
omega = k_alpha * alpha + k_theta * θ_err  → quay đúng hướng
```

### Tại sao cần `wrap_to_pi`?

Góc tích lũy liên tục (ví dụ: `theta = 7.3 rad`).
Không chuẩn hóa → sai số tính sai → robot quay liên tục không dừng.
`wrap_to_pi` đưa về `[-π, π]` để phép trừ góc luôn đúng.

---

## 🔄 Pipeline (tương đương ROS2)

```
Waypoints
    ↓
PoseController.compute_control(pose, goal)
    ↓
geometry_msgs/Twist  { linear.x = v,  angular.z = omega }
    ↓
Differential Drive Kinematics
    v_left  = v - (B/2) * omega
    v_right = v + (B/2) * omega
    ↓
Robot.update(v, omega, dt)   ← Euler integration
    x     += v * cos(θ) * dt
    y     += v * sin(θ) * dt
    theta += omega * dt
    ↓
nav_msgs/Odometry  { x, y, theta }
```

---

## 🔧 Mở rộng trong tương lai

| Controller       | Cách thay thế                            |
|-----------------|------------------------------------------|
| Pure Pursuit    | Kế thừa `BaseController`, override `compute_control()` |
| Stanley         | Dùng cross-track error thay alpha        |
| Lyapunov        | Ổn định toàn cục có chứng minh          |
| Sliding Mode    | Bền vững với nhiễu                       |
| MPC             | Tối ưu hóa đa bước                       |

Thay waypoints bằng **camera lane detection**:
```python
# Thay dòng này trong simulator.py:
goal = self.wp_manager.current_goal
# Bằng:
goal = lane_detector.get_next_target()
```
