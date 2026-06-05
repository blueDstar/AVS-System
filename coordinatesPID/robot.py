"""
=============================================================================
 robot.py - Robot Class: State + Kinematics
=============================================================================
 Mô tả:
   Mô hình toán học của robot 4 bánh vi sai (differential drive / skid-steer).
   Lớp này chịu trách nhiệm:
     1. Lưu trữ trạng thái (state): x, y, theta
     2. Tính toán động học thuận (forward kinematics): v, omega → x_dot, y_dot, theta_dot
     3. Tích phân trạng thái theo thời gian

 Tương đương trong ROS2:
   - Input:  geometry_msgs/Twist (linear.x = v, angular.z = omega)
   - Output: nav_msgs/Odometry  (x, y, theta)

 Mô hình động học vi sai:
   x_dot     = v * cos(theta)
   y_dot     = v * sin(theta)
   theta_dot = omega

 Tại sao dùng mô hình này?
   Robot skid-steer điều khiển bánh trái/phải độc lập.
   Vận tốc tuyến tính v là trung bình 2 bên,
   vận tốc góc omega phụ thuộc vào chênh lệch tốc độ 2 bên.
   Mô hình ICR (Instantaneous Center of Rotation) áp dụng hoàn toàn.
=============================================================================
"""

import math
import numpy as np
from config import (
    ROBOT_L, ROBOT_B, ROBOT_WHEEL_RADIUS,
    DT, V_MAX, OMEGA_MAX
)


class Robot:
    """
    Mô hình robot 4 bánh vi sai (Differential Drive / Skid-Steer).

    Thông số vật lý:
      L = 0.091 m  ← khoảng cách trước-sau (wheelbase)
      B = 0.135 m  ← khoảng cách trái-phải (track width)
      r = 0.0325 m ← bán kính bánh

    Sơ đồ bố trí bánh:
        [ZQ]-------[YQ]
          |    →     |      → = hướng đầu xe
          |   [ ]    |      ZQ=ZháQ, ZH=ZháuH, YQ=YPhải, YH=YPhảiH
        [ZH]-------[YH]

    Quy ước:
      - Trục X: hướng tiến xe
      - Trục Y: sang phải (nhìn từ trên)
      - Theta: góc so với trục X dương, ngược chiều kim đồng hồ (+)
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        """
        Khởi tạo robot tại vị trí (x, y) với hướng theta.

        Args:
            x     (float): tọa độ X ban đầu [m]
            y     (float): tọa độ Y ban đầu [m]
            theta (float): góc hướng ban đầu [rad]
        """
        # ── Trạng thái pose ──────────────────────────────────────────────────
        self.x     = x
        self.y     = y
        self.theta = theta

        # ── Thông số vật lý ──────────────────────────────────────────────────
        self.L = ROBOT_L           # wheelbase [m]
        self.B = ROBOT_B           # track width [m]
        self.r = ROBOT_WHEEL_RADIUS # bán kính bánh [m]

        # ── Vận tốc điều khiển hiện tại ──────────────────────────────────────
        self.v     = 0.0   # vận tốc tuyến tính [m/s]
        self.omega = 0.0   # vận tốc góc [rad/s]

        # ── Vận tốc bánh xe (tính từ v, omega) ───────────────────────────────
        self.v_left  = 0.0   # vận tốc dài bánh trái [m/s]
        self.v_right = 0.0   # vận tốc dài bánh phải [m/s]

        # ── Tốc độ góc từng bánh [rad/s] ─────────────────────────────────────
        # ZQ = Zá Quặt (trái trước), ZH = Zá Hậu (trái sau)
        # YQ = Yá Quặt (phải trước), YH = Yá Hậu (phải sau)
        self.omega_zq = 0.0   # bánh trái trước
        self.omega_zh = 0.0   # bánh trái sau
        self.omega_yq = 0.0   # bánh phải trước
        self.omega_yh = 0.0   # bánh phải sau

        # ── Lịch sử trajectory (để vẽ trail) ─────────────────────────────────
        self.trail = [(x, y)]

        # ── Bộ đếm cập nhật (dùng cho trail sampling) ─────────────────────────
        self._step_count = 0
        self._trail_sample_rate = 3  # lưu mỗi N bước

    # =========================================================================
    # KINEMATICS: Tính vận tốc bánh từ v và omega
    # =========================================================================

    def compute_wheel_velocities(self, v: float, omega: float) -> dict:
        """
        Tính vận tốc từng bánh từ lệnh (v, omega).

        Công thức differential drive:
          v_left  = v - (B/2) * omega
          v_right = v + (B/2) * omega

        Giải thích:
          - Bánh trái nằm cách tâm B/2 về phía trái.
          - Khi omega > 0 (quay ngược chiều KĐH = quay trái):
              → bánh phải quay nhanh hơn
              → bánh trái quay chậm hơn (hoặc ngược)
          - Điều này tạo ra sự chênh lệch → robot quay

        Args:
            v     (float): vận tốc tuyến tính [m/s]
            omega (float): vận tốc góc [rad/s]

        Returns:
            dict: các vận tốc bánh và tốc độ góc bánh
        """
        # Vận tốc dài của mỗi bên bánh [m/s]
        v_left  = v - (self.B / 2.0) * omega
        v_right = v + (self.B / 2.0) * omega

        # Tốc độ góc mỗi bánh [rad/s] = v_side / r
        omega_wheel_left  = v_left  / self.r
        omega_wheel_right = v_right / self.r

        return {
            'v_left':       v_left,
            'v_right':      v_right,
            'omega_zq':     omega_wheel_left,   # trái trước
            'omega_zh':     omega_wheel_left,   # trái sau (cùng side)
            'omega_yq':     omega_wheel_right,  # phải trước
            'omega_yh':     omega_wheel_right,  # phải sau (cùng side)
        }

    # =========================================================================
    # UPDATE: Tích phân trạng thái theo thời gian
    # =========================================================================

    def update(self, v: float, omega: float, dt: float = DT):
        """
        Cập nhật trạng thái robot theo bước thời gian dt.

        Mô hình động học vi sai:
          x_dot     = v * cos(theta)   ← thành phần X của vận tốc
          y_dot     = v * sin(theta)   ← thành phần Y của vận tốc
          theta_dot = omega            ← tốc độ quay

        Tích phân Euler (phù hợp với dt nhỏ như 0.02s):
          x     += x_dot     * dt
          y     += y_dot     * dt
          theta += theta_dot * dt

        Tại sao dùng cos/sin?
          Robot di chuyển theo hướng theta hiện tại.
          Phân tích vector vận tốc theo 2 trục:
            - cos(theta) = hình chiếu lên trục X
            - sin(theta) = hình chiếu lên trục Y

        Args:
            v     (float): vận tốc tuyến tính [m/s]  (từ controller)
            omega (float): vận tốc góc [rad/s]       (từ controller)
            dt    (float): bước thời gian [s]
        """
        # Giới hạn vận tốc an toàn (saturation)
        v     = np.clip(v,     -V_MAX,     V_MAX)
        omega = np.clip(omega, -OMEGA_MAX, OMEGA_MAX)

        # Lưu vận tốc hiện tại
        self.v     = v
        self.omega = omega

        # ── Tính vận tốc bánh ────────────────────────────────────────────────
        wheel_data = self.compute_wheel_velocities(v, omega)
        self.v_left    = wheel_data['v_left']
        self.v_right   = wheel_data['v_right']
        self.omega_zq  = wheel_data['omega_zq']
        self.omega_zh  = wheel_data['omega_zh']
        self.omega_yq  = wheel_data['omega_yq']
        self.omega_yh  = wheel_data['omega_yh']

        # ── Tích phân động học ────────────────────────────────────────────────
        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # ── Chuẩn hóa góc về [-π, π] ─────────────────────────────────────────
        self.theta = self._wrap_to_pi(self.theta)

        # ── Ghi trail ─────────────────────────────────────────────────────────
        self._step_count += 1
        if self._step_count % self._trail_sample_rate == 0:
            self.trail.append((self.x, self.y))
            # Giới hạn độ dài trail để tránh tràn bộ nhớ
            if len(self.trail) > 3000:
                self.trail.pop(0)

    # =========================================================================
    # UTILITY
    # =========================================================================

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """
        Chuẩn hóa góc về khoảng [-π, π].

        Tại sao cần?
          - Góc theta tích lũy theo thời gian (ví dụ: 7.3 rad sau nhiều vòng quay).
          - Nếu không chuẩn hóa, sai số góc alpha = atan2(...) - theta
            có thể là 7.3 - 0.5 = 6.8 thay vì ~-0.5.
          - Robot sẽ quay theo chiều sai và liên tục.

        Công thức:
          angle = (angle + π) mod (2π) - π
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    def reset(self, x=0.0, y=0.0, theta=0.0):
        """Reset robot về trạng thái ban đầu."""
        self.x     = x
        self.y     = y
        self.theta = theta
        self.v     = 0.0
        self.omega = 0.0
        self.v_left = self.v_right = 0.0
        self.omega_zq = self.omega_zh = 0.0
        self.omega_yq = self.omega_yh = 0.0
        self.trail = [(x, y)]
        self._step_count = 0

    @property
    def pose(self) -> tuple:
        """Trả về pose hiện tại (x, y, theta)."""
        return (self.x, self.y, self.theta)

    def get_state_dict(self) -> dict:
        """
        Trả về dictionary chứa toàn bộ trạng thái robot.
        Tiện dụng để log hoặc gửi qua ROS2 topic.
        """
        return {
            'x':           self.x,
            'y':           self.y,
            'theta':       self.theta,
            'v':           self.v,
            'omega':       self.omega,
            'v_left':      self.v_left,
            'v_right':     self.v_right,
            'omega_zq':    self.omega_zq,
            'omega_zh':    self.omega_zh,
            'omega_yq':    self.omega_yq,
            'omega_yh':    self.omega_yh,
        }
