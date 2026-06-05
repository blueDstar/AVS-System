"""
=============================================================================
 controller.py - PoseController: Thuật Toán Điều Khiển Hướng Đến Mục Tiêu
=============================================================================
 Mô tả:
   Bộ điều khiển dạng pose-based (hay còn gọi là heading controller).
   Tính toán lệnh (v, omega) để robot di chuyển đến waypoint mục tiêu.

 Nguồn gốc thuật toán:
   Dựa trên lý thuyết điều khiển Lyapunov-based pose stabilization
   cho nonholonomic robots (Astolfi, 1999; Park & Kuipers, 2011).

 Tại sao KHÔNG dùng PID thuần túy cho pose?
   Robot nonholonomic (không trượt ngang được) cần bộ điều khiển đặc biệt.
   PID thông thường cho x và y riêng lẻ sẽ gây xung đột.
   Thay vào đó, ta dùng hệ tọa độ cực (rho, alpha, beta) để điều khiển.

 Tương đương ROS2:
   - Input:  robot pose (x, y, theta) + goal pose (x_g, y_g, theta_g)
   - Output: geometry_msgs/Twist (linear.x, angular.z)

 Mở rộng:
   Để thay thế bằng Pure Pursuit, Stanley, hay MPC:
   → Chỉ cần tạo class mới kế thừa BaseController
   → Override method compute_control()
=============================================================================
"""

import math
import numpy as np
from config import (
    K_RHO, K_ALPHA, K_THETA,
    V_MAX, OMEGA_MAX,
    RHO_THRESHOLD, THETA_THRESHOLD
)


class BaseController:
    """
    Abstract base class cho các bộ điều khiển.
    Để thay thế bằng Pure Pursuit / Stanley / Lyapunov / SMC:
      → Kế thừa lớp này và override compute_control()
    """

    def compute_control(self, robot_pose: tuple, goal_pose: tuple) -> tuple:
        """
        Tính toán lệnh điều khiển.

        Args:
            robot_pose (tuple): (x, y, theta) của robot hiện tại
            goal_pose  (tuple): (x_g, y_g, theta_g) mục tiêu

        Returns:
            tuple: (v, omega) - vận tốc tuyến tính và góc
        """
        raise NotImplementedError("Subclass must implement compute_control()")

    def is_goal_reached(self, robot_pose: tuple, goal_pose: tuple) -> bool:
        """Kiểm tra xem robot đã đến mục tiêu chưa."""
        raise NotImplementedError("Subclass must implement is_goal_reached()")


class PoseController(BaseController):
    """
    Bộ điều khiển pose-based dựa trên hệ tọa độ cực.

    ─────────────────────────────────────────────────────
    GIẢI THÍCH TOÁN HỌC:
    ─────────────────────────────────────────────────────

    Định nghĩa các biến sai số:
      dx    = x_goal - x_robot
      dy    = y_goal - y_robot
      rho   = sqrt(dx² + dy²)         ← khoảng cách Euclidean đến mục tiêu

      alpha = wrap_to_pi(atan2(dy, dx) - theta)
              ↑ sai số giữa hướng robot VÀ hướng đến mục tiêu
              ↑ nếu alpha > 0 → mục tiêu lệch trái → robot cần quay trái
              ↑ nếu alpha < 0 → mục tiêu lệch phải → robot cần quay phải

      theta_error = wrap_to_pi(theta_goal - theta)
              ↑ sai số giữa hướng robot VÀ hướng đích cuối
              ↑ giúp robot căn đúng góc khi tiếp cận waypoint

    Công thức điều khiển:
      v     = k_rho * rho                          ← proportional control
      omega = k_alpha * alpha + k_theta * theta_error ← combined heading

    Tại sao dùng cả alpha VÀ theta_error?
      - alpha  → giúp robot "nhìn" về phía waypoint trong quá trình di chuyển
      - theta_error → giúp robot đạt đúng góc CUỐI tại waypoint (heading goal)
      - Nếu chỉ dùng alpha: robot sẽ đến đúng (x,y) nhưng sai góc cuối
      - Nếu chỉ dùng theta_error: robot sẽ chỉ xoay tại chỗ

    Tại sao cần wrap_to_pi?
      atan2 trả về [-π, π] nhưng theta tích lũy liên tục.
      Ví dụ: theta = 7.3 rad, atan2 = 0.5 rad
        → alpha thực = 0.5 - 7.3 = -6.8 (SAI, vì -6.8 ≡ -0.5 mod 2π)
        → Cần wrap_to_pi(-6.8) = -0.5 (ĐÚNG)

    Tại sao robot quay đầu?
      Khi alpha lớn (mục tiêu lệch nhiều so với hướng robot):
        omega = k_alpha * alpha → lớn → robot quay nhanh
        v     = k_rho * rho    → nhỏ khi mới xuất phát → robot quay hơn tiến
      Khi gần đích (rho → 0): v → 0, robot chỉ xoay để căn góc theta_goal

    Điều kiện hoàn thành:
      rho < 0.03 m    VÀ    |theta_error| < 0.05 rad
      ↑ có thể điều chỉnh trong config.py
    ─────────────────────────────────────────────────────

    Mở rộng trong tương lai:
      ┌─────────────────────────────────────────────────┐
      │ Pure Pursuit  → theo curvilinear path           │
      │ Stanley       → dùng cross-track error          │
      │ Lyapunov      → ổn định toàn cục có chứng minh │
      │ SMC           → bền vững với nhiễu              │
      │ MPC           → tối ưu hóa đa bước              │
      └─────────────────────────────────────────────────┘
    """

    def __init__(self,
                 k_rho:   float = K_RHO,
                 k_alpha: float = K_ALPHA,
                 k_theta: float = K_THETA,
                 v_max:   float = V_MAX,
                 omega_max: float = OMEGA_MAX):
        """
        Khởi tạo PoseController.

        Args:
            k_rho   (float): hệ số khuếch đại theo khoảng cách
            k_alpha (float): hệ số khuếch đại theo sai số góc heading
            k_theta (float): hệ số khuếch đại theo sai số góc đích
            v_max   (float): giới hạn vận tốc tuyến tính [m/s]
            omega_max (float): giới hạn vận tốc góc [rad/s]
        """
        self.k_rho    = k_rho
        self.k_alpha  = k_alpha
        self.k_theta  = k_theta
        self.v_max    = v_max
        self.omega_max = omega_max

        # Lưu giá trị sai số gần nhất để debug/hiển thị
        self.rho         = 0.0
        self.alpha       = 0.0
        self.theta_error = 0.0
        self.last_v      = 0.0
        self.last_omega  = 0.0

    # =========================================================================
    # CORE CONTROL
    # =========================================================================

    def compute_control(self, robot_pose: tuple, goal_pose: tuple) -> tuple:
        """
        Tính lệnh điều khiển (v, omega) để robot đến goal_pose.

        PIPELINE ĐIỀU KHIỂN:
          robot_pose, goal_pose
              ↓
          Tính sai số: rho, alpha, theta_error
              ↓
          Tính v = k_rho * rho
          Tính omega = k_alpha * alpha + k_theta * theta_error
              ↓
          Clamp trong [v_max, omega_max]
              ↓
          Trả về (v, omega)

        Args:
            robot_pose (tuple): (x, y, theta) robot hiện tại [m, m, rad]
            goal_pose  (tuple): (x_g, y_g, theta_g) mục tiêu [m, m, rad]

        Returns:
            tuple: (v [m/s], omega [rad/s])
        """
        x,   y,   theta   = robot_pose
        x_g, y_g, theta_g = goal_pose

        # ── Sai số vị trí ─────────────────────────────────────────────────────
        dx = x_g - x
        dy = y_g - y

        # Khoảng cách Euclidean đến mục tiêu
        rho = math.sqrt(dx * dx + dy * dy)

        # ── Sai số góc heading về phía mục tiêu ──────────────────────────────
        # atan2(dy, dx) = góc của vector (dx, dy) so với trục X
        # alpha = góc cần quay để robot nhìn về phía mục tiêu
        alpha = self._wrap_to_pi(math.atan2(dy, dx) - theta)

        # ── Sai số góc đích cuối ──────────────────────────────────────────────
        # theta_error = góc cần quay để robot đạt đúng heading tại waypoint
        theta_error = self._wrap_to_pi(theta_g - theta)

        # ── Tính vận tốc điều khiển ───────────────────────────────────────────
        v     = self.k_rho   * rho
        omega = self.k_alpha * alpha + self.k_theta * theta_error

        # ── Giới hạn vận tốc (saturation) ────────────────────────────────────
        v     = float(np.clip(v,     -self.v_max,     self.v_max))
        omega = float(np.clip(omega, -self.omega_max, self.omega_max))

        # Lưu giá trị để debug
        self.rho         = rho
        self.alpha       = alpha
        self.theta_error = theta_error
        self.last_v      = v
        self.last_omega  = omega

        return v, omega

    def is_goal_reached(self, robot_pose: tuple, goal_pose: tuple) -> bool:
        """
        Kiểm tra robot đã đến mục tiêu chưa.

        Điều kiện đồng thời:
          1. rho < RHO_THRESHOLD      → đã đến vị trí (x, y)
          2. |theta_error| < THETA_THRESHOLD → đã đúng góc hướng

        Ghi chú:
          Cần cả hai điều kiện để tránh robot "đếm điểm" khi đi ngang qua
          mà chưa dừng đúng góc.
        """
        x,   y,   theta   = robot_pose
        x_g, y_g, theta_g = goal_pose

        dx  = x_g - x
        dy  = y_g - y
        rho = math.sqrt(dx * dx + dy * dy)
        theta_error = abs(self._wrap_to_pi(theta_g - theta))

        return (rho < RHO_THRESHOLD) and (theta_error < THETA_THRESHOLD)

    def get_debug_info(self) -> dict:
        """
        Trả về thông tin debug để hiển thị HUD.
        Tiện dụng khi tích hợp vào ROS2 diagnostic topic.
        """
        return {
            'rho':         self.rho,
            'alpha':       self.alpha,
            'theta_error': self.theta_error,
            'v_cmd':       self.last_v,
            'omega_cmd':   self.last_omega,
        }

    # =========================================================================
    # UTILITY
    # =========================================================================

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """
        Chuẩn hóa góc về [-π, π].

        Sử dụng atan2(sin, cos) thay vì modulo để xử lý
        đúng cả số âm và vô hạn.

        Ví dụ:
          wrap_to_pi(7.0)  = 0.717
          wrap_to_pi(-7.0) = -0.717
          wrap_to_pi(π)    = π
          wrap_to_pi(-π)   = -π (hoặc π, tùy cài đặt)
        """
        return math.atan2(math.sin(angle), math.cos(angle))


# =============================================================================
# WAYPOINT MANAGER - Quản lý danh sách waypoint
# =============================================================================

class WaypointManager:
    """
    Quản lý danh sách waypoint và trạng thái hiện tại.

    Chịu trách nhiệm:
      - Lưu trữ danh sách waypoint
      - Theo dõi waypoint hiện tại (index)
      - Chuyển sang waypoint tiếp theo khi đã đến đích
      - Báo hiệu khi hoàn thành tất cả waypoint
    """

    def __init__(self, waypoints: list):
        """
        Khởi tạo với danh sách waypoint.

        Args:
            waypoints (list): danh sách [(x, y, theta), ...]
        """
        self.waypoints     = list(waypoints)
        self.current_index = 0
        self.completed     = False
        self.reached_flags = [False] * len(waypoints)  # waypoint nào đã đi qua

    @property
    def current_goal(self) -> tuple:
        """Trả về waypoint đang hướng đến."""
        if self.completed:
            return self.waypoints[-1]
        return self.waypoints[self.current_index]

    @property
    def total(self) -> int:
        """Tổng số waypoint."""
        return len(self.waypoints)

    def advance(self):
        """
        Chuyển sang waypoint tiếp theo.
        Nếu đã qua hết → đánh dấu completed.
        """
        self.reached_flags[self.current_index] = True
        self.current_index += 1
        if self.current_index >= len(self.waypoints):
            self.current_index = len(self.waypoints) - 1
            self.completed = True

    def reset(self, waypoints: list = None):
        """Reset về đầu danh sách."""
        if waypoints is not None:
            self.waypoints = list(waypoints)
        self.current_index = 0
        self.completed = False
        self.reached_flags = [False] * len(self.waypoints)

    def get_status(self) -> dict:
        """Trả về trạng thái để hiển thị HUD."""
        return {
            'current_index': self.current_index,
            'total':         self.total,
            'completed':     self.completed,
            'current_goal':  self.current_goal,
        }
