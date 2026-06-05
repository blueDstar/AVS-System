"""
=============================================================================
 Robot Tự Hành 4 Bánh Vi Sai - Mô Phỏng PID / Pose Controller
=============================================================================
 Mô tả:
   Mô phỏng robot di chuyển qua các waypoint bằng bộ điều khiển pose-based.
   Giao diện pygame 2D hiển thị robot, đường đi, waypoint và thông số realtime.

 Cài đặt:
   pip install pygame numpy

 Chạy:
   python main.py

 Cấu trúc project:
   coordinatesPID/
   ├── main.py          ← điểm khởi động (file này)
   ├── robot.py         ← Robot class (state + kinematics)
   ├── controller.py    ← PoseController class (thuật toán điều khiển)
   ├── simulator.py     ← Simulator class (pygame loop + render)
   └── config.py        ← Thông số cấu hình

 Pipeline:
   Waypoints → PoseController → Twist(v, omega) → Robot kinematics → Render
   (Tương đương ROS2: geometry_msgs/Twist → diff_drive → odometry)
=============================================================================
"""

import sys
import os

# Thêm thư mục hiện tại vào sys.path để import module cùng cấp
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from simulator import Simulator


def main():
    """
    Hàm main: khởi động simulator và chạy vòng lặp chính.

    Thiết kế để dễ mở rộng:
      - Thay waypoints bằng camera lane detection
      - Thay Simulator bằng ROS2 node
      - Thay PoseController bằng Pure Pursuit / Stanley / MPC
    """
    print("=" * 60)
    print("  Robot Tự Hành 4 Bánh Vi Sai - Mô Phỏng Pygame")
    print("=" * 60)
    print("  Điều khiển:")
    print("    R  - Reset / nhập lại waypoint")
    print("    P  - Tạm dừng / tiếp tục")
    print("    T  - Bật/tắt hiển thị trail")
    print("    ESC - Thoát")
    print("=" * 60)

    # Khởi tạo và chạy simulator
    sim = Simulator()
    sim.run()


if __name__ == "__main__":
    main()
