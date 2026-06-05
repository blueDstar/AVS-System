"""
=============================================================================
 config.py - Thông số cấu hình toàn hệ thống
=============================================================================
 Tất cả hằng số vật lý, điều khiển, và giao diện được tập trung tại đây
 để dễ dàng tinh chỉnh và mở rộng.
=============================================================================
"""

# ---------------------------------------------------------------------------
# THÔNG SỐ VẬT LÝ ROBOT (Yahboomcar / Robot thực tế)
# ---------------------------------------------------------------------------

# Khoảng cách trục bánh trước-sau (wheelbase) [m]
ROBOT_L = 0.091

# Khoảng cách bánh trái-phải (track width) [m]
ROBOT_B = 0.135

# Bán kính bánh xe [m]
ROBOT_WHEEL_RADIUS = 0.0325

# Kích thước hiển thị robot (theo tỉ lệ vẽ, không phải thực)
ROBOT_DRAW_LENGTH = 0.18    # chiều dài hình chữ nhật robot (m)
ROBOT_DRAW_WIDTH  = 0.14    # chiều rộng hình chữ nhật robot (m)

# ---------------------------------------------------------------------------
# THÔNG SỐ BỘ ĐIỀU KHIỂN POSE (Pose Controller)
# ---------------------------------------------------------------------------
#
# GIẢI THÍCH TOÁN HỌC:
#   - rho   : khoảng cách Euclidean từ robot đến mục tiêu
#   - alpha : sai số góc giữa hướng robot và hướng đến mục tiêu
#             alpha = atan2(dy, dx) - theta
#   - theta_error: sai số góc so với hướng mục tiêu cuối (heading goal)
#             theta_error = theta_goal - theta
#
#   Công thức điều khiển:
#     v     = k_rho * rho              ← tiến về phía mục tiêu
#     omega = k_alpha * alpha + k_theta * theta_error ← quay về hướng đúng
#
#   Tại sao cần wrap_to_pi?
#     Góc theta có thể > 2π hoặc < -2π do tích lũy. Cần chuẩn hóa
#     về [-π, π] để tránh robot quay ngược chiều sai.
#

# Hệ số khuếch đại điều khiển
K_RHO   = 0.8   # tỉ lệ thuận với khoảng cách → điều chỉnh tốc độ tiến
K_ALPHA = 2.0   # tỉ lệ thuận với sai lệch góc heading → điều chỉnh tốc độ quay
K_THETA = 0.5   # tỉ lệ thuận với sai lệch góc đích → giúp căn hướng cuối

# Giới hạn vận tốc
V_MAX     = 0.5   # [m/s] - vận tốc tuyến tính tối đa
OMEGA_MAX = 1.5   # [rad/s] - vận tốc góc tối đa

# Ngưỡng hoàn thành waypoint
RHO_THRESHOLD   = 0.03   # [m]   - coi là đã đến đích khi rho < ngưỡng này
THETA_THRESHOLD = 0.05   # [rad] - coi là đúng hướng khi |theta_error| < ngưỡng này

# ---------------------------------------------------------------------------
# THÔNG SỐ MÔ PHỎNG
# ---------------------------------------------------------------------------

# Bước thời gian mô phỏng [s]
DT = 0.02

# Tốc độ khung hình mục tiêu
FPS = 60

# Số bước mô phỏng mỗi khung hình (để mô phỏng nhanh hơn render)
SIM_STEPS_PER_FRAME = 1

# ---------------------------------------------------------------------------
# THÔNG SỐ GIAO DIỆN PYGAME
# ---------------------------------------------------------------------------

# Kích thước cửa sổ [pixels]
SCREEN_WIDTH  = 1280
SCREEN_HEIGHT = 720

# Tỉ lệ chuyển đổi: 1 m trong thực tế = bao nhiêu pixel
# Với màn hình 1280x720 và vùng hiển thị 8m x 5m → 100 px/m
PIXELS_PER_METER = 100   # [px/m]

# Gốc tọa độ (origin) trên màn hình tính từ góc trên-trái
ORIGIN_X = 100    # [px] - gốc O trên màn hình
ORIGIN_Y = 620    # [px] - gốc O trên màn hình (gần đáy vì Y thực lên trên)

# Kích thước ô lưới [m]
GRID_SIZE = 0.5

# ---------------------------------------------------------------------------
# BẢNG MÀU (Color Palette)
# ---------------------------------------------------------------------------

# Nền
COLOR_BACKGROUND  = (15,  20,  35)    # Xanh đen đậm
COLOR_GRID_MAJOR  = (40,  55,  80)    # Lưới chính
COLOR_GRID_MINOR  = (25,  35,  55)    # Lưới phụ

# Trục tọa độ
COLOR_AXIS_X      = (220,  60,  60)   # Đỏ - trục X
COLOR_AXIS_Y      = (60,  200,  80)   # Xanh lá - trục Y
COLOR_AXIS_LABEL  = (200, 200, 220)   # Nhãn trục

# Robot body
COLOR_ROBOT_BODY  = (50,  130, 200)   # Xanh dương robot
COLOR_ROBOT_FRONT = (80,  200, 255)   # Đầu robot (sáng hơn)
COLOR_ROBOT_WHEEL = (180, 180, 190)   # Màu bánh xe
COLOR_ROBOT_CROSS = (255, 230,  50)   # Dấu X tại tâm

# Arrow/Heading
COLOR_HEADING_ARROW = (255, 200,  50) # Mũi tên hướng

# Trail/Path
COLOR_TRAIL       = (100, 160, 255)   # Đường đã đi
COLOR_TRAIL_FADE  = (40,   80, 150)   # Cuối trail (mờ hơn)

# Waypoints
COLOR_WP_DONE     = (100, 200, 100)   # Waypoint đã đi qua
COLOR_WP_ACTIVE   = (255, 200,  50)   # Waypoint đang hướng đến
COLOR_WP_PENDING  = (150, 150, 200)   # Waypoint chờ
COLOR_WP_LINE     = (80,  100, 150)   # Đường nối waypoint
COLOR_WP_LABEL    = (255, 255, 255)   # Nhãn waypoint

# HUD
COLOR_HUD_BG      = (15,  25,  50, 200)  # Nền HUD (semi-transparent)
COLOR_HUD_TEXT    = (200, 220, 255)       # Văn bản HUD
COLOR_HUD_VALUE   = (80,  220, 180)       # Giá trị số HUD
COLOR_HUD_TITLE   = (255, 200,  80)       # Tiêu đề HUD
COLOR_HUD_WARN    = (255, 120,  80)       # Cảnh báo HUD

# Input dialog
COLOR_DIALOG_BG   = (20,  30,  55)
COLOR_DIALOG_BORDER = (100, 150, 255)
COLOR_INPUT_BG    = (10,  15,  35)
COLOR_INPUT_ACTIVE = (80, 130, 255)

# ---------------------------------------------------------------------------
# WAYPOINTS MẶC ĐỊNH
# ---------------------------------------------------------------------------
# Mỗi waypoint: (x [m], y [m], theta [rad])
# Có thể thay bằng tọa độ từ camera hoặc path planner

DEFAULT_WAYPOINTS = [
    (1.0, 1.0, 0.0),
    (2.5, 1.5, 1.57),
    (3.0, 3.0, 3.14),
]

# Vị trí xuất phát mặc định
DEFAULT_START_X     = 0.0
DEFAULT_START_Y     = 0.0
DEFAULT_START_THETA = 0.0

# ---------------------------------------------------------------------------
# THÔNG SỐ FONT
# ---------------------------------------------------------------------------

FONT_SIZE_HUD    = 16
FONT_SIZE_LABEL  = 14
FONT_SIZE_TITLE  = 20
FONT_SIZE_DIALOG = 18
