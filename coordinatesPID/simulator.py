"""
=============================================================================
 simulator.py - Simulator Class: Pygame Loop + Render + Input
=============================================================================
 Mô tả:
   Vòng lặp chính của mô phỏng. Chịu trách nhiệm:
     1. Khởi tạo pygame
     2. Nhập waypoint từ người dùng (dialog)
     3. Vòng lặp game: update physics + render
     4. Vẽ grid, robot, waypoint, trail, HUD

 Pipeline render:
   update() → controller.compute_control() → robot.update() → render()

 Tọa độ màn hình:
   - Hệ thực: X sang phải, Y lên trên (chuẩn toán học)
   - Hệ pygame: X sang phải, Y xuống dưới
   - Chuyển đổi: screen_y = ORIGIN_Y - world_y * PIXELS_PER_METER
=============================================================================
"""

import pygame
import math
import sys
import numpy as np
from typing import List, Optional

from robot      import Robot
from controller import PoseController, WaypointManager
from config     import *


# =============================================================================
# COORDINATE CONVERSION
# =============================================================================

def world_to_screen(x: float, y: float) -> tuple:
    """
    Chuyển tọa độ thực (m) sang tọa độ màn hình (px).

    Tại sao cần đổi?
      - Hệ thực: Y tăng lên trên (chuẩn toán)
      - Pygame:  Y tăng xuống dưới (chuẩn screen)
      → Cần đảo trục Y

    Args:
        x (float): tọa độ X thực [m]
        y (float): tọa độ Y thực [m]

    Returns:
        tuple: (sx, sy) tọa độ pixel màn hình
    """
    sx = int(ORIGIN_X + x * PIXELS_PER_METER)
    sy = int(ORIGIN_Y - y * PIXELS_PER_METER)
    return sx, sy


def screen_to_world(sx: int, sy: int) -> tuple:
    """Chuyển ngược từ pixel về tọa độ thực [m]."""
    x = (sx - ORIGIN_X) / PIXELS_PER_METER
    y = (ORIGIN_Y - sy) / PIXELS_PER_METER
    return x, y


# =============================================================================
# INPUT DIALOG - Nhập waypoint từ bàn phím
# =============================================================================

class WaypointInputDialog:
    """
    Giao diện nhập liệu waypoint trong pygame.

    Cho phép người dùng nhập 3 waypoint theo định dạng:
      "x y theta" (ví dụ: "1.0 1.0 0")
    """

    def __init__(self, screen: pygame.Surface, font: pygame.font.Font,
                 font_small: pygame.font.Font, n_waypoints: int = 3):
        self.screen      = screen
        self.font        = font
        self.font_small  = font_small
        self.n_waypoints = n_waypoints

        # Buffer nhập liệu cho từng waypoint
        self.inputs      = [""] * n_waypoints
        self.active_idx  = 0        # đang nhập waypoint nào
        self.error_msg   = ""
        self.done        = False
        self.waypoints   = []

        # Placeholder gợi ý
        self.placeholders = [
            "1.0 1.0 0",
            "2.5 1.5 1.57",
            "3.0 3.0 3.14",
        ]

    def handle_event(self, event: pygame.event.Event) -> bool:
        """
        Xử lý sự kiện bàn phím.

        Returns:
            bool: True nếu đã xong (nhấn Enter lần cuối)
        """
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                # Validate waypoint hiện tại
                ok = self._validate_current()
                if ok:
                    if self.active_idx < self.n_waypoints - 1:
                        self.active_idx += 1
                        self.error_msg = ""
                    else:
                        # Đã nhập xong tất cả
                        self._finalize()
                        return True

            elif event.key == pygame.K_BACKSPACE:
                if self.inputs[self.active_idx]:
                    self.inputs[self.active_idx] = self.inputs[self.active_idx][:-1]
                self.error_msg = ""

            elif event.key == pygame.K_UP:
                self.active_idx = max(0, self.active_idx - 1)
                self.error_msg = ""

            elif event.key == pygame.K_DOWN:
                self.active_idx = min(self.n_waypoints - 1, self.active_idx + 1)
                self.error_msg = ""

            elif event.key == pygame.K_TAB:
                # Nếu ô trống thì dùng placeholder
                if not self.inputs[self.active_idx].strip():
                    self.inputs[self.active_idx] = self.placeholders[self.active_idx]
                self.active_idx = min(self.n_waypoints - 1, self.active_idx + 1)

            elif event.key == pygame.K_ESCAPE:
                # Dùng waypoint mặc định
                self.waypoints = [tuple(wp) for wp in DEFAULT_WAYPOINTS]
                self.done = True
                return True

            else:
                # Nhập ký tự (chỉ chấp nhận số, khoảng trắng, dấu chấm, trừ)
                ch = event.unicode
                if ch in "0123456789.-+e ":
                    self.inputs[self.active_idx] += ch

        return False

    def _validate_current(self) -> bool:
        """Parse và validate waypoint hiện tại."""
        raw = self.inputs[self.active_idx].strip()
        # Nếu trống → dùng mặc định
        if not raw:
            raw = self.placeholders[self.active_idx]
            self.inputs[self.active_idx] = raw

        parts = raw.split()
        if len(parts) != 3:
            self.error_msg = f"Cần 3 giá trị: x y theta (ví dụ: {self.placeholders[self.active_idx]})"
            return False
        try:
            float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            self.error_msg = "Giá trị không hợp lệ! Chỉ nhập số."
            return False
        return True

    def _finalize(self):
        """Parse tất cả waypoint thành list."""
        self.waypoints = []
        for i, raw in enumerate(self.inputs):
            raw = raw.strip()
            if not raw:
                raw = self.placeholders[i]
            parts = raw.split()
            self.waypoints.append((float(parts[0]), float(parts[1]), float(parts[2])))
        self.done = True

    def render(self):
        """Vẽ dialog nhập liệu lên màn hình."""
        # Nền mờ
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 180))
        self.screen.blit(overlay, (0, 0))

        # Hộp dialog
        dlg_w, dlg_h = 600, 420
        dlg_x = (SCREEN_WIDTH  - dlg_w) // 2
        dlg_y = (SCREEN_HEIGHT - dlg_h) // 2

        # Shadow
        shadow = pygame.Surface((dlg_w + 8, dlg_h + 8), pygame.SRCALPHA)
        shadow.fill((0, 0, 0, 100))
        self.screen.blit(shadow, (dlg_x - 4, dlg_y - 4))

        # Background
        pygame.draw.rect(self.screen, COLOR_DIALOG_BG,
                         (dlg_x, dlg_y, dlg_w, dlg_h), border_radius=12)
        pygame.draw.rect(self.screen, COLOR_DIALOG_BORDER,
                         (dlg_x, dlg_y, dlg_w, dlg_h), 2, border_radius=12)

        # Tiêu đề
        title = self.font.render("🤖  Nhập Tọa Độ Waypoint", True, COLOR_HUD_TITLE)
        self.screen.blit(title, (dlg_x + 30, dlg_y + 20))

        # Dòng mô tả
        desc_lines = [
            "Định dạng:  x  y  theta   (đơn vị: m, m, rad)",
            "Enter → tiếp theo  |  Tab → dùng mặc định  |  ESC → dùng tất cả mặc định",
        ]
        for i, line in enumerate(desc_lines):
            s = self.font_small.render(line, True, (160, 170, 200))
            self.screen.blit(s, (dlg_x + 30, dlg_y + 65 + i * 22))

        # Các ô nhập liệu
        for i in range(self.n_waypoints):
            field_y = dlg_y + 125 + i * 85

            # Label waypoint
            label_color = COLOR_WP_ACTIVE if i == self.active_idx else COLOR_WP_PENDING
            label = self.font.render(f"Waypoint {i + 1}", True, label_color)
            self.screen.blit(label, (dlg_x + 30, field_y))

            # Ô nhập
            input_rect = pygame.Rect(dlg_x + 30, field_y + 30, dlg_w - 60, 38)
            border_color = COLOR_INPUT_ACTIVE if i == self.active_idx else (60, 70, 100)
            pygame.draw.rect(self.screen, COLOR_INPUT_BG, input_rect, border_radius=6)
            pygame.draw.rect(self.screen, border_color, input_rect, 2, border_radius=6)

            # Text trong ô
            display_text = self.inputs[i] if self.inputs[i] else self.placeholders[i]
            text_color = COLOR_HUD_VALUE if self.inputs[i] else (80, 90, 120)
            txt_surf = self.font.render(display_text, True, text_color)
            self.screen.blit(txt_surf, (input_rect.x + 10, input_rect.y + 8))

            # Cursor nhấp nháy
            if i == self.active_idx and (pygame.time.get_ticks() // 500) % 2 == 0:
                cursor_x = input_rect.x + 10 + txt_surf.get_width() + 2
                pygame.draw.line(self.screen, COLOR_HUD_VALUE,
                                 (cursor_x, input_rect.y + 6),
                                 (cursor_x, input_rect.y + 30), 2)

        # Thông báo lỗi
        if self.error_msg:
            err_surf = self.font_small.render(f"⚠  {self.error_msg}", True, COLOR_HUD_WARN)
            self.screen.blit(err_surf, (dlg_x + 30, dlg_y + 385))

        # Gợi ý phím
        hint = self.font_small.render(
            f"Đang nhập Waypoint {self.active_idx + 1}/{self.n_waypoints}",
            True, (120, 130, 160))
        self.screen.blit(hint, (dlg_x + dlg_w - 260, dlg_y + 385))


# =============================================================================
# RENDERER - Vẽ các thành phần lên màn hình
# =============================================================================

class Renderer:
    """
    Chịu trách nhiệm toàn bộ việc vẽ lên pygame Surface.
    Tách biệt khỏi logic để dễ thay thế (OpenGL, matplotlib, v.v.)
    """

    def __init__(self, screen: pygame.Surface, fonts: dict):
        self.screen = screen
        self.fonts  = fonts

        # Surface riêng cho trail (để vẽ fade)
        self.trail_surface = pygame.Surface(
            (SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)

    # =========================================================================
    # GRID
    # =========================================================================

    def draw_grid(self):
        """Vẽ lưới ô vuông dạng sa bàn robot."""
        # Tính phạm vi hiển thị
        x_min, y_min = screen_to_world(0, SCREEN_HEIGHT)
        x_max, y_max = screen_to_world(SCREEN_WIDTH, 0)

        # Lưới nhỏ (minor grid) - mỗi 0.1m
        minor_step = 0.1
        x = math.floor(x_min / minor_step) * minor_step
        while x <= x_max:
            sx, _ = world_to_screen(x, 0)
            if 0 <= sx <= SCREEN_WIDTH:
                pygame.draw.line(self.screen, COLOR_GRID_MINOR,
                                 (sx, 0), (sx, SCREEN_HEIGHT), 1)
            x += minor_step

        y = math.floor(y_min / minor_step) * minor_step
        while y <= y_max:
            _, sy = world_to_screen(0, y)
            if 0 <= sy <= SCREEN_HEIGHT:
                pygame.draw.line(self.screen, COLOR_GRID_MINOR,
                                 (0, sy), (SCREEN_WIDTH, sy), 1)
            y += minor_step

        # Lưới chính (major grid) - mỗi 0.5m
        x = math.floor(x_min / GRID_SIZE) * GRID_SIZE
        while x <= x_max:
            sx, _ = world_to_screen(x, 0)
            if 0 <= sx <= SCREEN_WIDTH:
                pygame.draw.line(self.screen, COLOR_GRID_MAJOR,
                                 (sx, 0), (sx, SCREEN_HEIGHT), 1)
                # Nhãn tọa độ X
                if abs(x) < 0.001:
                    continue
                lbl = self.fonts['small'].render(f"{x:.1f}", True, (80, 100, 140))
                self.screen.blit(lbl, (sx + 3, ORIGIN_Y + 5))
            x += GRID_SIZE

        y = math.floor(y_min / GRID_SIZE) * GRID_SIZE
        while y <= y_max:
            _, sy = world_to_screen(0, y)
            if 0 <= sy <= SCREEN_HEIGHT:
                pygame.draw.line(self.screen, COLOR_GRID_MAJOR,
                                 (0, sy), (SCREEN_WIDTH, sy), 1)
                if abs(y) < 0.001:
                    continue
                lbl = self.fonts['small'].render(f"{y:.1f}", True, (80, 100, 140))
                self.screen.blit(lbl, (ORIGIN_X + 5, sy - 18))
            y += GRID_SIZE

    def draw_axes(self):
        """Vẽ hệ trục tọa độ X-Y."""
        ox, oy = world_to_screen(0, 0)

        # Trục X (đỏ)
        pygame.draw.line(self.screen, COLOR_AXIS_X,
                         (ox, oy), (SCREEN_WIDTH - 20, oy), 2)
        # Mũi tên X
        self._draw_arrow_head(self.screen, COLOR_AXIS_X,
                              (SCREEN_WIDTH - 25, oy), (SCREEN_WIDTH - 10, oy))
        lbl_x = self.fonts['label'].render("X (m)", True, COLOR_AXIS_X)
        self.screen.blit(lbl_x, (SCREEN_WIDTH - 50, oy - 22))

        # Trục Y (xanh)
        pygame.draw.line(self.screen, COLOR_AXIS_Y,
                         (ox, oy), (ox, 20), 2)
        # Mũi tên Y
        self._draw_arrow_head(self.screen, COLOR_AXIS_Y,
                              (ox, 25), (ox, 10))
        lbl_y = self.fonts['label'].render("Y (m)", True, COLOR_AXIS_Y)
        self.screen.blit(lbl_y, (ox + 8, 10))

        # Nhãn gốc O
        lbl_o = self.fonts['label'].render("O", True, COLOR_AXIS_LABEL)
        self.screen.blit(lbl_o, (ox + 5, oy + 5))

    @staticmethod
    def _draw_arrow_head(surface, color, start, end, size=8):
        """Vẽ đầu mũi tên."""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.sqrt(dx * dx + dy * dy)
        if length < 1:
            return
        ux, uy = dx / length, dy / length
        # Hai cánh mũi tên
        left  = (end[0] - size * ux + size * 0.5 * uy,
                 end[1] - size * uy - size * 0.5 * ux)
        right = (end[0] - size * ux - size * 0.5 * uy,
                 end[1] - size * uy + size * 0.5 * ux)
        pygame.draw.polygon(surface, color, [end, left, right])

    # =========================================================================
    # TRAIL
    # =========================================================================

    def draw_trail(self, trail: list, show: bool = True):
        """Vẽ đường đi của robot với hiệu ứng fade."""
        if not show or len(trail) < 2:
            return

        n = len(trail)
        for i in range(1, n):
            # Fade màu theo thứ tự (cũ = mờ, mới = sáng)
            t = i / n
            r = int(COLOR_TRAIL_FADE[0] + t * (COLOR_TRAIL[0] - COLOR_TRAIL_FADE[0]))
            g = int(COLOR_TRAIL_FADE[1] + t * (COLOR_TRAIL[1] - COLOR_TRAIL_FADE[1]))
            b = int(COLOR_TRAIL_FADE[2] + t * (COLOR_TRAIL[2] - COLOR_TRAIL_FADE[2]))

            p1 = world_to_screen(*trail[i - 1])
            p2 = world_to_screen(*trail[i])
            pygame.draw.line(self.screen, (r, g, b), p1, p2, 2)

    # =========================================================================
    # WAYPOINTS
    # =========================================================================

    def draw_waypoints(self, wp_manager: WaypointManager):
        """Vẽ các waypoint và đường nối giữa chúng."""
        waypoints = wp_manager.waypoints
        if not waypoints:
            return

        # Vẽ đường nối waypoint
        if len(waypoints) > 1:
            pts = [world_to_screen(wx, wy) for wx, wy, _ in waypoints]
            pygame.draw.lines(self.screen, COLOR_WP_LINE, False, pts, 1)

        # Vẽ từng waypoint
        for i, (wx, wy, wtheta) in enumerate(waypoints):
            sx, sy = world_to_screen(wx, wy)

            # Màu theo trạng thái
            if wp_manager.reached_flags[i]:
                color = COLOR_WP_DONE
                radius = 8
            elif i == wp_manager.current_index:
                color = COLOR_WP_ACTIVE
                radius = 12
                # Vòng nhấp nháy
                pulse = int(4 + 3 * math.sin(pygame.time.get_ticks() * 0.005))
                pygame.draw.circle(self.screen, (*color, 80), (sx, sy), radius + pulse)
                pygame.draw.circle(self.screen, color, (sx, sy), radius + pulse, 1)
            else:
                color = COLOR_WP_PENDING
                radius = 8

            # Vẽ hình tròn waypoint
            pygame.draw.circle(self.screen, color, (sx, sy), radius)
            pygame.draw.circle(self.screen, (255, 255, 255), (sx, sy), radius, 1)

            # Vẽ hướng theta tại waypoint (mũi tên nhỏ)
            arrow_len = 20
            ex = sx + int(arrow_len * math.cos(wtheta))
            ey = sy - int(arrow_len * math.sin(wtheta))
            pygame.draw.line(self.screen, color, (sx, sy), (ex, ey), 2)

            # Label waypoint
            lbl = self.fonts['label'].render(f"WP{i + 1}", True, COLOR_WP_LABEL)
            self.screen.blit(lbl, (sx + 12, sy - 18))

            # Hiển thị tọa độ
            coord = self.fonts['small'].render(
                f"({wx:.1f}, {wy:.1f}, {math.degrees(wtheta):.0f}°)",
                True, (160, 170, 200))
            self.screen.blit(coord, (sx + 12, sy))

    # =========================================================================
    # ROBOT
    # =========================================================================

    def draw_robot(self, robot: Robot, waypoint_manager: WaypointManager,
                   show_cross: bool = False):
        """
        Vẽ robot hình chữ nhật có hướng, 4 bánh, mũi tên heading.

        Args:
            robot           : Robot object
            waypoint_manager: để biết dấu X ở đâu
            show_cross      : hiện dấu X tại tâm khi đến waypoint
        """
        sx, sy = world_to_screen(robot.x, robot.y)
        theta  = robot.theta

        # Kích thước robot theo pixel
        L_px = int(ROBOT_DRAW_LENGTH * PIXELS_PER_METER)
        W_px = int(ROBOT_DRAW_WIDTH  * PIXELS_PER_METER)

        # ── Vẽ body robot (hình chữ nhật xoay) ──────────────────────────────
        body_pts = self._rotated_rect(sx, sy, L_px, W_px, theta)
        pygame.draw.polygon(self.screen, COLOR_ROBOT_BODY, body_pts)
        pygame.draw.polygon(self.screen, (100, 160, 220), body_pts, 2)

        # ── Vẽ đầu xe (hình chữ nhật nhỏ phía trước, màu sáng hơn) ──────────
        front_len = L_px * 0.35
        front_offset = L_px * 0.5 - front_len * 0.5  # đẩy về phía trước
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # Tâm phần đầu xe (offset về phía trước)
        front_cx = sx + int(cos_t * (L_px * 0.32))
        front_cy = sy - int(sin_t * (L_px * 0.32))
        front_pts = self._rotated_rect(front_cx, front_cy,
                                       int(front_len), int(W_px * 0.9), theta)
        pygame.draw.polygon(self.screen, COLOR_ROBOT_FRONT, front_pts)

        # ── Vẽ 4 bánh xe ──────────────────────────────────────────────────────
        wheel_L = int(0.055 * PIXELS_PER_METER)  # chiều dài bánh
        wheel_W = int(0.020 * PIXELS_PER_METER)  # chiều rộng bánh

        # Offset vị trí bánh
        hL = L_px * 0.35   # nửa chiều dài (offset dọc)
        hW = W_px * 0.5    # nửa chiều rộng (offset ngang)

        # Hướng ngang (vuông góc với hướng tiến)
        perp_cos = -sin_t
        perp_sin =  cos_t

        wheel_positions = [
            # (offset_dọc, offset_ngang)
            (+hL, +hW),   # trái trước  (ZQ)
            (-hL, +hW),   # trái sau    (ZH)
            (+hL, -hW),   # phải trước  (YQ)
            (-hL, -hW),   # phải sau    (YH)
        ]

        for (dl, dw) in wheel_positions:
            wx_screen = sx + int(cos_t * dl + perp_cos * dw)
            wy_screen = sy - int(sin_t * dl - perp_sin * dw)
            wpts = self._rotated_rect(wx_screen, wy_screen, wheel_L, wheel_W, theta)
            pygame.draw.polygon(self.screen, COLOR_ROBOT_WHEEL, wpts)
            pygame.draw.polygon(self.screen, (220, 220, 230), wpts, 1)

        # ── Vẽ mũi tên heading ────────────────────────────────────────────────
        arrow_len = int(L_px * 0.6)
        ax = sx + int(cos_t * arrow_len)
        ay = sy - int(sin_t * arrow_len)
        pygame.draw.line(self.screen, COLOR_HEADING_ARROW, (sx, sy), (ax, ay), 3)
        self._draw_arrow_head(self.screen, COLOR_HEADING_ARROW, (sx, sy), (ax, ay), size=10)

        # ── Dấu X tại tâm (khi đến waypoint) ─────────────────────────────────
        if show_cross:
            cross_size = 10
            pygame.draw.line(self.screen, COLOR_ROBOT_CROSS,
                             (sx - cross_size, sy - cross_size),
                             (sx + cross_size, sy + cross_size), 3)
            pygame.draw.line(self.screen, COLOR_ROBOT_CROSS,
                             (sx + cross_size, sy - cross_size),
                             (sx - cross_size, sy + cross_size), 3)

    @staticmethod
    def _rotated_rect(cx: int, cy: int, length: int, width: int,
                      angle: float) -> list:
        """
        Tính 4 đỉnh hình chữ nhật xoay quanh tâm (cx, cy).

        Chú ý: Pygame Y đảo, nên sin phải có dấu âm.
        """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        hw = width  // 2
        hl = length // 2

        # 4 góc theo hệ local (trước-phải)
        corners = [
            (+hl, +hw),
            (+hl, -hw),
            (-hl, -hw),
            (-hl, +hw),
        ]

        pts = []
        for (lx, ly) in corners:
            # Xoay và chuyển về hệ màn hình
            sx =  lx * cos_a - ly * sin_a
            sy = -lx * sin_a - ly * cos_a   # đảo dấu Y cho pygame
            pts.append((int(cx + sx), int(cy + sy)))
        return pts

    # =========================================================================
    # HUD
    # =========================================================================

    def draw_hud(self, robot: Robot, ctrl: PoseController,
                 wp_manager: WaypointManager,
                 fps: float, paused: bool, show_trail: bool):
        """
        Vẽ HUD (Heads-Up Display) hiển thị thông số realtime.

        Layout:
          ┌───────────────────────┐
          │  ROBOT STATE          │
          │  x, y, theta          │
          │─────────────────────  │
          │  VELOCITY             │
          │  v, omega             │
          │  v_left, v_right      │
          │─────────────────────  │
          │  WHEEL SPEED          │
          │  omega_zq/zh/yq/yh    │
          │─────────────────────  │
          │  CONTROLLER           │
          │  rho, alpha, theta_e  │
          │─────────────────────  │
          │  SYSTEM               │
          │  waypoint, fps        │
          └───────────────────────┘
        """
        state = robot.get_state_dict()
        ctrl_info = ctrl.get_debug_info()
        wp_status = wp_manager.get_status()

        # Nền semi-transparent
        hud_w, hud_h = 290, 420
        hud_x, hud_y = 10, 10
        hud_surf = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        hud_surf.fill((15, 25, 50, 210))
        pygame.draw.rect(hud_surf, (80, 100, 160, 180),
                         (0, 0, hud_w, hud_h), 1, border_radius=10)
        self.screen.blit(hud_surf, (hud_x, hud_y))

        y_off = hud_y + 10
        pad_x = hud_x + 12

        def section_title(text):
            nonlocal y_off
            surf = self.fonts['label'].render(text, True, COLOR_HUD_TITLE)
            self.screen.blit(surf, (pad_x, y_off))
            y_off += 20
            pygame.draw.line(self.screen, (60, 80, 130),
                             (pad_x, y_off - 3), (hud_x + hud_w - 12, y_off - 3), 1)

        def data_row(label, value, unit="", warn=False):
            nonlocal y_off
            color_val = COLOR_HUD_WARN if warn else COLOR_HUD_VALUE
            lbl_s = self.fonts['small'].render(f"{label}:", True, COLOR_HUD_TEXT)
            val_s = self.fonts['hud'].render(f"{value}  {unit}", True, color_val)
            self.screen.blit(lbl_s, (pad_x, y_off))
            self.screen.blit(val_s, (pad_x + 110, y_off))
            y_off += 18

        # ── ROBOT STATE ───────────────────────────────────────────────────────
        section_title("  ROBOT POSE")
        data_row("x",     f"{state['x']:+.4f}", "m")
        data_row("y",     f"{state['y']:+.4f}", "m")
        data_row("theta", f"{math.degrees(state['theta']):+.2f}", "°")
        y_off += 4

        # ── VELOCITY ──────────────────────────────────────────────────────────
        section_title("  VELOCITY  (Twist)")
        data_row("v (linear.x)",    f"{state['v']:+.4f}",     "m/s",
                 abs(state['v']) > V_MAX * 0.9)
        data_row("ω (angular.z)",   f"{state['omega']:+.4f}", "r/s",
                 abs(state['omega']) > OMEGA_MAX * 0.9)
        data_row("v_left",          f"{state['v_left']:+.4f}",  "m/s")
        data_row("v_right",         f"{state['v_right']:+.4f}", "m/s")
        y_off += 4

        # ── WHEEL SPEED ───────────────────────────────────────────────────────
        section_title("  WHEEL ω  [rad/s]")
        data_row("ω_ZQ (LF)", f"{state['omega_zq']:+.3f}", "r/s")
        data_row("ω_ZH (LR)", f"{state['omega_zh']:+.3f}", "r/s")
        data_row("ω_YQ (RF)", f"{state['omega_yq']:+.3f}", "r/s")
        data_row("ω_YH (RR)", f"{state['omega_yh']:+.3f}", "r/s")
        y_off += 4

        # ── CONTROLLER ────────────────────────────────────────────────────────
        section_title("  CONTROLLER")
        data_row("rho   (dist)", f"{ctrl_info['rho']:.4f}",
                 "m", ctrl_info['rho'] < RHO_THRESHOLD)
        data_row("alpha (head)", f"{math.degrees(ctrl_info['alpha']):+.2f}", "°")
        data_row("θ_err (goal)", f"{math.degrees(ctrl_info['theta_error']):+.2f}",
                 "°", abs(ctrl_info['theta_error']) < THETA_THRESHOLD)
        y_off += 4

        # ── SYSTEM ────────────────────────────────────────────────────────────
        section_title("  SYSTEM")
        wp_idx = wp_status['current_index']
        wp_tot = wp_status['total']
        status_str = "DONE ✓" if wp_status['completed'] else f"{wp_idx + 1}/{wp_tot}"
        data_row("Waypoint", status_str)
        data_row("FPS",      f"{fps:.0f}")

        # Trạng thái pause/trail
        state_strs = []
        if paused:     state_strs.append("⏸ PAUSED")
        if show_trail: state_strs.append("TRAIL ✓")
        if state_strs:
            s = self.fonts['small'].render("  ".join(state_strs), True, COLOR_HUD_TITLE)
            self.screen.blit(s, (pad_x, y_off))
            y_off += 18

    def draw_controls_hint(self):
        """Vẽ gợi ý phím điều khiển ở góc dưới phải."""
        hints = [
            "R: Reset   P: Pause   T: Trail   ESC: Quit",
        ]
        for i, h in enumerate(hints):
            s = self.fonts['small'].render(h, True, (100, 110, 140))
            self.screen.blit(s, (SCREEN_WIDTH - s.get_width() - 10,
                                  SCREEN_HEIGHT - 20 - (len(hints) - 1 - i) * 18))


# =============================================================================
# SIMULATOR - Vòng lặp chính
# =============================================================================

class Simulator:
    """
    Vòng lặp chính của mô phỏng.

    Pipeline mỗi frame:
      1. Xử lý sự kiện pygame
      2. Cập nhật vật lý: controller → robot.update()
      3. Kiểm tra waypoint hoàn thành → advance hoặc stop
      4. Render: grid → trail → waypoints → robot → HUD
    """

    def __init__(self):
        """Khởi tạo pygame và các thành phần."""
        pygame.init()
        pygame.font.init()

        # ── Màn hình ──────────────────────────────────────────────────────────
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("🤖 Robot Differential Drive - PID Simulation")

        # ── Font ──────────────────────────────────────────────────────────────
        try:
            # Ưu tiên font hệ thống đẹp hơn
            font_hud   = pygame.font.SysFont("Consolas", FONT_SIZE_HUD,   bold=False)
            font_label = pygame.font.SysFont("Consolas", FONT_SIZE_LABEL, bold=True)
            font_small = pygame.font.SysFont("Consolas", FONT_SIZE_LABEL - 2)
            font_title = pygame.font.SysFont("Consolas", FONT_SIZE_TITLE, bold=True)
            font_dialog = pygame.font.SysFont("Consolas", FONT_SIZE_DIALOG)
        except Exception:
            font_hud = font_label = font_small = font_title = font_dialog = \
                pygame.font.Font(None, 20)

        self.fonts = {
            'hud':    font_hud,
            'label':  font_label,
            'small':  font_small,
            'title':  font_title,
            'dialog': font_dialog,
        }

        # ── Clock ─────────────────────────────────────────────────────────────
        self.clock = pygame.time.Clock()
        self.fps   = 0.0

        # ── State flags ───────────────────────────────────────────────────────
        self.running    = True
        self.paused     = False
        self.show_trail = True
        self.show_cross = False    # hiện dấu X khi đến waypoint
        self.cross_timer = 0       # countdown hiển thị dấu X

        # ── Khởi tạo tạm thời (sẽ reset sau khi nhập waypoint) ───────────────
        self.robot      = Robot(DEFAULT_START_X, DEFAULT_START_Y, DEFAULT_START_THETA)
        self.controller = PoseController()
        self.wp_manager = WaypointManager(DEFAULT_WAYPOINTS)

        # ── Renderer ──────────────────────────────────────────────────────────
        self.renderer = Renderer(self.screen, self.fonts)

        # ── Input dialog ──────────────────────────────────────────────────────
        self.dialog: Optional[WaypointInputDialog] = None
        self._show_input_dialog()

    # =========================================================================
    # INPUT DIALOG
    # =========================================================================

    def _show_input_dialog(self):
        """Hiển thị dialog nhập waypoint."""
        self.dialog = WaypointInputDialog(
            self.screen,
            self.fonts['dialog'],
            self.fonts['small'],
            n_waypoints=3
        )

    def _reset_simulation(self, waypoints: list):
        """Reset toàn bộ mô phỏng với waypoint mới."""
        self.robot      = Robot(DEFAULT_START_X, DEFAULT_START_Y, DEFAULT_START_THETA)
        self.controller = PoseController()
        self.wp_manager = WaypointManager(waypoints)
        self.show_cross = False
        self.cross_timer = 0

    # =========================================================================
    # UPDATE
    # =========================================================================

    def update(self, dt: float):
        """
        Cập nhật vật lý mô phỏng mỗi bước.

        Pipeline:
          goal → controller.compute_control → (v, omega) → robot.update

        Args:
            dt (float): bước thời gian [s]
        """
        if self.paused or self.wp_manager.completed:
            # Robot vẫn giữ nguyên vị trí khi dừng
            self.robot.update(0.0, 0.0, dt)
            return

        # ── Lấy mục tiêu hiện tại ────────────────────────────────────────────
        goal = self.wp_manager.current_goal

        # ── Tính lệnh điều khiển ─────────────────────────────────────────────
        v, omega = self.controller.compute_control(self.robot.pose, goal)

        # ── Cập nhật trạng thái robot ─────────────────────────────────────────
        self.robot.update(v, omega, dt)

        # ── Kiểm tra hoàn thành waypoint ─────────────────────────────────────
        if self.controller.is_goal_reached(self.robot.pose, goal):
            # Hiện dấu X tại tâm
            self.show_cross = True
            self.cross_timer = 90  # frames

            # Chuyển sang waypoint tiếp theo
            self.wp_manager.advance()

            if self.wp_manager.completed:
                print(f"\n✅ Hoàn thành tất cả {self.wp_manager.total} waypoint!")
                print(f"   Pose cuối: x={self.robot.x:.4f}m, "
                      f"y={self.robot.y:.4f}m, θ={math.degrees(self.robot.theta):.2f}°")
            else:
                idx = self.wp_manager.current_index
                print(f"✓ Waypoint {idx} xong → Hướng đến WP{idx + 1}: "
                      f"{self.wp_manager.current_goal}")

        # Đếm ngược timer dấu X
        if self.cross_timer > 0:
            self.cross_timer -= 1
        if self.cross_timer == 0:
            self.show_cross = False

    # =========================================================================
    # RENDER
    # =========================================================================

    def render(self):
        """Vẽ toàn bộ khung hình."""
        self.screen.fill(COLOR_BACKGROUND)

        # ── Grid & Axes ───────────────────────────────────────────────────────
        self.renderer.draw_grid()
        self.renderer.draw_axes()

        # ── Trail ─────────────────────────────────────────────────────────────
        self.renderer.draw_trail(self.robot.trail, self.show_trail)

        # ── Waypoints ─────────────────────────────────────────────────────────
        if self.dialog is None:  # chỉ vẽ sau khi xong dialog
            self.renderer.draw_waypoints(self.wp_manager)

        # ── Robot ─────────────────────────────────────────────────────────────
        self.renderer.draw_robot(self.robot, self.wp_manager,
                                  show_cross=self.show_cross)

        # ── HUD ───────────────────────────────────────────────────────────────
        if self.dialog is None:
            self.renderer.draw_hud(
                self.robot, self.controller, self.wp_manager,
                self.fps, self.paused, self.show_trail
            )
            self.renderer.draw_controls_hint()

        # ── Banner hoàn thành ─────────────────────────────────────────────────
        if self.wp_manager.completed:
            banner = self.fonts['title'].render(
                "✅  ĐÃ HOÀN THÀNH TẤT CẢ WAYPOINT  —  Nhấn R để reset",
                True, (80, 255, 160))
            bx = (SCREEN_WIDTH - banner.get_width()) // 2
            self.screen.blit(banner, (bx, SCREEN_HEIGHT // 2 - 20))

        # ── Dialog nhập liệu ──────────────────────────────────────────────────
        if self.dialog is not None:
            self.dialog.render()

        pygame.display.flip()

    # =========================================================================
    # MAIN LOOP
    # =========================================================================

    def run(self):
        """Vòng lặp chính."""
        while self.running:
            # ── Xử lý sự kiện ────────────────────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

                elif event.type == pygame.KEYDOWN:
                    # Nếu dialog đang mở
                    if self.dialog is not None:
                        done = self.dialog.handle_event(event)
                        if done:
                            self._reset_simulation(self.dialog.waypoints)
                            self.dialog = None
                    else:
                        # Phím điều khiển chính
                        if event.key == pygame.K_ESCAPE:
                            self.running = False

                        elif event.key == pygame.K_r:
                            # Reset → mở lại dialog nhập waypoint
                            self._show_input_dialog()

                        elif event.key == pygame.K_p:
                            # Pause / Resume
                            self.paused = not self.paused
                            print("⏸ Paused" if self.paused else "▶ Resumed")

                        elif event.key == pygame.K_t:
                            # Toggle trail
                            self.show_trail = not self.show_trail

                else:
                    # Chuyển event cho dialog (mouse click v.v.)
                    if self.dialog is not None:
                        self.dialog.handle_event(event)

            # ── Update vật lý ─────────────────────────────────────────────────
            if self.dialog is None:
                for _ in range(SIM_STEPS_PER_FRAME):
                    self.update(DT)

            # ── Render ────────────────────────────────────────────────────────
            self.render()

            # ── FPS ───────────────────────────────────────────────────────────
            self.clock.tick(FPS)
            self.fps = self.clock.get_fps()

        pygame.quit()
        sys.exit(0)
