# Lý Thuyết: Trích Xuất Waypoints Làn Đường Qua Homography

## 1. Bài Toán

Camera gắn trên xe nhìn xuống đường. Ảnh camera bị **méo phối cảnh** (perspective distortion): vật ở xa trông nhỏ hơn vật ở gần, các đường song song hội tụ về 1 điểm.

**Mục tiêu:** Chuyển tọa độ pixel `(u, v)` trên ảnh camera thành tọa độ thực `(X_mm, Y_mm)` trên mặt đường, đơn vị milimet, phục vụ trích xuất waypoints và fit polynomial làn đường cho bộ điều khiển xe.

---

## 2. Homography Là Gì?

Homography là một **phép biến đổi hình học** ánh xạ điểm từ mặt phẳng này sang mặt phẳng khác. Trong trường hợp của chúng ta:

- **Mặt phẳng nguồn:** Ảnh camera (pixel)
- **Mặt phẳng đích:** Mặt đường thực (mm)

Phép biến đổi được biểu diễn bằng **ma trận H kích thước 3×3**:

```
┌ X_mm ┐       ┌ h11  h12  h13 ┐   ┌ u ┐
│ Y_mm │ = s · │ h21  h22  h23 │ × │ v │
└  1   ┘       └ h31  h32  h33 ┘   └ 1 ┘

Trong đó:
  (u, v)       = tọa độ pixel trên ảnh camera
  (X_mm, Y_mm) = tọa độ thực trên mặt đường (mm)
  s            = hệ số tỉ lệ (tính tự động)
  H            = ma trận homography 3x3
```

Sau khi nhân ma trận:
```
X_mm = (h11·u + h12·v + h13) / (h31·u + h32·v + h33)
Y_mm = (h21·u + h22·v + h23) / (h31·u + h32·v + h33)
```

---

## 3. Cách Tính Ma Trận H (Calibration)

Ma trận H có 8 ẩn số (h33 chuẩn hóa = 1). Cần tối thiểu **4 cặp điểm** tương ứng giữa ảnh và thực tế.

### Ví dụ minh họa:

Đặt một vật hình chữ nhật (kích thước đã biết) trên mặt đường, trước camera:

```
Anh camera (pixel):                Mat duong thuc (mm):

    P1o---------oP2                P1o-----------oP2
     \           /                 |              |
      \         /     -------->    |              |
       \       /        H          |              |
    P4o---------oP3                P4o-----------oP3

  Hinh thang (do phoi canh)        Hinh chu nhat (kich thuoc thuc)
```

| Điểm | Pixel (u, v) | Thực (X_mm, Y_mm) |
|------|-------------|-------------------|
| P1 | (185, 120) | (-150, 1000) |
| P2 | (455, 120) | (150, 1000) |
| P3 | (410, 350) | (150, 500) |
| P4 | (230, 350) | (-150, 500) |

OpenCV tính H từ 4 cặp điểm này:
```cpp
Mat H = getPerspectiveTransform(src_points, dst_points);
```

---

## 4. Hệ Tọa Độ Thực (Vehicle-Centric)

```
      Y (mm) — huong truoc xe (doc / longitudinal)
      ^
      |    <- lan duong trai dai theo Y
      |
      O----------> X (mm) — sang phai (ngang / lateral)
    (0,0)
    = diem chieu camera xuong mat duong / dau xe
```

- **X (mm):** Độ lệch ngang (lateral offset). X > 0 = sang phải, X < 0 = sang trái.
- **Y (mm):** Khoảng cách phía trước xe (longitudinal). Y > 0 = phía trước.

### Phạm vi tọa độ thực tế trong hệ thống:

| Trục | Phạm vi | Ý nghĩa |
|------|---------|---------|
| **X** | -1000mm đến +1000mm | Chiều ngang tối đa 2 mét |
| **Y** | 0mm đến 3500mm | Tầm nhìn phía trước tối đa 3.5 mét |

---

## 5. Trích Xuất Waypoints Từ Polygon Phân Đoạn

Segmentation model cho ra **polygon contour** của từng làn đường (`main-lane`, `other-lane`, `turn-lane`). Để fit polynomial, cần trích xuất **đường trung tâm (centerline)** trong tọa độ thực.

### Phương pháp: Ray-casting Intersection (Scan Line)

Hệ thống hoạt động trực tiếp trên **polygon real-world** sau khi đã transform qua H (không quét pixel mask từng hàng).

#### 5.1 Chuyển polygon pixel → tọa độ mm

Với mỗi điểm `(u, v)` trong polygon:
```
w = h31·u + h32·v + h33
X = (h11·u + h12·v + h13) / w
Y = (h21·u + h22·v + h23) / w
```
Điều kiện: `|w| > 1e-6` (tránh chia cho 0). Kết quả được làm tròn đến 0.1mm.

#### 5.2 Quét dọc (Y-sweep) — cho main-lane / other-lane

Làn đường thẳng trải dài theo Y. Với mỗi mức Y (bước 100mm):
```
Tai Y = y_i, tim tat ca X giao voi canh polygon:
  - Voi moi canh (p1->p2):
    neu y1 <= y_i <= y2 (hoac y2 <= y_i <= y1):
      X_intersect = x1 + (y_i - y1) * (x2 - x1) / (y2 - y1)
  - X_center = (X_min + X_max) / 2
  -> waypoint: (X_center, y_i)
```

#### 5.3 Quét ngang (X-sweep) — cho turn-lane

Làn rẽ trải dài theo X. Với mỗi mức X (bước 100mm):
```
Tai X = x_i, tim tat ca Y giao voi canh polygon:
  - Y_center = (Y_min + Y_max) / 2
  -> waypoint: (x_i, Y_center)
```

---

## 6. Fit Đa Thức Bậc 3

### 6.1 Polynomial x(y) — main-lane / other-lane

Làn đường thẳng: biểu diễn X ngang theo hàm của Y dọc:
```
x(y) = a3·y^3 + a2·y^2 + a1·y + a0
```

### 6.2 Polynomial y(x) — turn-lane

Làn rẽ: biểu diễn Y theo X:
```
y(x) = a3·x^3 + a2·x^2 + a1·x + a0
```

### Ý nghĩa các hệ số tại vị trí xe (tại t = 0):

| Hệ số | Ý nghĩa | Công thức |
|-------|---------|-----------|
| **a0** | **Lateral offset** — Độ lệch ngang tại gốc (mm) | `x(0) = a0` |
| **a1** | **Heading angle** — Góc giữa hướng xe và đường (rad) | `psi = arctan(a1)` |
| **a2** | **Curvature** — Độ cong tại vị trí xe (1/mm) | `kappa = 2·a2` |
| **a3** | **Rate of curvature change** | Dùng cho dự đoán xa |

### 6.3 Phương pháp Least Squares (SVD)

Cho N điểm `{(ti, si)}`, tìm `[a3, a2, a1, a0]` sao cho tổng bình phương sai số nhỏ nhất. Viết dưới dạng ma trận `A · c = b`:

```
    [ t1^3  t1^2  t1  1 ]       [ a3 ]       [ s1 ]
A = | t2^3  t2^2  t2  1 |,  c = | a2 |,  b = | s2 |
    |  :     :    :   : |       | a1 |       |  : |
    [ tn^3  tn^2  tn  1 ]       [ a0 ]       [ sn ]
```

Dùng **SVD** (Singular Value Decomposition) — ổn định hơn QR với dữ liệu thực tế nhiễu:
```cpp
cv::solve(A, b, coeffs, cv::DECOMP_SVD);
```

Fallback khi ít hơn 4 điểm (dùng bậc 1 thay vì bậc 3):
```cpp
cv::solve(A_2col, b, coeffs, cv::DECOMP_SVD);  // chi a1, a0
```

---

## 7. Spatial & Temporal Smoothing

### 7.1 Spatial Smoothing (3-point moving average)

Trước khi fit polynomial, làm mượt waypoints thô để giảm nhiễu từ contour polygon:
```
smoothed[i].s = (raw[i-1].s + raw[i].s + raw[i+1].s) / 3
```
Áp dụng cho các điểm nội, giữ nguyên endpoint.

### 7.2 Temporal Smoothing (EMA — Exponential Moving Average)

Sau khi fit polynomial, áp dụng EMA trên các hệ số qua các frame liên tiếp:
```
coeff[t] = alpha * coeff_raw[t] + (1 - alpha) * coeff[t-1]
```
Với `alpha = 0.25` (25% frame mới, 75% lịch sử) — làm mượt đường cong, giảm rung lắc đột ngột.

Tương tự cho control metrics:
```
lateral_offset  = alpha * lateral_offset  + (1-alpha) * prev_lateral_offset
heading_angle   = alpha * heading_angle   + (1-alpha) * prev_heading_angle
curvature       = alpha * curvature       + (1-alpha) * prev_curvature
```

---

## 8. Regenerate Waypoints Từ Polynomial Đã Làm Mượt

Sau khi có polynomial đã temporal-smooth, **tái tạo lại waypoints** từ polynomial để đảm bảo đường cong mượt và nhất quán (không dùng waypoints thô ban đầu):

```
Voi main-lane / other-lane (sweep Y, buoc 100mm):
  x_val = a3·y^3 + a2·y^2 + a1·y + a0
  -> smooth_waypoint: (x_val, y)

Voi turn-lane (sweep X, buoc 100mm):
  y_val = a3·x^3 + a2·x^2 + a1·x + a0
  -> smooth_waypoint: (x, y_val)
```

Những waypoints mượt này được publish lên `/avs/telemetry_realworld` để:
- **BEV Canvas** (frontend) hiển thị đường cong polyline màu theo class
- **Bộ điều khiển** downstream sử dụng `lateral_offset_mm`, `heading_angle_rad`, `curvature_inv_mm`

---

## 9. Giả Định & Hạn Chế

| Giả định | Hệ quả |
|----------|--------|
| Mặt đường phẳng | Nếu mặt đường có độ dốc, tọa độ mm sẽ bị sai |
| Camera cố định trên xe | Nếu camera bị xê dịch, cần calibrate lại |
| Vùng nhìn thấy hữu hạn | Polynomial chỉ chính xác trong phạm vi camera (~3.5m) |
| Polygon đủ lớn | Làn bị che khuất một phần → ít waypoints → polynomial kém chính xác |

Với dự án hiện tại (băng rôn in trên sàn phẳng), tất cả giả định đều thỏa mãn.
