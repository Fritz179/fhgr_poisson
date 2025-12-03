import cv2
import numpy as np

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def draw_legacy_gauge(center, radius, img, roll, pitch, throttle):
    """Scaled legacy gauge showing roll/pitch vector and throttle fill."""
    x, y = center
    radius = max(10, int(radius))
    outline_thickness = max(2, radius // 40)
    cv2.circle(img, center, radius, (51, 51, 51), outline_thickness)  # Outline

    throttle_color = (255, 255, 255) if throttle >= 0 else (255, 0, 0)
    throttle_radius = int(round(min(1.0, abs(throttle) / 100.0) * (radius - outline_thickness * 2)))
    if throttle_radius > 0:
        cv2.circle(img, center, throttle_radius, throttle_color, -1)  # Throttle fill

    center_radius = max(4, radius // 25)
    cv2.circle(img, center, center_radius, (0, 0, 255), -1)  # Center

    vector_scale = radius * 0.7 / 100.0
    dx = int(round(roll * vector_scale))
    dy = int(round(pitch * vector_scale))
    arrow_thickness = max(2, radius // 35)
    cv2.arrowedLine(img, (x, y), (x + dx, y + dy), (0, 255, 0), arrow_thickness, cv2.LINE_AA, 0, 0.2)

    tick_radius = radius - outline_thickness * 2
    tick_length = max(4, radius // 12)
    tick_thickness = max(1, outline_thickness - 1)
    for angle in range(0, 360, 45):
        rad = np.deg2rad(angle)
        outer = (int(x + tick_radius * np.cos(rad)), int(y + tick_radius * np.sin(rad)))
        inner = (int(x + (tick_radius - tick_length) * np.cos(rad)), int(y + (tick_radius - tick_length) * np.sin(rad)))
        cv2.line(img, inner, outer, (100, 100, 100), tick_thickness, cv2.LINE_AA)


def draw_attitude_indicator(center, radius, img, roll, pitch):
    """Cessna-style attitude indicator with artificial horizon."""
    x, y = center
    radius = max(20, int(radius))
    size = int(radius * 2.4)
    half = size // 2
    cx = cy = half

    overlay = np.zeros((size, size, 3), dtype=np.uint8)

    pitch_norm = clamp(pitch, -100, 100) / 100.0
    pitch_shift = int(round(pitch_norm * radius * 0.7))
    horizon_y = cy + pitch_shift

    sky_color = (240, 220, 180)  # light blue-ish (BGR)
    ground_color = (70, 90, 140)  # brown/earth tone (BGR)
    cv2.rectangle(overlay, (0, 0), (size, horizon_y), sky_color, -1)
    cv2.rectangle(overlay, (0, horizon_y), (size, size), ground_color, -1)

    horizon_thickness = max(2, radius // 35)
    cv2.line(overlay, (0, horizon_y), (size, horizon_y), (255, 255, 255), horizon_thickness, cv2.LINE_AA)

    pitch_range_px = radius * 0.8
    for offset in range(-80, 81, 20):
        offset_px = int(round(offset / 100.0 * pitch_range_px))
        y_line = horizon_y - offset_px
        if 0 <= y_line < size:
            line_len = int(radius * (0.55 if offset % 40 == 0 else 0.4))
            cv2.line(
                overlay,
                (cx - line_len, y_line),
                (cx + line_len, y_line),
                (255, 255, 255),
                max(1, radius // 45),
                cv2.LINE_AA,
            )

    roll_angle = clamp(roll, -100, 100) * 0.7  # scale to reasonable bank
    rot_mat = cv2.getRotationMatrix2D((cx, cy), -roll_angle, 1.0)
    rotated = cv2.warpAffine(
        overlay,
        rot_mat,
        (size, size),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )

    mask = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(mask, (cx, cy), radius, 255, -1)

    img_h, img_w = img.shape[:2]
    x1 = x - half
    y1 = y - half
    x2 = x1 + size
    y2 = y1 + size
    ox1 = max(0, -x1)
    oy1 = max(0, -y1)
    ox2 = size - max(0, x2 - img_w)
    oy2 = size - max(0, y2 - img_h)
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(img_w, x2)
    y2 = min(img_h, y2)

    overlay_roi = rotated[oy1:oy2, ox1:ox2]
    mask_roi = mask[oy1:oy2, ox1:ox2]
    roi = img[y1:y2, x1:x2]
    mask_bool = mask_roi > 0
    roi[mask_bool] = overlay_roi[mask_bool]

    ring_thickness = max(2, radius // 25)
    cv2.circle(img, center, radius, (255, 255, 255), ring_thickness, cv2.LINE_AA)

    tick_len = radius * 0.12
    tick_outer = radius
    tick_inner = tick_outer - tick_len
    tick_thickness = max(2, radius // 35)
    for angle in (-60, -45, -30, -20, 0, 20, 30, 45, 60):
        rad = np.deg2rad(angle)
        outer = (int(x + tick_outer * np.sin(rad)), int(y - tick_outer * np.cos(rad)))
        inner = (int(x + tick_inner * np.sin(rad)), int(y - tick_inner * np.cos(rad)))
        cv2.line(img, inner, outer, (255, 255, 255), tick_thickness, cv2.LINE_AA)

    wing_span = int(radius * 0.9)
    body_height = int(radius * 0.25)
    symbol_thickness = max(2, radius // 30)
    cv2.line(img, (x - wing_span // 2, y), (x + wing_span // 2, y), (0, 200, 255), symbol_thickness, cv2.LINE_AA)
    cv2.line(img, (x, y), (x, y + body_height), (0, 200, 255), symbol_thickness, cv2.LINE_AA)
    cv2.circle(img, center, max(3, radius // 40), (0, 200, 255), -1)
