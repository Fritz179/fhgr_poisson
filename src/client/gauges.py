import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as R

ROLL_MAX = 180.0
PITCH_MAX = 90.0
THROTTLE_MAX = 100.0
YAW_MAX = 180.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def draw_compass(center, radius, img, rot: R):
    """Simple compass: 0 = North (up), yaw positive to the right (clockwise)."""
    x, y = center
    radius = max(20, int(radius))
    outline_thickness = max(2, radius // 25)
    cv2.circle(img, center, radius, (255, 255, 255), outline_thickness, cv2.LINE_AA)

    # Ticks every 30 deg
    tick_outer = radius - outline_thickness // 2
    long_len = max(6, radius // 6)
    short_len = max(3, radius // 10)
    tick_thickness = max(1, outline_thickness - 1)
    for angle in range(0, 360, 30):
        rad = np.deg2rad(angle)
        outer = (int(x + tick_outer * np.cos(rad)), int(y + tick_outer * np.sin(rad)))
        inner_len = long_len if angle % 90 == 0 else short_len
        inner = (int(x + (tick_outer - inner_len) * np.cos(rad)), int(y + (tick_outer - inner_len) * np.sin(rad)))
        cv2.line(img, inner, outer, (180, 180, 180), tick_thickness, cv2.LINE_AA)

    # Labels
    labels = [(0, "E"), (90, "S"), (180, "W"), (270, "N")]
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", max(10, radius // 5))
    except OSError:
        font = ImageFont.load_default()
    pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)
    for angle, label in labels:
        rad = np.deg2rad(angle)
        tx = x + int((tick_outer - long_len * 1.5) * np.cos(rad))
        ty = y + int((tick_outer - long_len * 1.5) * np.sin(rad)) - font.size // 2
        draw.text((tx - font.getlength(label) / 2, ty), label, font=font, fill=(255, 255, 255))
    img[:] = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

    # Pointer (yaw 0 = north/up, positive clockwise)
    yaw = rot.as_euler("xyz", degrees=True)[2]
    clamped_yaw = clamp(yaw, -YAW_MAX, YAW_MAX)
    rad = np.deg2rad(clamped_yaw)
    dx = int(round(radius * 0.75 * np.sin(rad)))
    dy = int(round(radius * 0.75 * -np.cos(rad)))
    pointer_thickness = max(2, radius // 20)
    cv2.arrowedLine(img, center, (x + dx, y + dy), (0, 200, 255), pointer_thickness, cv2.LINE_AA, 0, 0.25)


def draw_legacy_gauge(center, radius, img, rot: R, throttle):
    """Scaled legacy gauge showing roll/pitch vector and throttle fill."""
    x, y = center
    radius = max(10, int(radius))
    outline_thickness = max(2, radius // 40)
    cv2.circle(img, center, radius, (51, 51, 51), outline_thickness)  # Outline

    throttle_color = (255, 255, 255) if throttle >= 0 else (255, 0, 0)
    throttle_radius = int(round(min(1.0, abs(throttle) / THROTTLE_MAX) * (radius - outline_thickness * 2)))
    if throttle_radius > 0:
        cv2.circle(img, center, throttle_radius, throttle_color, -1)  # Throttle fill

    center_radius = max(4, radius // 25)
    cv2.circle(img, center, center_radius, (0, 0, 255), -1)  # Center

    roll, pitch, _ = rot.as_euler("xyz", degrees=True)
    roll_scale = radius * 0.7 / ROLL_MAX
    pitch_scale = radius * 0.7 / PITCH_MAX
    dx = int(round(clamp(roll, -ROLL_MAX, ROLL_MAX) * roll_scale))
    # Screen y grows downward; invert pitch so negative pitch (nose down) draws downward.
    dy = int(round(-clamp(pitch, -PITCH_MAX, PITCH_MAX) * pitch_scale))
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


def draw_attitude_indicator(center, radius, img, rot: R):
    """Cessna-style attitude indicator with artificial horizon."""
    x, y = center
    radius = max(20, int(radius))
    size = int(radius * 2.4)
    half = size // 2
    cx = cy = half

    overlay = np.zeros((size, size, 3), dtype=np.uint8)

    roll, pitch, _ = rot.as_euler("xyz", degrees=True)
    pitch_norm = clamp(pitch, -PITCH_MAX, PITCH_MAX) / PITCH_MAX
    pitch_scale_px = radius * 0.7
    pitch_shift = int(round(pitch_norm * pitch_scale_px))
    horizon_y = cy + pitch_shift

    sky_color = (240, 220, 180)  # light blue-ish (BGR)
    ground_color = (70, 90, 140)  # brown/earth tone (BGR)
    cv2.rectangle(overlay, (0, 0), (size, horizon_y), sky_color, -1)
    cv2.rectangle(overlay, (0, horizon_y), (size, size), ground_color, -1)

    horizon_thickness = max(2, radius // 35)
    cv2.line(overlay, (0, horizon_y), (size, horizon_y), (255, 255, 255), horizon_thickness, cv2.LINE_AA)

    pitch_range_px = pitch_scale_px
    text_pad = max(4, radius // 25)
    font_size = max(8, int(radius * 0.08))  # smaller text
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", font_size)
    except OSError:
        font = ImageFont.load_default()

    tick_positions = []

    long_len = int(radius * 0.55)
    short_len = int(radius * 0.35)

    for idx, offset in enumerate(range(-90, 91, 15)):
        offset_px = int(round(offset / PITCH_MAX * pitch_range_px))
        y_line = horizon_y - offset_px
        if 0 <= y_line < size:
            line_len = long_len if idx % 2 == 0 else short_len
            tick_positions.append((line_len, y_line, f"{offset:+}"))
            cv2.line(
                overlay,
                (cx - line_len, y_line),
                (cx + line_len, y_line),
                (255, 255, 255),
                max(1, radius // 45),
                cv2.LINE_AA,
            )

    # Draw pitch labels with PIL on the right side only
    if tick_positions:
        pil_overlay = Image.fromarray(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil_overlay)
        for line_len, y_line, label in tick_positions:
            pos = (cx + line_len + text_pad, y_line - font_size // 2)
            draw.text(pos, label, font=font, fill=(255, 255, 255))
        overlay = cv2.cvtColor(np.array(pil_overlay), cv2.COLOR_RGB2BGR)

    roll_angle = clamp(roll, -ROLL_MAX, ROLL_MAX)
    rot_mat = cv2.getRotationMatrix2D((cx, cy), roll_angle, 1.0)
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
