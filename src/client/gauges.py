import cv2
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation as R

from .fonts import load_font
from .connection import State

ROLL_MAX = 180.0
PITCH_MAX = 90.0
THROTTLE_MAX = 1.0
YAW_MAX = 180.0
TEMP_MIN = -10.0
TEMP_MAX = 50.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def _blit_centered(img, center, overlay):
    """Center-blit overlay (RGB) onto img using nonzero mask."""
    h, w = overlay.shape[:2]
    x, y = center
    x1 = max(0, x - w // 2)
    y1 = max(0, y - h // 2)
    x2 = min(img.shape[1], x1 + w)
    y2 = min(img.shape[0], y1 + h)
    sx1 = max(0, w // 2 - x + x1)
    sy1 = max(0, h // 2 - y + y1)
    sx2 = sx1 + (x2 - x1)
    sy2 = sy1 + (y2 - y1)
    sub = overlay[sy1:sy2, sx1:sx2]
    mask = sub.any(axis=2)
    roi = img[y1:y2, x1:x2]
    roi[mask] = sub[mask]


def draw_compass(center, radius, img, state: State):
    """Simple compass: 0 = North (up), yaw positive clockwise."""
    x, y = center
    radius = max(20, int(radius))
    rot = R.from_quat(state.quat)
    outline_thickness = max(2, radius // 25)
    cv2.circle(img, center, radius, (255, 255, 255), outline_thickness, cv2.LINE_AA)

    # Ticks every 30 deg
    tick_outer = radius - outline_thickness // 2
    long_len = max(6, radius // 6)
    short_len = max(3, radius // 10)
    tick_thickness = max(1, outline_thickness - 1)
    for angle in range(0, 360, 30):
        rad = np.deg2rad(angle - 90)
        outer = (int(x + tick_outer * np.cos(rad)), int(y + tick_outer * np.sin(rad)))
        inner_len = long_len if angle % 90 == 0 else short_len
        inner = (int(x + (tick_outer - inner_len) * np.cos(rad)), int(y + (tick_outer - inner_len) * np.sin(rad)))
        cv2.line(img, inner, outer, (180, 180, 180), tick_thickness, cv2.LINE_AA)
        # Two smaller ticks between main ticks (every 10 degrees)
        for offset in (10, 20):
            sub_angle = angle + offset - 90
            rad_sub = np.deg2rad(sub_angle)
            inner_sub_len = max(2, short_len // 2)
            outer_sub = (int(x + tick_outer * np.cos(rad_sub)), int(y + tick_outer * np.sin(rad_sub)))
            inner_sub = (
                int(x + (tick_outer - inner_sub_len) * np.cos(rad_sub)),
                int(y + (tick_outer - inner_sub_len) * np.sin(rad_sub)),
            )
            cv2.line(img, inner_sub, outer_sub, (120, 120, 120), max(1, tick_thickness - 1), cv2.LINE_AA)

    # Labels every 30 deg: N/E/S/W at cardinals, numbers elsewhere (30deg -> "3", etc.)
    font = load_font(max(8, radius // 6))
    pil_img = Image.fromarray(img)
    draw = ImageDraw.Draw(pil_img)
    for angle in range(0, 360, 30):
        if angle == 0:
            label = "N"
        elif angle == 90:
            label = "E"
        elif angle == 180:
            label = "S"
        elif angle == 270:
            label = "W"
        else:
            label = str(angle // 10)
        rad = np.deg2rad(angle - 90)
        tx = x + int((tick_outer - long_len * 1.6) * np.cos(rad))
        ty = y + int((tick_outer - long_len * 1.6) * np.sin(rad)) - font.size // 2
        draw.text((tx - font.getlength(label) / 2, ty), label, font=font, fill=(255, 255, 255))
    img[:] = np.array(pil_img)

    # Pointer (yaw 0 = north/up, positive clockwise)
    yaw = rot.as_euler("xyz", degrees=True)[2]
    clamped_yaw = clamp(yaw, -YAW_MAX, YAW_MAX)
    rad = np.deg2rad(clamped_yaw - 90)
    dx = int(round(radius * 0.75 * np.sin(rad)))
    dy = int(round(radius * 0.75 * -np.cos(rad)))
    pointer_thickness = max(2, radius // 20)
    cv2.arrowedLine(img, center, (x + dx, y + dy), (0, 200, 255), pointer_thickness, cv2.LINE_AA, 0, 0.25)

def draw_thermometer(center, radius, img, state: State):
    """Draw a simple vertical thermometer and digital readout. Returns bounding box (x, y, w, h)."""
    base_w = max(20, int(radius * 0.6))
    h = max(80, int(radius * 1.8))
    tick_space = max(int(base_w * 0.8), int(radius * 0.4), 20)
    w = base_w + tick_space  # extra width to fit ticks/labels
    temp_c = state.temp_c
    text_pad = max(4, w // 6)
    digital_font = load_font(max(8, int(w * 0.15)))
    text_extra = digital_font.size + text_pad
    overlay = np.zeros((h + text_extra, w, 3), dtype=np.uint8)

    tube_width = max(4, int(base_w * 0.18))  # thinner tube
    bulb_radius = max(tube_width, base_w // 4)  # smaller bulb
    x_offset = tick_space // 2  # shift tube left to make room for ticks/labels on right
    tube_x1 = x_offset + (base_w - tube_width) // 2
    tube_x2 = tube_x1 + tube_width
    tube_y1 = 0
    tube_y2 = h - bulb_radius

    cv2.rectangle(overlay, (tube_x1, tube_y1), (tube_x2, tube_y2), (200, 200, 200), 1, cv2.LINE_AA)

    ratio = (temp_c - TEMP_MIN) / (TEMP_MAX - TEMP_MIN)
    ratio = clamp(ratio, 0.0, 1.0)
    fill_height = int((tube_y2 - tube_y1 - 4) * ratio)
    fill_y1 = tube_y2 - fill_height
    fill_color = (0, 80, 255)
    cv2.rectangle(overlay, (tube_x1 + 2, fill_y1), (tube_x2 - 2, tube_y2 - 2), fill_color, -1, cv2.LINE_AA)

    bulb_center = (w // 2, h - bulb_radius)
    cv2.circle(overlay, bulb_center, bulb_radius, (200, 200, 200), 2, cv2.LINE_AA)
    cv2.circle(overlay, bulb_center, bulb_radius - 3, fill_color, -1, cv2.LINE_AA)

    major_tick_len = max(4, int(base_w * 0.2))
    minor_tick_len = max(2, int(major_tick_len * 0.4))
    tick_x1 = tube_x2  # flush with tube edge
    tick_x2 = tick_x1 + major_tick_len
    tick_x1_minor = tick_x1
    tick_x2_minor = tick_x1 + minor_tick_len
    tick_thickness = max(1, base_w // 18)
    span = (TEMP_MAX - TEMP_MIN)

    label_font = load_font(max(6, int(base_w * 0.15)))
    pil_img = Image.fromarray(overlay)
    draw = ImageDraw.Draw(pil_img)
    t_vals = list(range(int(TEMP_MIN), int(TEMP_MAX) + 1, 10))
    for t in t_vals:
        ratio_t = (t - TEMP_MIN) / span
        y_tick = tube_y2 - int((tube_y2 - tube_y1) * ratio_t)
        cv2.line(overlay, (tick_x1, y_tick), (tick_x2, y_tick), (220, 220, 220), tick_thickness, cv2.LINE_AA)
        label = f"{t:+}"
        bbox = draw.textbbox((0, 0), label, font=label_font)
        lbl_w = bbox[2] - bbox[0]
        lbl_h = bbox[3] - bbox[1]
        lbl_x = tick_x2 + 1
        lbl_y = y_tick - lbl_h // 2
        draw.text((lbl_x, lbl_y), label, font=label_font, fill=(180, 180, 180))
        # Minor tick at midpoint to next major
        mid_t = t + 5
        if mid_t <= TEMP_MAX:
            ratio_m = (mid_t - TEMP_MIN) / span
            y_mid = tube_y2 - int((tube_y2 - tube_y1) * ratio_m)
            cv2.line(overlay, (tick_x1_minor, y_mid), (tick_x2_minor, y_mid), (200, 200, 200), max(1, tick_thickness - 1), cv2.LINE_AA)

    temp_reading = f"{temp_c:.1f} C"
    bbox = draw.textbbox((0, 0), temp_reading, font=digital_font)
    txt_w = bbox[2] - bbox[0]
    txt_h = bbox[3] - bbox[1]
    text_x = (w - txt_w) // 2
    text_y = h + (text_extra - txt_h) // 2
    draw.text((text_x, text_y), temp_reading, font=digital_font, fill=(255, 255, 255))
    overlay = np.array(pil_img)

    _blit_centered(img, center, overlay)
    return (center[0] - w // 2, center[1] - (h + text_extra) // 2, w, h + text_extra)


def draw_legacy_gauge(center, radius, img, state: State):
    """Scaled legacy gauge showing roll/pitch vector and throttle fill."""
    x, y = center
    radius = max(10, int(radius))
    outline_thickness = max(2, radius // 40)
    cv2.circle(img, center, radius, (51, 51, 51), outline_thickness)  # Outline

    throttle_color = (255, 255, 255) if state.throttle >= 0 else (255, 0, 0)
    throttle_radius = int(round(min(1.0, abs(state.throttle) / THROTTLE_MAX) * (radius - outline_thickness * 2)))
    if throttle_radius > 0:
        cv2.circle(img, center, throttle_radius, throttle_color, -1)  # Throttle fill

    center_radius = max(4, radius // 25)
    cv2.circle(img, center, center_radius, (0, 0, 255), -1)  # Center

    roll, pitch, _ = R.from_quat(state.quat).as_euler("xyz", degrees=True)
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


def draw_attitude_indicator(center, radius, img, state: State):
    """Cessna-style attitude indicator with artificial horizon."""
    x, y = center
    radius = max(20, int(radius))
    size = int(radius * 2.4)
    half = size // 2
    cx = cy = half

    overlay = np.zeros((size, size, 3), dtype=np.uint8)

    roll, pitch, _ = R.from_quat(state.quat).as_euler("xyz", degrees=True)
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
    font = load_font(font_size)

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
        pil_overlay = Image.fromarray(overlay)
        draw = ImageDraw.Draw(pil_overlay)
        for line_len, y_line, label in tick_positions:
            pos = (cx + line_len + text_pad, y_line - font_size // 2)
            draw.text(pos, label, font=font, fill=(255, 255, 255))
        overlay = np.array(pil_overlay)

    roll_angle = -clamp(roll, -ROLL_MAX, ROLL_MAX)
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

    tick_outer = radius
    tick_thickness = max(2, radius // 35)
    tick_len_major = int(radius * 0.14)
    tick_len_minor = int(radius * 0.09)
    tick_angles = list(range(-60, 61, 10))  # evenly spaced 10 deg increments
    for angle in tick_angles:
        rad = np.deg2rad(angle)
        inner_len = tick_len_major if angle % 30 == 0 else tick_len_minor
        outer = (int(x + tick_outer * np.sin(rad)), int(y - tick_outer * np.cos(rad)))
        inner = (
            int(x + (tick_outer - inner_len) * np.sin(rad)),
            int(y - (tick_outer - inner_len) * np.cos(rad)),
        )
        cv2.line(img, inner, outer, (255, 255, 255), tick_thickness, cv2.LINE_AA)

    # Roll labels at major ticks
    label_angles = [-60, -30, 0, 30, 60]
    label_offset = int(radius * 0.18)
    label_font_size = max(10, int(radius * 0.12))
    roll_font = load_font(label_font_size)
    pil_img = Image.fromarray(img)
    draw = ImageDraw.Draw(pil_img)
    for angle in label_angles:
        text = f"{angle:+}"
        rad = np.deg2rad(angle)
        tx = x + int((tick_outer + label_offset) * np.sin(rad))
        ty = y - int((tick_outer + label_offset) * np.cos(rad))
        bbox = draw.textbbox((0, 0), text, font=roll_font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]
        draw.text((tx - tw // 2, ty - th // 2), text, font=roll_font, fill=(255, 255, 255))
    img[:] = np.array(pil_img)

    wing_span = int(radius * 0.9)
    body_height = int(radius * 0.25)
    symbol_thickness = max(2, radius // 30)
    cv2.line(img, (x - wing_span // 2, y), (x + wing_span // 2, y), (0, 200, 255), symbol_thickness, cv2.LINE_AA)
    cv2.line(img, (x, y), (x, y + body_height), (0, 200, 255), symbol_thickness, cv2.LINE_AA)
    cv2.circle(img, center, max(3, radius // 40), (0, 200, 255), -1)
