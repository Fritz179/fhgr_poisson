import cv2
import numpy as np
import time
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as R

from .connection import State
from .fonts import load_font
from .gauges import draw_attitude_indicator, draw_compass, draw_legacy_gauge, draw_thermometer

WHITE = (255, 255, 255)
ACTIVE_COLOR = (0, 200, 0)
WARNING_COLOR = (255, 0, 0)
CONNECTING_COLOR = (0, 165, 255)
DISCONNECTED_COLOR = (64, 64, 255)

TEXT_SIZE = 40
WIDTH = 3408
HEIGHT = 2130


def text_width(draw: ImageDraw.ImageDraw, text: str, font) -> int:
    bbox = draw.textbbox((0, 0), text, font=font)
    return bbox[2] - bbox[0]


def detect_screen_size():
    try:
        import tkinter as tk
    except Exception:
        return None, None

    try:
        root = tk.Tk()
        root.withdraw()
        root.update_idletasks()
        width = root.winfo_screenwidth()
        height = root.winfo_screenheight()
        root.destroy()
        return width, height
    except Exception:
        return None, None


def compute_render_geometry(screen_width, screen_height):
    scale = 1.0
    render_width = WIDTH
    render_height = HEIGHT
    warning_message = None

    if screen_width and screen_height:
        if screen_width != WIDTH or screen_height != HEIGHT:
            scale = min(screen_width / WIDTH, screen_height / HEIGHT)
            render_width = int(WIDTH * scale)
            render_height = int(HEIGHT * scale)
            warning_message = f"Warning: display {screen_width}x{screen_height}; scaling to {render_width}x{render_height}"
    
    return render_width, render_height, scale, warning_message


def _sv(value: float, scale: float) -> int:
    return max(1, int(round(value * scale)))


def _load_mono_font(size: int):
    try:
        return ImageFont.truetype("DejaVuSansMono.ttf", size)
    except (OSError, IOError):
        return load_font(size)


def _draw_key_hint(draw: ImageDraw.ImageDraw, x: int, y: int, keys, description: str, active_keys, font):
    current_x = x
    for idx, key_entry in enumerate(keys):
        if isinstance(key_entry, tuple):
            label, key_id = key_entry
        else:
            label = key_entry.upper()
            key_id = key_entry.lower()

        color = ACTIVE_COLOR if key_id in active_keys else WHITE
        draw.text((current_x, y), label, font=font, fill=color)
        current_x += text_width(draw, label, font)

        if idx < len(keys) - 1:
            separator = "/"
            draw.text((current_x, y), separator, font=font, fill=WHITE)
            current_x += text_width(draw, separator, font)

    current_x += 15
    draw.text((current_x, y), f": {description}", font=font, fill=WHITE)


def render_frame(
    render_width: int,
    render_height: int,
    scale: float,
    display_state,
    active_keys,
    status_lines,
    selected_pid: int,
):
    font_size = max(8, int(round(TEXT_SIZE * scale * 0.5)))
    font = load_font(font_size)
    title_font_size = max(12, int(round(font_size * 2)))
    title_font = load_font(title_font_size)
    line_spacing = max(int(font_size * 1.3), font_size + _sv(8, scale))
    margin = 0

    rot = R.from_quat(display_state.quat)
    roll, pitch, yaw = rot.as_euler("xyz", degrees=True)
    # Wrap yaw to keep gauges sensible
    if yaw > 180.0 or yaw < -180.0:
        yaw = ((yaw + 180.0) % 360.0) - 180.0
    display_roll = roll
    display_pitch = pitch
    display_yaw = yaw
    display_throttle = display_state.throttle
    display_temp = display_state.temp_c

    half_width = max(1, render_width // 2)
    half_height = max(1, render_height // 2)
    img = np.zeros((half_height, half_width, 3), dtype=np.uint8)

    size = min(half_width, half_height)
    margin = int(size * 0.02)
    cell_size = (size - 2 * margin) / 3
    radius = int(cell_size * 0.45)
    grid_w = grid_h = cell_size * 3 + margin * 2
    offset_x = int((half_width - grid_w / 3 * 2) / 2)
    offset_y = int((half_height - grid_h) / 2)

    panel = [
        [draw_legacy_gauge, draw_attitude_indicator, draw_thermometer],
        [draw_compass, draw_legacy_gauge, draw_legacy_gauge],
        [draw_legacy_gauge, draw_legacy_gauge, draw_legacy_gauge],
    ]

    timings = []
    for r, row in enumerate(panel):
        for c, func in enumerate(row):
            cx = int(offset_x + margin + (c + 0.5) * cell_size)
            cy = int(offset_y + margin + (r + 0.5) * cell_size)
            start = time.perf_counter()
            func((cx, cy), radius, img, display_state)
            dur = (time.perf_counter() - start) * 1000.0
            timings.append((func.__name__, dur))

    # if timings:
    #     summary = " | ".join(f"{name}:{d:.1f}ms" for name, d in timings)
    #     print(f"timings {summary}", flush=True)

    pil_img = Image.fromarray(img)
    draw = ImageDraw.Draw(pil_img)

    header_y = margin
    title_gap = max(line_spacing, int(title_font_size * 1.1))

    draw.text((margin, header_y), "Poisson Robot Control", font=title_font, fill=WHITE)

    hint_y = header_y + title_gap
    for text, color in status_lines:
        draw.text((margin, hint_y), text, font=font, fill=color)
        hint_y += line_spacing

    _draw_key_hint(draw, margin, hint_y, ["w", "s"], "Pitch Down/Up", active_keys, font)
    hint_y += line_spacing
    _draw_key_hint(draw, margin, hint_y, ["a", "d"], "Roll Left/Right", active_keys, font)
    hint_y += line_spacing
    _draw_key_hint(draw, margin, hint_y, ["q", "e"], "Yaw Left/Right", active_keys, font)
    hint_y += line_spacing
    _draw_key_hint(draw, margin, hint_y, [("Shift", "shift"), ("Ctrl", "ctrl")], "Throttle Up/Down", active_keys, font)
    hint_y += line_spacing
    _draw_key_hint(draw, margin, hint_y, ["f", "r"], "Fine Throttle Down/Up", active_keys, font)
    hint_y += line_spacing
    _draw_key_hint(draw, margin, hint_y, [("Space", " ")], "Emergency Stop", active_keys, font)

    stats_y = hint_y + line_spacing
    # Fixed-width R/P/Y block to avoid horizontal jitter
    mono_font = _load_mono_font(font_size)
    rpy_text = f"RPY:{display_roll:>4.0f}{display_pitch:>4.0f}{display_yaw:>4.0f}"
    draw.text((margin, stats_y), rpy_text, font=mono_font, fill=WHITE)
    stats_y += line_spacing
    ax, ay, az = display_state.accel
    gx, gy, gz = display_state.gyro
    accel_text = f"Acc:{ax:>5.2f}{ay:>5.2f}{az:>5.2f}"
    gyro_text = f"Gyr:{gx:>5.2f}{gy:>5.2f}{gz:>5.2f}"
    draw.text((margin, stats_y), accel_text, font=mono_font, fill=WHITE)
    stats_y += line_spacing
    draw.text((margin, stats_y), gyro_text, font=mono_font, fill=WHITE)
    stats_y += line_spacing
    pid_names = ["CMD_PID", "FAB_PID", "YRK_PID"]
    sel = selected_pid if 0 <= selected_pid < len(pid_names) else 0
    draw.text((margin, stats_y), f"PID: {pid_names[sel]}", font=font, fill=WHITE)
    stats_y += line_spacing
    if display_state.pid_values and 0 <= sel < len(display_state.pid_values):
        pv = display_state.pid_values[sel]
        draw.text((margin, stats_y), f"PID Val: {pv[0]:.4f}, {pv[1]:.4f}, {pv[2]:.4f}", font=font, fill=WHITE)
        stats_y += line_spacing
    draw.text((margin, stats_y), f"State Throttle: {display_throttle:.2f}", font=font, fill=WHITE)
    stats_y += line_spacing
    draw.text((margin, stats_y), f"Temp: {display_temp:.1f} °C", font=font, fill=WHITE)

    footer_y = render_height - margin - 50
    draw.text((margin, footer_y), "ESC to exit", font=font, fill=WHITE)

    frame = np.array(pil_img)
    if (half_width, half_height) != (render_width, render_height):
        frame = cv2.resize(frame, (render_width, render_height), interpolation=cv2.INTER_LINEAR)
    return frame


class Display:
    def __init__(self, state_timeout: float, start_time: float):
        self.state_timeout = state_timeout
        self.start_time = start_time
        self.fallback_temp = 0.0
        self.prev_quat = None
        self.prev_time = None
        screen_width, screen_height = detect_screen_size()
        self.render_width, self.render_height, self.scale, self.warning_message = compute_render_geometry(
            screen_width, screen_height
        )

    def render(self, target_state, connection, active_keys, now, selected_pid: int):
        received_state, last_received_time, connect_error, sock_connected = connection.get_state()
        rx_fresh = last_received_time is not None and (now - last_received_time) <= self.state_timeout
        if rx_fresh:
            display_state = received_state
            self.fallback_temp = display_state.temp_c
            self.prev_quat = display_state.quat
            self.prev_time = now
        else:
            # Simulate temperature drift when we have no telemetry
            self.fallback_temp += 0.1
            prev_quat = self.prev_quat or target_state.quat
            prev_time = self.prev_time or (now - 0.016)
            dt = max(now - prev_time, 1e-3)
            current_rot = R.from_quat(target_state.quat)
            prev_rot_obj = R.from_quat(prev_quat)
            curr_euler = np.array(current_rot.as_euler("xyz", degrees=True))
            prev_euler = np.array(prev_rot_obj.as_euler("xyz", degrees=True))
            diff = curr_euler - prev_euler
            # unwrap to [-180, 180] to avoid jumps
            diff = (diff + 180.0) % 360.0 - 180.0
            gyro_fake = tuple(diff / dt / 90.0)
            # crude fake accel proportional to angular change
            accel_fake = tuple((d / 9.0) for d in diff)
            display_state = State(
                quat=target_state.quat,
                throttle=target_state.throttle,
                accel=accel_fake,
                gyro=gyro_fake,
                temp_c=self.fallback_temp,
                selected_pid=selected_pid,
                pid_values=target_state.pid_values,
            )
            self.prev_quat = target_state.quat
            self.prev_time = now

        status_lines = []
        if sock_connected:
            if not rx_fresh:
                status_lines.append(("Connected (no data yet)", CONNECTING_COLOR))
            if connect_error:
                status_lines.append((connect_error, WARNING_COLOR))
        else:
            if last_received_time is None and (now - self.start_time) < self.state_timeout:
                status_lines.append(("Connecting...", CONNECTING_COLOR))
            else:
                status_lines.append(("Not connected", DISCONNECTED_COLOR))
                if connect_error:
                    status_lines.append((connect_error, DISCONNECTED_COLOR))
        if self.warning_message:
            status_lines.append((self.warning_message, WARNING_COLOR))

        img = render_frame(
            self.render_width,
            self.render_height,
            self.scale,
            display_state,
            active_keys,
            status_lines,
            selected_pid,
        )
        return img, display_state
