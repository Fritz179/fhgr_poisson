import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as R

from .gauges import draw_attitude_indicator, draw_compass, draw_legacy_gauge

WHITE = (255, 255, 255)
ACTIVE_COLOR = (0, 200, 0)
WARNING_COLOR = (0, 0, 255)
CONNECTING_COLOR = (255, 165, 0)
DISCONNECTED_COLOR = (255, 64, 64)

TEXT_SIZE = 40
WIDTH = 3408
HEIGHT = 2130


def load_font(size: int):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except (OSError, IOError):
        return ImageFont.load_default()


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
):
    font_size = max(8, int(round(TEXT_SIZE * scale)))
    font = load_font(font_size)
    title_font_size = max(12, int(round(font_size * 2)))
    title_font = load_font(title_font_size)
    line_spacing = max(int(font_size * 1.3), font_size + _sv(8, scale))
    margin = _sv(10, scale)

    rot = R.from_quat(display_state.quat)
    roll, pitch, yaw = rot.as_euler("xyz", degrees=True)
    # Wrap yaw to keep gauges sensible
    if yaw > 180.0 or yaw < -180.0:
        yaw = ((yaw + 180.0) % 360.0) - 180.0
    display_roll = roll
    display_pitch = pitch
    display_yaw = yaw
    display_throttle = display_state.throttle

    img = np.zeros((render_height, render_width, 3), dtype=np.uint8)

    gauge_radius = max(60, int(min(render_width, render_height) * 0.16))
    legacy_center = (int(render_width * 0.25), render_height // 2)
    attitude_center = (render_width // 2, render_height // 2)
    compass_center = (int(render_width * 0.75), render_height // 2)

    draw_legacy_gauge(legacy_center, gauge_radius, img, rot, display_throttle)
    draw_attitude_indicator(attitude_center, gauge_radius, img, rot)
    draw_compass(compass_center, gauge_radius, img, rot)

    pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
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
    draw.text((margin, stats_y), f"State Roll: {int(round(display_roll))}", font=font, fill=WHITE)
    stats_y += line_spacing
    draw.text((margin, stats_y), f"State Pitch: {int(round(display_pitch))}", font=font, fill=WHITE)
    stats_y += line_spacing
    draw.text((margin, stats_y), f"State Yaw: {int(round(display_yaw))}", font=font, fill=WHITE)
    stats_y += line_spacing
    draw.text((margin, stats_y), f"State Throttle: {int(round(display_throttle))}", font=font, fill=WHITE)

    footer_y = render_height - margin - 50
    draw.text((margin, footer_y), "ESC to exit", font=font, fill=WHITE)

    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


class Display:
    def __init__(self, state_timeout: float, start_time: float):
        self.state_timeout = state_timeout
        self.start_time = start_time
        screen_width, screen_height = detect_screen_size()
        self.render_width, self.render_height, self.scale, self.warning_message = compute_render_geometry(
            screen_width, screen_height
        )

    def render(self, target_state, connection, active_keys, now):
        received_state, last_received_time, connect_error, sock_connected = connection.get_state()
        rx_fresh = last_received_time is not None and (now - last_received_time) <= self.state_timeout
        display_state = received_state if rx_fresh else target_state

        status_lines = []
        if not rx_fresh:
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
        )
        return img, display_state
