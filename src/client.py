"""
TODO:
- Output propeller speed + stepper positions
- read keycodes to change movement

W = pitch down
S = pitch up
A = roll left
D = roll right
Q = yaw left
E = yaw right
Shift = increase throttle
Ctrl = decrease throttle
Space = emergency stop
R/F = fine throttle trim
I = invert flight mode
"""

# Create an opencv window and display text

import time
from threading import Lock, Thread
from dataclasses import dataclass

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pynput import keyboard
import socket

from gauges import draw_attitude_indicator, draw_legacy_gauge


WHITE = (255, 255, 255)
ACTIVE_COLOR = (0, 200, 0)
WARNING_COLOR = (0, 0, 255)
CONNECTING_COLOR = (255, 165, 0)
DISCONNECTED_COLOR = (255, 64, 64)

WIDTH = 3408
HEIGHT = 2130
TEXT_SIZE = 40
STATE_TIMEOUT = 5.0

@dataclass
class State:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0

    def as_msg(self) -> bytes:
        return f"{self.roll:.2f},{self.pitch:.2f},{self.yaw:.2f},{self.throttle:.2f}".encode()


command_state = State()
display_state = State()


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def load_font(size):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except (OSError, IOError):
        return ImageFont.load_default()


def write_text(draw, text, position, font, color=WHITE):
    draw.text(position, text, font=font, fill=color)


def text_width(draw, text, font):
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


def draw_key_hint(draw, x, y, keys, description, active_keys, font):
    current_x = x
    for idx, key_entry in enumerate(keys):
        if isinstance(key_entry, tuple):
            label, key_id = key_entry
        else:
            label = key_entry.upper()
            key_id = key_entry.lower()

        color = ACTIVE_COLOR if key_id in active_keys else WHITE
        write_text(draw, label, (current_x, y), font, color)
        current_x += text_width(draw, label, font)

        if idx < len(keys) - 1:
            separator = "/"
            write_text(draw, separator, (current_x, y), font)
            current_x += text_width(draw, separator, font)

    current_x += 15
    write_text(draw, f": {description}", (current_x, y), font)

def main():
    global display_state, command_state
    window_name = "Poisson Robot Control"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    roll = 0.0  # -100 to 100
    pitch = 0.0  # -100 to 100
    yaw = 0.0  # -100 to 100
    throttle = 0.0  # -100 to 100
    # Important: sqrt(pitch^2 + roll^2) <= 100
    command_state = State()
    display_state = State()

    direction_rate = 80.0  # units per second while holding W/A/S/D
    yaw_rate = 80.0  # units per second while holding Q/E
    throttle_rate = 60.0  # units per second while holding Shift/Ctrl
    fine_throttle_rate = 20.0  # units per second while holding R/F
    screen_width, screen_height = detect_screen_size()
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

    def sv(value):
        return max(1, int(round(value * scale)))

    font_size = max(8, int(round(TEXT_SIZE * scale)))
    font = load_font(font_size)
    title_font_size = max(12, int(round(font_size * 2)))
    title_font = load_font(title_font_size)
    line_spacing = max(int(font_size * 1.3), font_size + sv(8))

    keys_lock = Lock()
    pressed_keys = set()
    running = True
    invert_pitch = False
    shift_keys = {keyboard.Key.shift, keyboard.Key.shift_r, keyboard.Key.shift_l}
    ctrl_keys = {key for key in (getattr(keyboard.Key, "ctrl", None), getattr(keyboard.Key, "ctrl_r", None), getattr(keyboard.Key, "ctrl_l", None)) if key is not None}

    def on_press(key):
        nonlocal pitch, roll, yaw, throttle, running, invert_pitch

        if key == keyboard.Key.space:
            # Emergency stop
            pitch = 0.0
            roll = 0.0
            yaw = 0.0
            throttle = 0.0
            with keys_lock:
                pressed_keys.add(" ")
            return

        if key == keyboard.Key.esc:
            running = False
            return

        if key in shift_keys:
            with keys_lock:
                pressed_keys.add("shift")
            return
        if key in ctrl_keys:
            with keys_lock:
                pressed_keys.add("ctrl")
            return

        try:
            char = key.char.lower()
        except AttributeError:
            return

        if char == "i":
            invert_pitch = not invert_pitch
            with keys_lock:
                pressed_keys.add(char)
            return

        pressed_keys.add(char)

    def on_release(key):
        if key == keyboard.Key.space:
            with keys_lock:
                pressed_keys.discard(" ")
            return

        if key in shift_keys:
            with keys_lock:
                pressed_keys.discard("shift")
            return
        if key in ctrl_keys:
            with keys_lock:
                pressed_keys.discard("ctrl")
            return

        try:
            char = key.char.lower()
        except AttributeError:
            return

        with keys_lock:
            pressed_keys.discard(char)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    PI_ADDR = ("poisson.local", 5005)
    sock.settimeout(0.1)
    sock_connected = False
    connect_error = None
    last_connect_attempt = 0.0

    start_time = time.time()
    prev_time = start_time
    last_received_time = None
    display_state = command_state

    def ensure_connected(force=False):
        nonlocal sock_connected, connect_error, last_connect_attempt
        now = time.time()
        if sock_connected and not force:
            return
        if not force and (now - last_connect_attempt) < 1.0:
            return
        last_connect_attempt = now
        try:
            sock.connect(PI_ADDR)
            sock_connected = True
            connect_error = None
        except OSError as exc:
            sock_connected = False
            connect_error = str(exc)

    def receive_loop():
        nonlocal last_received_time, sock_connected, connect_error
        global display_state

        while running:
            if not sock_connected:
                ensure_connected()
                time.sleep(0.1)
                continue

            try:
                data = sock.recv(1024)
            except socket.timeout:
                continue
            except OSError as exc:
                sock_connected = False
                connect_error = str(exc)
                continue

            try:
                r, p, y, t = [float(x) for x in data.decode().split(",")]
            except ValueError:
                continue

            display_state = State(r, p, y, t)
            last_received_time = time.time()

    receiver_thread = Thread(target=receive_loop, daemon=True)
    receiver_thread.start()

    try:
        while running:
            current_time = time.time()
            delta_time = current_time - prev_time
            prev_time = current_time

            with keys_lock:
                active_keys = set(pressed_keys)

            pitch_down_key = "w"
            pitch_up_key = "s"

            if invert_pitch:
                pitch_down_key, pitch_up_key = pitch_up_key, pitch_down_key

            if pitch_down_key in active_keys:
                pitch -= direction_rate * delta_time
            if pitch_up_key in active_keys:
                pitch += direction_rate * delta_time

            if "a" in active_keys:
                roll -= direction_rate * delta_time
            if "d" in active_keys:
                roll += direction_rate * delta_time

            if "q" in active_keys:
                yaw -= yaw_rate * delta_time
            if "e" in active_keys:
                yaw += yaw_rate * delta_time

            if "shift" in active_keys:
                throttle += throttle_rate * delta_time
            if "ctrl" in active_keys:
                throttle -= throttle_rate * delta_time

            if "r" in active_keys:
                throttle += fine_throttle_rate * delta_time
            if "f" in active_keys:
                throttle -= fine_throttle_rate * delta_time

            # Clamp values
            pitch = clamp(pitch, -100, 100)
            roll = clamp(roll, -100, 100)
            yaw = clamp(yaw, -100, 100)
            throttle = clamp(throttle, -100, 100)

            # Important: sqrt(pitch^2 + roll^2) <= 100
            magnitude = (pitch ** 2 + roll ** 2) ** 0.5
            if magnitude > 100:
                limit_scale = 100 / magnitude
                pitch *= limit_scale
                roll *= limit_scale

            command_state = State(roll, pitch, yaw, throttle)

            now = time.time()

            rx_fresh = last_received_time is not None and (now - last_received_time) <= STATE_TIMEOUT
            if not rx_fresh:
                display_state = command_state

            render_state = display_state
            display_roll = render_state.roll
            display_pitch = render_state.pitch
            display_yaw = render_state.yaw
            display_throttle = render_state.throttle

            img = np.zeros((render_height, render_width, 3), dtype=np.uint8)

            gauge_radius = max(60, int(min(render_width, render_height) * 0.18))
            legacy_center = (int(render_width * 0.32), render_height // 2)
            attitude_center = (int(render_width * 0.68), render_height // 2)

            draw_legacy_gauge(legacy_center, gauge_radius, img, display_roll, display_pitch, display_throttle)
            draw_attitude_indicator(attitude_center, gauge_radius, img, display_roll, display_pitch)

            pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_img)

            margin = sv(10)
            header_y = margin
            title_gap = max(line_spacing, int(title_font_size * 1.1))
            status_lines = []
            if rx_fresh:
                pass
            else:
                if last_received_time is None and (now - start_time) < STATE_TIMEOUT:
                    status_lines.append(("Connecting...", CONNECTING_COLOR))
                else:
                    status_lines.append(("Not connected", DISCONNECTED_COLOR))
                    if connect_error:
                        status_lines.append((connect_error, DISCONNECTED_COLOR))
            if warning_message:
                status_lines.append((warning_message, WARNING_COLOR))

            write_text(draw, "Poisson Robot Control", (margin, header_y), title_font)

            hint_y = header_y + title_gap
            for text, color in status_lines:
                write_text(draw, text, (margin, hint_y), font, color)
                hint_y += line_spacing

            draw_key_hint(draw, margin, hint_y, [pitch_down_key, pitch_up_key], "Pitch Down/Up", active_keys, font)
            hint_y += line_spacing
            draw_key_hint(draw, margin, hint_y, ["a", "d"], "Roll Left/Right", active_keys, font)
            hint_y += line_spacing
            draw_key_hint(draw, margin, hint_y, ["q", "e"], "Yaw Left/Right", active_keys, font)
            hint_y += line_spacing
            draw_key_hint(draw, margin, hint_y, [("Shift", "shift"), ("Ctrl", "ctrl")], "Throttle Up/Down", active_keys, font)
            hint_y += line_spacing
            draw_key_hint(draw, margin, hint_y, ["f", "r"], "Fine Throttle Down/Up", active_keys, font)
            hint_y += line_spacing
            draw_key_hint(draw, margin, hint_y, [("Space", " ")], "Emergency Stop", active_keys, font)
            hint_y += line_spacing
            draw_key_hint(draw, margin, hint_y, ["i"], "Invert flight mode", active_keys, font)

            stats_y = hint_y + line_spacing
            write_text(draw, f"State Roll: {int(round(display_roll))}", (margin, stats_y), font)
            stats_y += line_spacing
            write_text(draw, f"State Pitch: {int(round(display_pitch))}", (margin, stats_y), font)
            stats_y += line_spacing
            write_text(draw, f"State Yaw: {int(round(display_yaw))}", (margin, stats_y), font)
            stats_y += line_spacing
            write_text(draw, f"State Throttle: {int(round(display_throttle))}", (margin, stats_y), font)

            footer_y = render_height - margin - 50
            write_text(draw, "ESC to exit", (margin, footer_y), font)

            img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

            # Push updated PWM values to the pi
            if sock_connected:
                try:
                    sock.send(command_state.as_msg())
                except OSError as exc:
                    sock_connected = False
                    connect_error = str(exc)

            # Hide opencv window decorations
            key = cv2.waitKey(1)
            if key == 27:
                running = False
                break
            cv2.imshow(window_name, img)
    finally:
        running = False
        try:
            if sock_connected:
                sock.send(State().as_msg())
        except OSError:
            pass
        try:
            receiver_thread.join(timeout=0.5)
        except RuntimeError:
            pass

        try:
            sock.close()
        except OSError:
            pass

        cv2.destroyWindow(window_name)


if __name__ == "__main__":
    main()

"""
TODO:

- attitude, artificial horizon
- Slip/skid ball
- turn needle
- vertical speed
- speedometer / throttle bars
- g meter?
- flight director / nav ball
- compass / tape
"""
