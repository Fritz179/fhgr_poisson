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
from threading import Lock

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pynput import keyboard

import socket

WHITE = (255, 255, 255)
ACTIVE_COLOR = (0, 200, 0)
WARNING_COLOR = (0, 0, 255)

WIDTH = 3408
HEIGHT = 2130
TEXT_SIZE = 40


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


def draw_heading(img, x, y, dx, dy, throttle):
    radius = 100

    cv2.circle(img, (x, y), radius, (51, 51, 51), 2)  # Outline

    throttle_color = (255, 255, 255) if throttle >= 0 else (255, 0, 0)
    throttle_radius = int(round(abs(throttle)))
    cv2.circle(img, (x, y), throttle_radius, throttle_color, -1)  # Throttle fill
    cv2.circle(img, (x, y), 5, (0, 0, 255), -1)  # Center

    cv2.arrowedLine(img, (x, y), (int(x + dx), int(y + dy)), (0, 255, 0), 2, cv2.LINE_AA, 0, 0.2)


def main():
    window_name = "Poisson Robot Control"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    roll = 0.0  # -100 to 100
    pitch = 0.0  # -100 to 100
    yaw = 0.0  # -100 to 100
    throttle = 0.0  # -100 to 100
    # Important: sqrt(pitch^2 + roll^2) <= 100

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

    prev_time = time.time()

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

            img = np.zeros((render_height, render_width, 3), dtype=np.uint8)

            draw_heading(img, render_width // 2, render_height // 2, roll, pitch, throttle)

            pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_img)

            margin = sv(10)
            header_y = margin
            title_gap = max(line_spacing, int(title_font_size * 1.1))
            warning_y = header_y + title_gap

            write_text(draw, "Poisson Robot Control", (margin, header_y), title_font)

            hint_y = warning_y + line_spacing if warning_message else header_y + title_gap
            if warning_message:
                write_text(draw, warning_message, (margin, warning_y), font, WARNING_COLOR)

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
            write_text(draw, f"Roll: {int(round(roll))}", (margin, stats_y), font)
            stats_y += line_spacing
            write_text(draw, f"Pitch: {int(round(pitch))}", (margin, stats_y), font)
            stats_y += line_spacing
            write_text(draw, f"Yaw: {int(round(yaw))}", (margin, stats_y), font)
            stats_y += line_spacing
            write_text(draw, f"Throttle: {int(round(throttle))}", (margin, stats_y), font)
            
            footer_y = render_height - margin - 50
            write_text(draw, "ESC to exit", (margin, footer_y), font)

            img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

            # Push updated PWM values to the pi
            msg = f"{roll:.2f},{pitch:.2f},{yaw:.2f},{throttle:.2f}".encode()
            sock.sendto(msg, PI_ADDR)

            # Hide opencv window decorations
            key = cv2.waitKey(1)
            if key == 27:
                break
            cv2.imshow(window_name, img)
    finally:
        try:
            sock.sendto(b"0.00,0.00,0.00,0.00", PI_ADDR)
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

I want to have multiple guages, each of them is drawn in a function, the first parameter is (x, y) and second parameter is radius an everything should be scaled to that.

rename draw_heading to draw_legacy_gauge, 
Add another gauge on the right side for attitude control, with artificial horizon

I like the cessna style gauges
"""
