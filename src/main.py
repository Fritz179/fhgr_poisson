"""
TODO:
- Output propeller speed + stepper positions
- read keycodes to change movement

W/S => up/down
A/D => left/right
Q/E => faster/slower

Enter => stop all movement
"""

# Create an opencv window and display text

import time
from threading import Lock

import cv2
import numpy as np
from pynput import keyboard

FONT = cv2.FONT_HERSHEY_DUPLEX
FONT_SCALE = 1
FONT_THICKNESS = 1
WHITE = (255, 255, 255)
ACTIVE_COLOR = (0, 200, 0)

def write_text(img, text, position, color=WHITE):
    cv2.putText(img, text, position, FONT, FONT_SCALE, color, FONT_THICKNESS, cv2.LINE_4)
    cv2.putText(img, text, position, cv2.FONT_HERSHEY_DUPLEX, 1, color, 1, cv2.LINE_4)

def draw_key_hint(img, x, y, keys, description, active_keys):
    current_x = x
    for idx, key_entry in enumerate(keys):
        if isinstance(key_entry, tuple):
            label, key_id = key_entry
        else:
            label = key_entry.upper()
            key_id = key_entry.lower()

        color = ACTIVE_COLOR if key_id in active_keys else WHITE
        write_text(img, label, (current_x, y), color)
        text_size = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICKNESS)[0]
        current_x += text_size[0]

        if idx < len(keys) - 1:
            separator = "/"
            write_text(img, separator, (current_x, y))
            sep_size = cv2.getTextSize(separator, FONT, FONT_SCALE, FONT_THICKNESS)[0]
            current_x += sep_size[0]

    current_x += 15
    write_text(img, f": {description}", (current_x, y))

def draw_heading(img, x, y, dx, dy, speed):
    radius = 100

    cv2.circle(img, (x, y), radius, (51, 51, 51), 2) # Outline

    speed_color = (255, 255, 255) if speed >= 0 else (255, 0, 0)
    speed_radius = int(round(abs(speed)))
    cv2.circle(img, (x, y), speed_radius, speed_color, -1) # Speed
    cv2.circle(img, (x, y), 5, (0, 0, 255), -1) # Center

    cv2.arrowedLine(img, (x, y), (int(x + dx), int(y + dy)), (0, 255, 0), 2, cv2.LINE_AA, 0, 0.2)


def main():
    window_name = "Poisson Robot Control"
    cv2.namedWindow(window_name)

    pitch = 0.0 # -100 to 100
    roll = 0.0 # -100 to 100
    speed = 0.0 # -100 to 100
    # Important: sqrt(pitch^2 + roll^2) <= 100


    keys_lock = Lock()
    pressed_keys = set()
    running = True
    invert_pitch = False
    alternate_mode = False

    def on_press(key):
        nonlocal pitch, roll, speed, running, invert_pitch, alternate_mode

        if key == keyboard.Key.enter:
            pitch = 0.0
            roll = 0.0
            speed = 0.0
            with keys_lock:
                pressed_keys.add("enter")
            return
        
        if key == keyboard.Key.space:
            with keys_lock:
                pressed_keys.add(" ")
            return
        
        if key == keyboard.Key.esc:
            running = False
            return
        
        if key == keyboard.Key.shift:
            with keys_lock:
                pressed_keys.add("shift")
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
        
        if char == "o":
            alternate_mode = not alternate_mode
            with keys_lock:
                pressed_keys.add(char)
            return

        pressed_keys.add(char)


    def on_release(key):
        if key == keyboard.Key.enter:
            with keys_lock:
                pressed_keys.discard("enter")
            return

        if key == keyboard.Key.space:
            with keys_lock:
                pressed_keys.discard(" ")
            return

        if key == keyboard.Key.shift:
            with keys_lock:
                pressed_keys.discard("shift")
            return

        try:
            char = key.char.lower()
        except AttributeError:
            return

        with keys_lock:
            pressed_keys.discard(char)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()


    prev_time = time.time()

    while running:
        current_time = time.time()
        delta_time = current_time - prev_time
        prev_time = current_time

        with keys_lock:
            active_keys = set(pressed_keys)

        direction_rate = 80.0  # units per second while holding W/A/S/D
        speed_rate = 60.0  # units per second while holding Q/E

        up = " " if alternate_mode else "s"
        down = "shift" if alternate_mode else "w"

        if invert_pitch:
            [up, down] = [down, up]

        if down in active_keys:
            pitch += direction_rate * delta_time
        if up in active_keys:
            pitch -= direction_rate * delta_time

        if "a" in active_keys:
            roll -= direction_rate * delta_time
        if "d" in active_keys:
            roll += direction_rate * delta_time

        forward = "w" if alternate_mode else "e"
        backward = "s" if alternate_mode else "q"

        if backward in active_keys:
            speed -= speed_rate * delta_time
        if forward in active_keys:
            speed += speed_rate * delta_time

        img = np.zeros((720, 1280, 3), dtype=np.uint8)

        write_text(img, "Poisson Robot Control", (10, 30))
        draw_key_hint(img, 10, 70, [up, down], "Move Up/Down", active_keys)
        draw_key_hint(img, 10, 110, ["a", "d"], "Move Left/Right", active_keys)
        draw_key_hint(img, 10, 150, [backward, forward], "Increase/Decrease Speed", active_keys)
        draw_key_hint(img, 10, 190, [("Enter", "enter")], "Stop All Movement", active_keys)
        draw_key_hint(img, 10, 230, ["i"], "Invert flight mode", active_keys)
        draw_key_hint(img, 10, 270, ["o"], "Toggle Alternate Mode", active_keys)

        draw_heading(img, 640, 360, roll, pitch, speed)

        # Clamp values
        pitch = max(-100, min(100, pitch))
        roll = max(-100, min(100, roll))
        speed = max(-100, min(100, speed))

        # Important: sqrt(pitch^2 + roll^2) <= 100
        magnitude = (pitch ** 2 + roll ** 2) ** 0.5
        if magnitude > 100:
            scale = 100 / magnitude
            pitch *= scale
            roll *= scale

        # 1ms = full left, 1.5ms = middle, 2ms = full right

        left_value = pitch - roll
        right_value = pitch + roll

        motor_pwm = 1500 + speed * 5
        left_pwm = 1500 + left_value * 5
        right_pwm = 1500 + right_value * 5

        left_angle = left_value * 0.9
        right_angle = right_value * 0.9

        write_text(img, f"Pitch: {int(round(pitch))}", (10, 310))
        write_text(img, f"Roll: {int(round(roll))}", (10, 350))
        write_text(img, f"Speed: {int(round(speed))}", (10, 390))

        write_text(img, f"Left Servo: Angle: {left_angle:.0f}deg, PWM: {left_pwm:.0f}us", (10, 590))
        write_text(img, f"Right Servo: Angle: {right_angle:.0f}deg, PWM: {right_pwm:.0f}us", (10, 630))
        write_text(img, f"Motor: Speed: {int(round(speed))}, PWM: {motor_pwm:.0f}us", (10, 670))
        write_text(img, "ESC to exit", (10, 710))


        # Hide opencv window decorations
        key = cv2.waitKey(1)
        if key == 27:
            break
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow(window_name, img)


    listener.stop()
    listener.join()

    cv2.destroyWindow(window_name)


if __name__ == "__main__":
    main()
