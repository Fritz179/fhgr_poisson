"""
Client main entrypoint and control loop.
"""

import time
from threading import Lock
import cv2
from pynput import keyboard
import math
from scipy.spatial.transform import Rotation as R

from .connection import Connection, State, THROTTLE_LIMIT
from .display import Display

STATE_TIMEOUT = 5.0

target_state = State()
display_state = State()


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def main():
    global display_state, target_state
    window_name = "Poisson Robot Control"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    throttle = 0.0  # -100 to 100
    orientation = R.identity()  # true attitude
    target_state = State()
    display_state = State()

    direction_rate = 90.0  # deg/s while holding W/A/S/D
    yaw_rate = 60.0  # deg/s while holding Q/E
    throttle_rate = 60.0  # units per second while holding Shift/Ctrl
    fine_throttle_rate = 20.0  # units per second while holding R/F
    keys_lock = Lock()
    pressed_keys = set()
    running = True
    shift_keys = {keyboard.Key.shift, keyboard.Key.shift_r, keyboard.Key.shift_l}
    ctrl_keys = {key for key in (getattr(keyboard.Key, "ctrl", None), getattr(keyboard.Key, "ctrl_r", None), getattr(keyboard.Key, "ctrl_l", None)) if key is not None}

    def on_press(key):
        nonlocal throttle, orientation, running

        if key == keyboard.Key.space:
            # Reset attitude only
            yaw_keep = orientation.as_euler("xyz", degrees=True)[2]
            orientation = R.from_euler("xyz", [0.0, 0.0, yaw_keep], degrees=True)

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

    connection = Connection(("poisson.local", 5005))

    start_time = time.time()
    display = Display(STATE_TIMEOUT, start_time)
    prev_time = start_time

    try:
        while running:
            current_time = time.time()
            delta_time = current_time - prev_time
            prev_time = current_time

            with keys_lock:
                active_keys = set(pressed_keys)

            pitch_down_key = "w"
            pitch_up_key = "s"

            # Angular rates in deg/s (body frame)
            pitch_rate_cmd = 0.0
            roll_rate_cmd = 0.0
            yaw_rate_cmd = 0.0

            if pitch_down_key in active_keys:
                pitch_rate_cmd -= direction_rate
            if pitch_up_key in active_keys:
                pitch_rate_cmd += direction_rate

            if "a" in active_keys:
                roll_rate_cmd -= direction_rate
            if "d" in active_keys:
                roll_rate_cmd += direction_rate

            if "q" in active_keys:
                yaw_rate_cmd -= yaw_rate
            if "e" in active_keys:
                yaw_rate_cmd += yaw_rate

            if "shift" in active_keys:
                throttle += throttle_rate * delta_time
            if "ctrl" in active_keys:
                throttle -= throttle_rate * delta_time

            if "r" in active_keys:
                throttle += fine_throttle_rate * delta_time
            if "f" in active_keys:
                throttle -= fine_throttle_rate * delta_time

            # Integrate body-frame rotation via quaternion/rotation
            roll_step = roll_rate_cmd * delta_time
            pitch_step = pitch_rate_cmd * delta_time
            yaw_step = yaw_rate_cmd * delta_time

            delta_rot = R.from_euler("xyz", [roll_step, pitch_step, yaw_step], degrees=True)
            orientation = orientation * delta_rot  # body-frame increment

            throttle = clamp(throttle, -THROTTLE_LIMIT, THROTTLE_LIMIT)

            target_state = State.from_rotation(orientation, throttle)

            now = time.time()

            img, display_state = display.render(
                target_state=target_state,
                connection=connection,
                active_keys=active_keys,
                now=now,
            )

            connection.set_command(target_state)

            # Hide opencv window decorations
            key = cv2.waitKey(1)
            if key == 27:
                running = False
                break
            cv2.imshow(window_name, img)
    finally:
        running = False
        connection.close()

        cv2.destroyWindow(window_name)


if __name__ == "__main__":
    main()
