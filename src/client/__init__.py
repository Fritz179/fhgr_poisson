"""
Client main entrypoint and control loop.
"""

import time
from threading import Lock
import cv2
from pynput import keyboard
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
    throttle_rate = 1.0  # units per second while holding Shift/Ctrl
    fine_throttle_rate = 0.3  # units per second while holding R/F
    pid_selection = 0  # 0=NO_PID,1=FAB_PID,2=YRK_PID
    pid_values = [
        [1 / 90, 1 / 90, 1 / 90], # CMD
        [1 / 40, 1 / 40, 1 / 90], # Fabrizio
        [1 / 90, 1 / 90, 1 / 90], # York
    ]
    keys_lock = Lock()
    pressed_keys = set()
    running = True
    shift_keys = {keyboard.Key.shift, keyboard.Key.shift_r, keyboard.Key.shift_l}
    ctrl_keys = {key for key in (getattr(keyboard.Key, "ctrl", None), getattr(keyboard.Key, "ctrl_r", None), getattr(keyboard.Key, "ctrl_l", None)) if key is not None}
    frame_times = []

    def on_press(key):
        nonlocal throttle, orientation, running, pid_selection

        if key == keyboard.Key.space:
            # Reset attitude only
            yaw = 0
            if display.prev_quat is not None:
                yaw = R.from_quat(display.prev_quat).as_euler("xyz", degrees=True)[2]

            orientation = R.from_euler("xyz", [0.0, 0.0, yaw], degrees=True)

            with keys_lock:
                pressed_keys.add(" ")
                # Also reset throttle
                throttle = 0.0
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

        if char in {"0", "1", "2"}:
            pid_selection = int(char)
            return

        # PID tuning keys (only for selected PID)
        factor_up = 1.1
        factor_down = 0.9
        if char == "o":  # P up
            pid_values[pid_selection][0] *= factor_up
            return
        if char == "p":  # P down
            pid_values[pid_selection][0] *= factor_down
            return
        if char == "k":  # I down
            pid_values[pid_selection][1] *= factor_down
            return
        if char == "l":  # I up
            pid_values[pid_selection][1] *= factor_up
            return
        if char == "n":  # D down
            pid_values[pid_selection][2] *= factor_down
            return
        if char == "m":  # D up
            pid_values[pid_selection][2] *= factor_up
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

            target_state = State(
                quat=tuple(orientation.as_quat()),
                throttle=throttle,
                accel=(0.0, 0.0, 0.0),
                gyro=(0.0, 0.0, 0.0),
                temp_c=0.0,
                selected_pid=pid_selection,
                pid_values=tuple(tuple(row) for row in pid_values),
            )

            img, display_state = display.render(
                target_state=target_state,
                connection=connection,
                active_keys=active_keys,
                selected_pid=pid_selection,
                frame_times=frame_times.copy(),
            )

            connection.set_command(target_state)

            # Hide opencv window decorations
            key = cv2.waitKey(1)
            if key == 27:
                running = False
                break
            cv2.imshow(window_name, img)

            loop_end = time.time()
            frame_times.append(loop_end - current_time)
            if len(frame_times) > 20:
                frame_times.pop(0)
    finally:
        running = False
        connection.close()

        cv2.destroyWindow(window_name)


if __name__ == "__main__":
    main()

"""
- attitude, artificial horizon
- Slip/skid ball
- turn needle
- vertical speed
- speedometer / throttle bars
- g meter?
- flight director / nav ball
- compass / tape
"""
