import math
import socket
import threading
import time

from scipy.spatial.transform import Rotation as R
from periphery import PWM

from .mpu6050 import MPU6050

PWM_FREQUENCY = 50
PWM_PERIOD_US = 1_000_000 / PWM_FREQUENCY
THROTTLE_LIMIT = 100.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def parse_quaternion_command(data: bytes):
    """Decode UDP payload into roll/pitch/yaw (deg) and throttle."""
    try:
        qx, qy, qz, qw, throttle = [float(x) for x in data.decode().split(",")]
    except ValueError as exc:
        raise ValueError(f"Malformed command payload: {exc}")

    rot = R.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = rot.as_euler("xyz", degrees=True)

    # Wrap yaw to keep it comparable to UI range
    if yaw > 180.0 or yaw < -180.0:
        yaw = ((yaw + 180.0) % 360.0) - 180.0

    throttle = clamp(throttle, -THROTTLE_LIMIT, THROTTLE_LIMIT)
    return roll, pitch, yaw, throttle


def imu_monitor(imu: MPU6050, stop_event: threading.Event):
    """Background IMU loop printing RPY angles, angular speed, acceleration, and temperature."""
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    alpha = 0.96  # complementary filter weight
    last_time = time.monotonic()

    try:
        while not stop_event.is_set():
            ax, ay, az, gx, gy, gz, temp_c = imu.read_scaled()
            now = time.monotonic()
            dt = max(now - last_time, 1e-3)
            last_time = now

            roll_acc = math.degrees(math.atan2(ay, az))
            pitch_acc = math.degrees(math.atan2(-ax, math.sqrt((ay * ay) + (az * az))))

            roll = (alpha * (roll + gx * dt)) + ((1 - alpha) * roll_acc)
            pitch = (alpha * (pitch + gy * dt)) + ((1 - alpha) * pitch_acc)
            yaw += gz * dt

            print(
                f"IMU RPY(deg): roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f} | "
                f"gyro(deg/s)=({gx:.2f}, {gy:.2f}, {gz:.2f}) | "
                f"accel(g)=({ax:.3f}, {ay:.3f}, {az:.3f}) | temp={temp_c:.2f}C"
            )

            stop_event.wait(0.05)
    except Exception as exc:
        print(f"IMU monitor stopped: {exc}")
    finally:
        imu.close()


def main():
    pwm_left = None
    pwm_right = None
    pwm_middle = None
    pwm_motor = None
    sock = None
    imu = None
    imu_stop_event = threading.Event()
    imu_thread = None

    try:
        imu = MPU6050()
        print("MPU6050 initialized on /dev/i2c-1 (address 0x68).")
        imu_thread = threading.Thread(target=imu_monitor, args=(imu, imu_stop_event), daemon=True)
        imu_thread.start()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", 5005))

        pwm_left = PWM(1, 0)
        pwm_right = PWM(2, 0)
        pwm_middle = PWM(3, 0)
        pwm_motor = PWM(0, 0)

        for dev in (pwm_left, pwm_right, pwm_middle):
            dev.frequency = PWM_FREQUENCY
            dev.duty_cycle = 0.075  # 1.5 ms / 20 ms neutral
            dev.enable()

        pwm_motor.frequency = PWM_FREQUENCY
        pwm_motor.duty_cycle = 1 - 0.075  # Inverted for motor
        pwm_motor.enable()
 
        while True:
            data, addr = sock.recvfrom(1024)
            try:
                roll, pitch, yaw, throttle = parse_quaternion_command(data)
            except ValueError as exc:
                print(f"Ignoring packet from {addr}: {exc}")
                continue

            left = pitch + roll
            right = pitch - roll
            middle = yaw

            left_duty = clamp(1500 + left * 5, 1000, 2000)
            right_duty = clamp(1500 + right * 5, 1000, 2000)
            middle_duty = clamp(1500 + middle * 5, 1000, 2000)
            motor_duty = clamp(1500 + throttle * 5, 1000, 2000)

            print(
                f"Received quat->rpy roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}, throttle: {throttle:.2f} => "
                f"left: {left:.2f}, right: {right:.2f}"
            )

            pwm_left.duty_cycle = left_duty / PWM_PERIOD_US
            pwm_middle.duty_cycle = middle_duty / PWM_PERIOD_US
            pwm_right.duty_cycle = right_duty / PWM_PERIOD_US
            pwm_motor.duty_cycle = 1 - motor_duty / PWM_PERIOD_US

    finally:
        imu_stop_event.set()
        if imu_thread and imu_thread.is_alive():
            imu_thread.join(timeout=1.0)

        if sock:
            sock.close()

        for dev in (pwm_left, pwm_right, pwm_middle, pwm_motor):
            if dev:
                try:
                    dev.disable()
                finally:
                    dev.close()

        print("Cleaned up PWM devices.")


if __name__ == "__main__":
    main()
