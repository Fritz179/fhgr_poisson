import time

from periphery import PWM
from scipy.spatial.transform import Rotation as R

from .connection import Connection
from .mpu6050 import MPU6050
from .control import fabrizio_pid, york_pid, no_pid

PWM_FREQUENCY = 50
PWM_PERIOD_US = 1_000_000 / PWM_FREQUENCY


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

def main():
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

    imu = None
    connection = None

    def update_poisson():
        nonlocal pwm_left, pwm_right, pwm_middle, pwm_motor, connection, imu

        if not connection or not imu:
            print("No connection or IMU available yet.")
            return

        command = connection.get_latest()
        state = imu.get_state()

        if command is None or state is None:
            print("No command or state available yet.")
            return
        
        imu_roll, imu_pitch, imu_yaw = state.quat.as_euler("xyz", degrees=True)
        if imu_yaw > 180.0 or imu_yaw < -180.0:
            imu_yaw = ((imu_yaw + 180.0) % 360.0) - 180.0

        if command.pid_selection == 1:
            output = fabrizio_pid(state, command)
        elif command.pid_selection == 2:
            output = york_pid(state, command)
        else:
            output = no_pid(state, command)

        print(
            f"IMU: roll={imu_roll:.2f}, pitch={imu_pitch:.2f}, yaw={imu_yaw:.2f}\n"
            f"CMD: roll={command.roll:.2f}, pitch={command.pitch:.2f}, "
            f"yaw={command.yaw:.2f}, throttle={command.throttle:.2f}\n"
            f"PID: {command.pid_selection}, P: {command.pid_data[0]:.2f}, I: {command.pid_data[1]:.2f}, D: {command.pid_data[2]:.2f}\n"
            f"OUT: left={output.left:.2f}, middle={output.middle:.2f}, "
            f"right={output.right:.2f}, motor={output.throttle:.2f}\n"
        )

        left_duty = clamp(1500 + output.left * 500, 1000, 2000)
        right_duty = clamp(1500 + output.right * 500, 1000, 2000)
        middle_duty = clamp(1500 + output.middle * 500, 1000, 2000)
        motor_duty = clamp(1500 + output.throttle * 500, 1000, 2000)

        pwm_left.duty_cycle = left_duty / PWM_PERIOD_US
        pwm_middle.duty_cycle = middle_duty / PWM_PERIOD_US
        pwm_right.duty_cycle = right_duty / PWM_PERIOD_US
        pwm_motor.duty_cycle = 1 - motor_duty / PWM_PERIOD_US

    try:
        connection = Connection(on_command=update_poisson)
        print("Listening for commands on 0.0.0.0:5005.")

        imu = MPU6050(on_update=update_poisson, connection=connection)
        print("MPU6050 initialized on /dev/i2c-1 (address 0x68).")

        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Stopping server.")

    finally:
        if imu:
            imu.close()
        if connection:
            connection.close()

        for dev in (pwm_left, pwm_right, pwm_middle, pwm_motor):
            if dev:
                try:
                    dev.disable()
                finally:
                    dev.close()

        print("Cleaned up PWM devices.")


if __name__ == "__main__":
    main()
