import time

from periphery import PWM
from scipy.spatial.transform import Rotation as R

from .connection import Connection
from .mpu6050 import MPU6050
from .control import fabrizio_pid, york_pid, no_pid

PWM_FREQUENCY = 50
PWM_PERIOD_US = 1_000_000 / PWM_FREQUENCY

# Tuple of (min, base, max) for each control surface
LEFT_LIMITS = (-0.8, 0.03, 0.8)
MIDDLE_LIMITS = (-0.65, -0.12, 0.45)
RIGHT_LIMITS = (-0.45, 0.14, 0.7)
THRUST_LIMITS = (-1, 0, 1)

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

def main():
    pwm_left = PWM(2, 0)
    pwm_right = PWM(3, 0) 
    pwm_middle = PWM(1, 0)
    pwm_motor = PWM(0, 0)

    for dev in (pwm_left, pwm_right, pwm_middle):
        dev.frequency = PWM_FREQUENCY
        dev.duty_cycle = 0.075  # 1.5 ms / 20 ms neutral
        dev.enable()

    # pwm_right.frequency = PWM_FREQUENCY
    # pwm_right.duty_cycle = 0.065  # Inverted for motor
    # pwm_right.enable()

    pwm_motor.frequency = PWM_FREQUENCY
    pwm_motor.duty_cycle = 1 - 0.075  # Inverted for motor
    pwm_motor.enable()

    imu = None
    connection = None

    def update_poisson():
        nonlocal pwm_left, pwm_right, pwm_middle, pwm_motor, connection, imu

        if not imu or not imu.get_state():
            print("No IMU available yet.")
            return
        
        state = imu.get_state()

        imu_roll, imu_pitch, imu_yaw = state.quat.as_euler("xyz", degrees=True)
        if imu_yaw > 180.0 or imu_yaw < -180.0:
            imu_yaw = ((imu_yaw + 180.0) % 360.0) - 180.0
        

        if not connection or not connection.sender_socket:
            print(f"IMU: roll={imu_roll:.2f}, pitch={imu_pitch:.2f}, yaw={imu_yaw:.2f}")
            
            print("No connection available yet.")
            return

        command = connection.get_latest()

        if command.pid_selection == 1:
            output = fabrizio_pid(state, command)
        elif command.pid_selection == 2:
            output = york_pid(state, command)
        else:
            output = no_pid(None, command)

        def get_duty(value, limits):
            min, mid, max = limits
            val = clamp(value + mid, min, max)
            pwm = clamp(1500 + val * 500, 1000, 2000)
            return pwm
        
        left_duty = get_duty(output.left, LEFT_LIMITS)
        right_duty = get_duty(output.right, RIGHT_LIMITS)
        middle_duty = get_duty(output.middle, MIDDLE_LIMITS)
        motor_duty = get_duty(output.throttle, THRUST_LIMITS)

        print(
            f"IMU: roll={imu_roll:.2f}, pitch={imu_pitch:.2f}, yaw={imu_yaw:.2f}\n"
            f"CMD: roll={command.roll:.2f}, pitch={command.pitch:.2f}, "
            f"yaw={command.yaw:.2f}, throttle={command.throttle:.2f}\n"
            f"PID: {command.pid_selection}, P: {command.pid_data[0]:.2f}, I: {command.pid_data[1]:.2f}, D: {command.pid_data[2]:.2f}\n"
            f"OUT: left={output.left:.2f}, middle={output.middle:.2f}, "
            f"right={output.right:.2f}, motor={output.throttle:.2f}\n"
            f"Duty: left={left_duty:.0f}, middle={middle_duty:.0f}, "
            f"right={right_duty:.0f}, motor={motor_duty:.0f}\n"
        )

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
