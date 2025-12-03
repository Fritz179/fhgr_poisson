import socket
from periphery import PWM

PWM_FREQUENCY = 50
PWM_PERIOD_US = 1_000_000 / PWM_FREQUENCY

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

def main():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", 5005))

        pwm_left = PWM(1, 0)
        pwm_right = PWM(2, 0)
        pwm_middle = PWM(3, 0)
        pwm_motor = PWM(0, 0)

        for dev in (pwm_left, pwm_right, pwm_middle, pwm_motor):
            dev.frequency = PWM_FREQUENCY
            dev.duty_cycle = 0.075  # 1.5 ms / 20 ms neutral
            dev.enable()

        while True:
            data, addr = sock.recvfrom(1024)
            roll, pitch, yaw, speed = data.decode().split(",")

            # Values range from -100 to 100
            roll = float(roll)
            pitch = float(pitch)
            yaw = float(yaw)
            speed = float(speed)

            left = pitch + roll
            right = pitch - roll
            middle = yaw

            left_duty = clamp(1500 + left * 5, 1000, 2000)
            right_duty = clamp(1500 + right * 5, 1000, 2000)
            middle_duty = clamp(1500 + middle * 5, 1000, 2000)
            motor_duty = clamp(1500 + speed * 5, 1000, 2000)

            print(f"Received speed: {speed}, roll: {roll}, pitch: {pitch}, yaw: {yaw} => left: {left}, right: {right}")

            pwm_left.duty_cycle = left_duty / PWM_PERIOD_US
            pwm_middle.duty_cycle = middle_duty / PWM_PERIOD_US
            pwm_right.duty_cycle = right_duty / PWM_PERIOD_US
            pwm_motor.duty_cycle = motor_duty / PWM_PERIOD_US

    finally:
        for dev in (pwm_left, pwm_right, pwm_middle, pwm_motor):
            if dev:
                try:
                    dev.disable()
                finally:
                    dev.close()

        print("Cleaned up PWM devices.")


if __name__ == "__main__":
    main()
