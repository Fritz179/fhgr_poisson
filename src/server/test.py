import time

from periphery import PWM
from scipy.spatial.transform import Rotation as R


PWM_FREQUENCY = 50
PWM_PERIOD_US = 1_000_000 / PWM_FREQUENCY

# Tuple of (min, base, max) for each control surface
LEFT_LIMITS = (-0.9, 0, 0.9)
MIDDLE_LIMITS = (-0.5, 0.15, 0.8)
RIGHT_LIMITS = (-0.9, 0, 0.9)
THRUST_LIMITS = (-1, 0, 1)

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

def main():
    pwm_left = PWM(2, 0)
    pwm_right = PWM(3, 0) 
    pwm_middle = PWM(1, 0)

    for dev in (pwm_left, pwm_right, pwm_middle):
        dev.frequency = PWM_FREQUENCY
        dev.duty_cycle = 0.075  # 1.5 ms / 20 ms neutral
        dev.enable()


    duty = 0.05
    dir = 0.001

    while True:
        for dev in (pwm_left, pwm_right, pwm_middle):
            dev.duty_cycle = duty

        duty += dir
        if duty > 0.10:
            dir = -0.001
        elif duty < 0.05:
            dir = 0.001
        time.sleep(0.01)
        # return

if __name__ == "__main__":
    main()
