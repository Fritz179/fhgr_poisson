from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R

from .mpu6050 import ImuState
from .connection import Command



@dataclass
class Output:
    """
    Values range from -1.0 to 1.0
    """
    left: float
    middle: float
    right: float
    throttle: float

def no_pid(state: ImuState, command: Command) -> Output:
    left = command.pitch - command.roll
    right = command.pitch + command.roll
    middle = -command.yaw
    throttle = command.throttle

    pl, pm, pr = command.pid_data

    # Right surface is mounted 180
    return Output(left * pl, middle * pm, -right * pr, throttle)

def fabrizio_pid(state: ImuState, command: Command) -> Output:
    error = state.quat.inv() * command.quat
    roll, pitch, yaw = error.as_euler("xyz", degrees=True)

    pp, pr, py = command.pid_data
 

    left = pitch * pp + roll * pr
    right = pitch * pp - roll * pr
    middle = yaw * py
    throttle = command.throttle

    print(f"PID params: P_pitch={pp}, P_roll={pr}, P_yaw={py}\n"
        f"Errors: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}\n"
        f"OUT: left={left:.2f}, middle={middle:.2f}, right={right:.2f}\n")

    # Right surface is mounted 180
    return Output(left, middle, -right, throttle)

x = 0
vx = 0
def york_pid(state: ImuState, command: Command) -> Output:
    is_roll, is_pitch, is_yaw = state.quat.as_euler("xyz", degrees=True)
    target_roll, target_pitch, target_yaw = command.quat.as_euler("xyz", degrees=True)

    global x, vx
    vx += state.ax * state.dt
    x += vx * state.dt

    print(f"ax={state.ax:.4f}, dt={state.dt:.4f}, vx={vx:.4f}, x={x:.4f}")

    return Output(0, 0, 0, 0)