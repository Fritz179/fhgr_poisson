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
    left = command.pitch + command.roll
    right = command.pitch - command.roll
    middle = command.yaw
    throttle = command.throttle

    pl, pm, pr = command.pid_data

    return Output(left * pl, middle * pm, right * pr, throttle)

def fabrizio_pid(state: ImuState, command: Command) -> Output:
    
    return Output(0, 0, 0, 0)


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