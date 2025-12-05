import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

from scipy.spatial.transform import Rotation as R

from periphery import I2C


@dataclass
class ImuState:
    quat: R
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    temp_c: float
    timestamp: float
    dt: float

    def as_msg(self, throttle: float = 0.0) -> bytes:
        qx, qy, qz, qw = self.quat.as_quat()
        return (
            f"{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},"
            f"{self.ax:.4f},{self.ay:.4f},{self.az:.4f},"
            f"{self.gx:.4f},{self.gy:.4f},{self.gz:.4f},"
            f"{self.temp_c:.2f},{throttle:.2f}"
        ).encode()


class MPU6050:
    """Minimal MPU6050 helper for reading accel/gyro/temp over I2C."""

    ADDRESS = 0x68
    WHO_AM_I = 0x75
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B

    def __init__(
        self,
        i2c_bus="/dev/i2c-1",
        address=ADDRESS,
        on_update=None,
        connection=None,
        poll_interval: float = 0.05,
    ):
        self.address = None
        self.i2c = I2C(i2c_bus)
        self.address = address

        self._on_update = on_update
        self._connection = connection
        self._poll_interval = poll_interval

        self._stop_event = threading.Event()
        self._state_lock = threading.Lock()
        self._state: Optional[ImuState] = None
        self._thread: Optional[threading.Thread] = None

        self._configure()
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()

    def close(self):
        self._stop_event.set()
        try:
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except RuntimeError:
            pass
        if self.i2c:
            self.i2c.close()

    def _write_register(self, register, value):
        self.i2c.transfer(self.address, [I2C.Message([register, value])])

    def _read_registers(self, start_register, length):
        msgs = [I2C.Message([start_register]), I2C.Message(bytearray(length), read=True)]
        self.i2c.transfer(self.address, msgs)
        return bytes(msgs[1].data)

    def _read_registers_at(self, address, start_register, length):
        msgs = [I2C.Message([start_register]), I2C.Message(bytearray(length), read=True)]
        self.i2c.transfer(address, msgs)
        return bytes(msgs[1].data)

    def _configure(self):
        # Wake up device and set default ranges (gyro: +/-250 deg/s, accel: +/-2g).
        self._write_register(self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self._write_register(self.SMPLRT_DIV, 0x07)
        self._write_register(self.CONFIG, 0x06)
        self._write_register(self.GYRO_CONFIG, 0x00)
        self._write_register(self.ACCEL_CONFIG, 0x00)

    def read_scaled(self):
        """Return accel (g), gyro (deg/s), and temperature (C)."""
        data = self._read_registers(self.ACCEL_XOUT_H, 14)
        ax_raw, ay_raw, az_raw, temp_raw, gx_raw, gy_raw, gz_raw = struct.unpack(">hhhhhhh", data)

        accel_scale = 16384.0  # LSB/g
        gyro_scale = 131.0  # LSB/(deg/s)

        ax = ax_raw / accel_scale
        ay = ay_raw / accel_scale
        az = az_raw / accel_scale
        gx = gx_raw / gyro_scale
        gy = gy_raw / gyro_scale
        gz = gz_raw / gyro_scale
        temp_c = (temp_raw / 340.0) + 36.53

        return ax, ay, az, gx, gy, gz, temp_c

    def _monitor_loop(self):
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        orientation = R.identity()
        alpha = 0.96  # complementary filter weight
        last_time = time.monotonic()

        while not self._stop_event.is_set():
            try:
                ax, ay, az, gx, gy, gz, temp_c = self.read_scaled()
            except Exception as exc:
                print(f"IMU read failed: {exc}")
                self._stop_event.wait(self._poll_interval)
                continue

            now = time.monotonic()
            dt = max(now - last_time, 1e-3)
            last_time = now

            # Integrate gyro rates into the quaternion (body-frame)
            delta_rot = R.from_euler("xyz", [gx * dt, gy * dt, gz * dt], degrees=True)
            orientation = orientation * delta_rot

            roll_acc = math.degrees(math.atan2(ay, az))
            pitch_acc = math.degrees(math.atan2(-ax, math.sqrt((ay * ay) + (az * az))))

            # Blend gyro-integrated attitude with accelerometer reference for roll/pitch
            est_roll, est_pitch, est_yaw = orientation.as_euler("xyz", degrees=True)
            roll = (alpha * est_roll) + ((1 - alpha) * roll_acc)
            pitch = (alpha * est_pitch) + ((1 - alpha) * pitch_acc)
            yaw = est_yaw
            wrapped_yaw = self._wrap_yaw(yaw)

            orientation = R.from_euler("xyz", [roll, pitch, wrapped_yaw], degrees=True)

            state = ImuState(
                quat=orientation,
                ax=ax,
                ay=ay,
                az=az,
                gx=gx,
                gy=gy,
                gz=gz,
                temp_c=temp_c,
                timestamp=now,
                dt=dt,
            )

            with self._state_lock:
                self._state = state

            if self._connection:
                self._connection.send_state(state)

            if self._on_update:
                self._on_update()

            self._stop_event.wait(self._poll_interval)

    @staticmethod
    def _wrap_yaw(yaw: float) -> float:
        if yaw > 180.0 or yaw < -180.0:
            return ((yaw + 180.0) % 360.0) - 180.0
        return yaw

    def get_state(self) -> Optional[ImuState]:
        with self._state_lock:
            return self._state
