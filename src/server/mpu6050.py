import struct
import time

from periphery import I2C


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

    def __init__(self, i2c_bus="/dev/i2c-1", address=ADDRESS):
        self.address = None
        self.i2c = I2C(i2c_bus)
        self.address = address
        self._configure()

    def close(self):
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
