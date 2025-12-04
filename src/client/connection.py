import socket
import time
from dataclasses import dataclass
from threading import Thread
from typing import Optional

from scipy.spatial.transform import Rotation as R

THROTTLE_LIMIT = 100.0

@dataclass
class State:
    quat: tuple = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w
    throttle: float = 0.0  # -100 to 100

    def as_msg(self) -> bytes:
        qx, qy, qz, qw = self.quat
        return f"{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},{self.throttle:.2f}".encode()

    @staticmethod
    def from_rotation(rot: R, throttle: float):
        return State(tuple(rot.as_quat()), throttle)


class Connection:
    def __init__(self, addr):
        self.addr = addr
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        self.sock_connected = False
        self.connect_error: Optional[str] = None
        self.last_connect_attempt = 0.0
        self.last_received_time: Optional[float] = None
        self.display_state = State()
        self.running = True

        self._receiver = Thread(target=self._receive_loop, daemon=True)
        self._receiver.start()

    def _ensure_connected(self, force=False):
        now = time.time()
        if self.sock_connected and not force:
            return
        if not force and (now - self.last_connect_attempt) < 1.0:
            return
        self.last_connect_attempt = now
        try:
            self.sock.connect(self.addr)
            self.sock_connected = True
            self.connect_error = None
        except OSError as exc:
            self.sock_connected = False
            self.connect_error = str(exc)

    def _receive_loop(self):
        while self.running:
            if not self.sock_connected:
                self._ensure_connected()
                time.sleep(0.1)
                continue
            try:
                data = self.sock.recv(1024)
            except socket.timeout:
                continue
            except OSError as exc:
                self.sock_connected = False
                self.connect_error = str(exc)
                continue

            try:
                qx, qy, qz, qw, t = [float(x) for x in data.decode().split(",")]
            except ValueError:
                continue

            self.display_state = State(
                (qx, qy, qz, qw),
                max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, t)),
            )
            self.last_received_time = time.time()

    def get_state(self):
        return self.display_state, self.last_received_time, self.connect_error, self.sock_connected

    def set_command(self, state: State):
        if not self.sock_connected:
            return
        try:
            self.sock.send(state.as_msg())
        except OSError as exc:
            self.sock_connected = False
            self.connect_error = str(exc)

    def close(self):
        self.running = False
        try:
            self.sock.send(State().as_msg())
        except OSError:
            pass
        try:
            self._receiver.join(timeout=0.5)
        except RuntimeError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
