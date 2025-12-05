import socket
import time
from dataclasses import dataclass
from threading import Thread
from typing import Optional

from scipy.spatial.transform import Rotation as R

THROTTLE_LIMIT = 1.0

@dataclass
class State:
    quat: tuple = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w
    throttle: float = 0.0  # -1 to 1
    accel: tuple = (0.0, 0.0, 0.0)
    gyro: tuple = (0.0, 0.0, 0.0)
    temp_c: float = 0.0
    selected_pid: int = 0
    pid_values: tuple = ((1 / 90, 1 / 90, 1 / 90), (1.0, 1.0, 1.0), (1.0, 1.0, 1.0))

    def as_msg(self) -> bytes:
        qx, qy, qz, qw = self.quat
        flat_pids = ",".join(f"{v:.6f}" for v in self.pid_values[self.selected_pid])
        return (
            f"{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},"
            f"{self.throttle:.2f},"
            f"{self.selected_pid},"
            f"{flat_pids}"
        ).encode()

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
        self.next_connect_time = 0.0
        self.last_received_time: Optional[float] = None
        self.display_state = State()
        self.running = True
        self.pending_state: Optional[State] = None
        self.last_sent_state: Optional[State] = None
        
        self._receiver = Thread(target=self._receive_loop, daemon=True)
        self._receiver.start()
        self._sender = Thread(target=self._send_loop, daemon=True)
        self._sender.start()

    def _ensure_connected(self):
        now = time.time()
        if self.sock_connected:
            return
        if now < self.next_connect_time or (now - self.last_connect_attempt) < 1.0:
            return
        self.last_connect_attempt = now
        try:
            self.sock.connect(self.addr)
            self.sock_connected = True
            self.connect_error = None
            self.next_connect_time = now
        except OSError as exc:
            self.sock_connected = False
            self.connect_error = str(exc)
            # Back off further attempts to avoid busy errors
            self.next_connect_time = now + 5.0

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

            parts = data.decode().split(",")
            try:
                floats = [float(x) for x in parts[:13]]
            except ValueError:
                continue
            if len(floats) < 13:
                continue
            qx, qy, qz, qw, ax, ay, az, gx, gy, gz, temp_c, throttle_val = floats[:12]
            sel_pid = self.display_state.selected_pid
            if len(parts) >= 13:
                try:
                    sel_pid = int(float(parts[12]))
                except ValueError:
                    sel_pid = self.display_state.selected_pid

            self.display_state = State(
                quat=(qx, qy, qz, qw),
                throttle=max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, throttle_val)),
                accel=(ax, ay, az),
                gyro=(gx, gy, gz),
                temp_c=temp_c,
                selected_pid=sel_pid,
            )
            self.last_received_time = time.time()

    def get_state(self):
        return self.display_state, self.last_received_time, self.connect_error, self.sock_connected

    def set_command(self, state: State):
        # Stash latest command; sender thread will transmit when connected
        self.pending_state = state

    def _send_loop(self):
        while self.running:
            if self.pending_state is not None and self.pending_state != self.last_sent_state:
                if not self.sock_connected:
                    self._ensure_connected()
                if self.sock_connected:
                    try:
                        self.sock.send(self.pending_state.as_msg())
                        self.last_sent_state = self.pending_state
                    except OSError as exc:
                        self.sock_connected = False
                        self.connect_error = str(exc)
            time.sleep(0.02)

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
            self._sender.join(timeout=0.5)
        except RuntimeError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
