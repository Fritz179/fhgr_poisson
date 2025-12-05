import socket
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

from scipy.spatial.transform import Rotation as R

THROTTLE_LIMIT = 100.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


@dataclass
class Command:
    roll: float
    pitch: float
    yaw: float
    quat: R
    throttle: float

    pid_selection: int
    pid_data: list[float]


class Connection:
    """UDP helper for receiving commands and responding with state."""

    def __init__(self, bind_address=("0.0.0.0", 5005), on_command=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(bind_address)
        self.sock.settimeout(0.1)

        self._on_command = on_command
        self._latest_command: Optional[Command] = None
        self.sender_socket: Optional[Tuple[str, int]] = None

        self._running = True
        self._lock = threading.Lock()
        self._receiver = threading.Thread(target=self._receive_loop, daemon=True)
        self._receiver.start()

    def _receive_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                command = self._parse_command(data)
            except ValueError as exc:
                print(f"Ignoring malformed command: {exc}")
                continue

            with self._lock:
                self._latest_command = command
                self.sender_socket = addr

            if self._on_command:
                self._on_command()

    def _parse_command(self, data: bytes) -> Command:
        print(data)
        values = [float(x) for x in data.decode().split(",")]
        qx, qy, qz, qw, throttle, pid_selection = values[0:6]
        p, i, d = values[6:]

        rot = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = rot.as_euler("xyz", degrees=True)

        # Wrap yaw to keep it comparable to UI range
        if yaw > 180.0 or yaw < -180.0:
            yaw = ((yaw + 180.0) % 360.0) - 180.0

        throttle = clamp(throttle, -THROTTLE_LIMIT, THROTTLE_LIMIT)
        return Command(roll, pitch, yaw, rot, throttle, int(pid_selection), (p, i, d))

    def get_latest(self) -> Optional[Command]:
        with self._lock:
            return self._latest_command

    def send_state(self, state):
        with self._lock:
            target = self.sender_socket
            throttle = self._latest_command.throttle if self._latest_command else 0.0

        if not target or state is None:
            return

        try:
            payload = state.as_msg(throttle)
            self.sock.sendto(payload, target)
        except OSError as exc:
            print(f"Failed to send state to {target}: {exc}")

    def close(self):
        self._running = False
        try:
            self._receiver.join(timeout=0.5)
        except RuntimeError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
