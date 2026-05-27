"""
Lord Microstrain 3DM-GX4-25 IMU — class interface
Designed for use with the OmBURo L1 Adaptive Controller

NOTE: THIS CODE WAS WRITTEN BY CLAUDE AND IS NOT GUARANTEED TO BE CORRECT OR BUG-FREE.  USE WITH CAUTION.
"""

import serial
import struct
import time
import math
import threading
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


PORT      = "/dev/ttyACM0"
BAUDRATE  = 115200
BASE_RATE = 500  # Hz

MIP_SYNC1 = 0x75
MIP_SYNC2 = 0x65

DESC_BASE = 0x01
DESC_3DM  = 0x0C
DESC_IMU  = 0x80

FIELD_ACCEL = 0x04
FIELD_GYRO  = 0x05
FIELD_MAG   = 0x06
FIELD_EULER = 0x0C


# ── Data container ─────────────────────────────────────────────────────────────

@dataclass
class IMUReading:
    """
    One timestamped IMU sample.  Accelerometer values are in m/s² (converted
    from the sensor's native g output).  Gyro is rad/s.  Euler in degrees.
    All fields default to zero so callers never receive None.
    """
    timestamp:   float = 0.0

    accel_x: float = 0.0   # m/s²
    accel_y: float = 0.0
    accel_z: float = 0.0

    gyro_x:  float = 0.0   # rad/s
    gyro_y:  float = 0.0
    gyro_z:  float = 0.0

    roll:    float = 0.0   # deg
    pitch:   float = 0.0
    yaw:     float = 0.0

    valid:   bool  = False

    @property
    def accel(self) -> tuple[float, float, float]:
        return (self.accel_x, self.accel_y, self.accel_z)

    @property
    def gyro(self) -> tuple[float, float, float]:
        return (self.gyro_x, self.gyro_y, self.gyro_z)

    @property
    def accel_np(self) -> np.ndarray:
        return np.array([self.accel_x, self.accel_y, self.accel_z])

    @property
    def gyro_np(self) -> np.ndarray:
        return np.array([self.gyro_x, self.gyro_y, self.gyro_z])


# ── Main class ─────────────────────────────────────────────────────────────────

class MicrostrainIMU:
    """
    Non-blocking reader for the Lord Microstrain 3DM-GX4-25.

    Usage (blocking reads, simplest):
        imu = MicrostrainIMU()
        imu.connect()
        reading = imu.read()          # blocks up to timeout
        print(reading.accel, reading.gyro)
        imu.disconnect()

    Usage (background thread, for real-time control loops):
        imu = MicrostrainIMU(sample_rate_hz=200, background=True)
        imu.connect()
        ...
        reading = imu.latest           # always fresh, never blocks
        state = estimator.update(reading.gyro, reading.accel, ...)
        ...
        imu.disconnect()

    The background thread publishes to `self.latest` under a lock so the
    control loop always gets a consistent snapshot without blocking.
    """

    _GRAVITY = 9.80665   # m/s² per g

    def __init__(
        self,
        port:            str  = PORT,
        baudrate:        int  = BAUDRATE,
        sample_rate_hz:  int  = 200,
        fields:          list[int] | None = None,
        background:      bool = False,
        connect_timeout: float = 5.0,
    ):
        self.port           = port
        self.baudrate       = baudrate
        self.sample_rate_hz = sample_rate_hz
        self.fields         = fields or [FIELD_ACCEL, FIELD_GYRO, FIELD_EULER]
        self.background     = background
        self.connect_timeout = connect_timeout

        self._ser:    Optional[serial.Serial] = None
        self._buf:    bytes = b""
        self._lock:   threading.Lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop:   threading.Event = threading.Event()
        self._latest: IMUReading = IMUReading()

    # ── Public API ─────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Open serial port, ping device, configure data format, start stream.
        Returns True on success.  Call disconnect() when done.
        """
        print(f"[IMU] Opening {self.port} @ {self.baudrate} baud...")
        self._ser = serial.Serial(
            self.port, self.baudrate,
            timeout=0.05, dsrdtr=False, rtscts=False
        )
        time.sleep(1.5)
        self._ser.reset_input_buffer()

        if not self._send_and_ack(self._cmd_ping(), 0x01, "Ping"):
            print("[IMU] No response — check connection and power.")
            self._ser.close()
            return False

        if not self._send_and_ack(
            self._cmd_set_imu_format(self.sample_rate_hz, self.fields),
            0x08, f"Set format ({self.sample_rate_hz} Hz)"
        ):
            print("[IMU] Format config failed — attempting stream anyway.")

        if not self._send_and_ack(
            self._cmd_enable_imu_stream(True), 0x11, "Start stream"
        ):
            print("[IMU] Failed to start stream.")
            self._ser.close()
            return False

        if self.background:
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._background_loop, daemon=True, name="IMU-reader"
            )
            self._thread.start()
            print("[IMU] Background thread started.")

        print("[IMU] Ready.")
        return True

    def disconnect(self):
        """Stop stream, join background thread, close port."""
        if self.background:
            self._stop.set()
            if self._thread:
                self._thread.join(timeout=2.0)

        if self._ser and self._ser.is_open:
            try:
                self._ser.write(self._cmd_enable_imu_stream(False))
                time.sleep(0.2)
            except serial.SerialException:
                pass
            self._ser.close()
        print("[IMU] Disconnected.")

    def read(self, timeout: float = 1.0) -> IMUReading:
        """
        Blocking read: wait for and return the next valid IMU packet.
        Not for use when background=True (use .latest instead).
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            result = self._read_packet(timeout=deadline - time.time())
            if result is None:
                break
            desc_set, payload = result
            if desc_set == DESC_IMU:
                reading = self._parse_payload(payload)
                if reading.valid:
                    return reading
        return IMUReading()   # invalid sentinel

    @property
    def latest(self) -> IMUReading:
        """Thread-safe snapshot of the most recent sample (background mode)."""
        with self._lock:
            return self._latest

    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ── Background thread ──────────────────────────────────────────

    def _background_loop(self):
        while not self._stop.is_set():
            if not self._ser or not self._ser.is_open:
                break
            result = self._read_packet(timeout=0.1)
            if result is None:
                continue
            desc_set, payload = result
            if desc_set != DESC_IMU:
                continue
            reading = self._parse_payload(payload)
            if reading.valid:
                with self._lock:
                    self._latest = reading

    # ── Packet parsing ─────────────────────────────────────────────

    def _parse_payload(self, payload: bytes) -> IMUReading:
        reading = IMUReading(timestamp=time.time())
        i = 0
        while i + 1 < len(payload):
            field_len  = payload[i]
            field_desc = payload[i + 1]
            field_data = payload[i + 2: i + field_len]

            if field_desc == FIELD_ACCEL and len(field_data) >= 12:
                ax, ay, az = struct.unpack(">fff", field_data[:12])
                reading.accel_x = ax * self._GRAVITY
                reading.accel_y = ay * self._GRAVITY
                reading.accel_z = az * self._GRAVITY
                reading.valid = True

            elif field_desc == FIELD_GYRO and len(field_data) >= 12:
                reading.gyro_x, reading.gyro_y, reading.gyro_z = \
                    struct.unpack(">fff", field_data[:12])
                reading.valid = True

            elif field_desc == FIELD_EULER and len(field_data) >= 12:
                r, p, y = struct.unpack(">fff", field_data[:12])
                reading.roll  = math.degrees(r)
                reading.pitch = math.degrees(p)
                reading.yaw   = math.degrees(y)

            i += max(field_len, 1)

        return reading

    def _read_packet(self, timeout: float = 2.0) -> tuple[int, bytes] | None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self._ser:
                return None
            chunk = self._ser.read(self._ser.in_waiting or 1)
            if chunk:
                self._buf += chunk

            while True:
                idx = self._buf.find(bytes([MIP_SYNC1, MIP_SYNC2]))
                if idx == -1:
                    self._buf = self._buf[-1:]
                    break
                self._buf = self._buf[idx:]
                if len(self._buf) < 4:
                    break

                payload_len = self._buf[3]
                total_len   = 4 + payload_len + 2
                if len(self._buf) < total_len:
                    break

                packet  = self._buf[:total_len]
                payload = packet[4: 4 + payload_len]
                cs_got  = (packet[-2], packet[-1])
                cs_exp  = self._fletcher_checksum(packet[:-2])

                if cs_got != cs_exp:
                    self._buf = self._buf[1:]
                    continue

                self._buf = self._buf[total_len:]
                return packet[2], payload

        return None

    # ── Command builders ───────────────────────────────────────────

    @staticmethod
    def _fletcher_checksum(data: bytes) -> tuple[int, int]:
        b1 = b2 = 0
        for b in data:
            b1 = (b1 + b) & 0xFF
            b2 = (b2 + b1) & 0xFF
        return b1, b2

    @classmethod
    def _build_packet(cls, desc_set: int, fields: list[bytes]) -> bytes:
        payload = b"".join(fields)
        header  = bytes([MIP_SYNC1, MIP_SYNC2, desc_set, len(payload)])
        body    = header + payload
        cs1, cs2 = cls._fletcher_checksum(body)
        return body + bytes([cs1, cs2])

    @classmethod
    def _build_field(cls, field_desc: int, data: bytes) -> bytes:
        return bytes([len(data) + 2, field_desc]) + data

    def _cmd_ping(self) -> bytes:
        return self._build_packet(DESC_BASE, [self._build_field(0x01, b"")])

    def _cmd_set_imu_format(self, rate_hz: int, fields: list[int]) -> bytes:
        decimation = max(1, BASE_RATE // rate_hz)
        data = bytes([0x01, len(fields)])
        for f in fields:
            data += bytes([f, decimation >> 8, decimation & 0xFF])
        return self._build_packet(DESC_3DM, [self._build_field(0x08, data)])

    def _cmd_enable_imu_stream(self, enable: bool) -> bytes:
        data = bytes([0x01, 0x01, 0x01 if enable else 0x00])
        return self._build_packet(DESC_3DM, [self._build_field(0x11, data)])

    def _send_and_ack(self, packet: bytes, cmd_desc: int,
                      label: str, retries: int = 3) -> bool:
        for attempt in range(1, retries + 1):
            self._ser.reset_input_buffer()
            self._ser.write(packet)
            deadline = time.time() + 1.5
            while time.time() < deadline:
                result = self._read_packet(timeout=deadline - time.time())
                if result is None:
                    break
                desc_set, payload = result
                if desc_set == DESC_IMU:
                    continue
                if len(payload) >= 4 and payload[1] == 0xF1:
                    echo, error_code = payload[2], payload[3]
                    if echo == cmd_desc and error_code == 0x00:
                        print(f"  [IMU] {label}: OK")
                        return True
                    print(f"  [IMU] {label}: NACK 0x{error_code:02X} (attempt {attempt}/{retries})")
                    break
            else:
                print(f"  [IMU] {label}: timeout (attempt {attempt}/{retries})")
        return False