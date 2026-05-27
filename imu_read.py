"""
Lord Microstrain 3DM-GX4-25 IMU reader
Direct MIP (MicroStrain Inertial Protocol) implementation

Checksum: Fletcher checksum over the full packet including sync bytes
IMU base rate: 500 Hz (GX4-25 default)
"""

import serial
import struct
import time
import math

PORT      = "/dev/ttyACM0"
BAUDRATE  = 115200
BASE_RATE = 500  # 3DM-GX4-25 IMU stream default base rate (Hz)

# MIP packet header
MIP_SYNC1 = 0x75
MIP_SYNC2 = 0x65

# Descriptor Sets
DESC_BASE = 0x01
DESC_3DM  = 0x0C
DESC_IMU  = 0x80  # IMU data packets

# IMU field descriptors
FIELD_ACCEL = 0x04  # Scaled Accelerometer (g)
FIELD_GYRO  = 0x05  # Scaled Gyro (rad/s)
FIELD_MAG   = 0x06  # Scaled Magnetometer (Gauss)
FIELD_EULER = 0x0C  # Euler Angles (rad)


def fletcher_checksum(data: bytes) -> tuple[int, int]:
    """MIP Fletcher checksum over the full packet including sync bytes"""
    b1 = b2 = 0
    for b in data:
        b1 = (b1 + b) & 0xFF
        b2 = (b2 + b1) & 0xFF
    return b1, b2


def build_packet(desc_set: int, fields: list[bytes]) -> bytes:
    payload = b"".join(fields)
    header  = bytes([MIP_SYNC1, MIP_SYNC2, desc_set, len(payload)])
    body    = header + payload
    cs1, cs2 = fletcher_checksum(body)  # full packet including sync
    return body + bytes([cs1, cs2])


def build_field(field_desc: int, data: bytes) -> bytes:
    length = len(data) + 2  # field_len(1) + field_desc(1) + data
    return bytes([length, field_desc]) + data


def cmd_ping() -> bytes:
    return build_packet(DESC_BASE, [build_field(0x01, b"")])


def cmd_set_imu_format(sample_rate_hz: int, fields: list[int]) -> bytes:
    """Configure IMU output fields and sample rate (descriptor set 0x0C, field 0x08)"""
    decimation = max(1, BASE_RATE // sample_rate_hz)
    data = bytes([0x01, len(fields)])  # function=apply, num_descriptors
    for f in fields:
        data += bytes([f, decimation >> 8, decimation & 0xFF])
    return build_packet(DESC_3DM, [build_field(0x08, data)])


def cmd_enable_imu_stream(enable: bool) -> bytes:
    """Enable or disable IMU data stream (descriptor set 0x0C, field 0x11)"""
    data = bytes([0x01, 0x01, 0x01 if enable else 0x00])  # func=apply, dev=IMU
    return build_packet(DESC_3DM, [build_field(0x11, data)])


def parse_imu_packet(payload: bytes) -> dict:
    result = {}
    i = 0
    while i + 1 < len(payload):
        field_len  = payload[i]
        field_desc = payload[i + 1]
        field_data = payload[i + 2: i + field_len]

        if field_desc == FIELD_ACCEL and len(field_data) >= 12:
            result["accel_g"] = struct.unpack(">fff", field_data[:12])
        elif field_desc == FIELD_GYRO and len(field_data) >= 12:
            result["gyro_rads"] = struct.unpack(">fff", field_data[:12])
        elif field_desc == FIELD_MAG and len(field_data) >= 12:
            result["mag_gauss"] = struct.unpack(">fff", field_data[:12])
        elif field_desc == FIELD_EULER and len(field_data) >= 12:
            r, p, y = struct.unpack(">fff", field_data[:12])
            result["euler_deg"] = (math.degrees(r), math.degrees(p), math.degrees(y))

        i += max(field_len, 1)  # avoid infinite loop
    return result


def read_packet(ser: serial.Serial, timeout: float = 2.0) -> tuple[int, bytes] | None:
    """Parse one MIP packet from the stream; returns (desc_set, payload)"""
    deadline = time.time() + timeout
    buf = b""

    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk

        while True:
            idx = buf.find(bytes([MIP_SYNC1, MIP_SYNC2]))
            if idx == -1:
                buf = buf[-1:]
                break
            buf = buf[idx:]
            if len(buf) < 4:
                break

            payload_len = buf[3]
            total_len   = 4 + payload_len + 2

            # wait until the full packet has arrived
            if len(buf) < total_len:
                break

            packet  = buf[:total_len]
            payload = packet[4: 4 + payload_len]
            cs_got  = (packet[-2], packet[-1])
            cs_exp  = fletcher_checksum(packet[:-2])

            if cs_got != cs_exp:
                buf = buf[1:]  # checksum mismatch — resync from next byte
                continue

            buf = buf[total_len:]
            return packet[2], payload  # desc_set, payload

    return None


def send_and_ack(ser: serial.Serial, packet: bytes, cmd_desc: int,
                 label: str, retries: int = 3) -> bool:
    """Send command and wait for ACK (ignore IMU stream packets; retry on NACK/timeout)"""
    for attempt in range(1, retries + 1):
        ser.reset_input_buffer()
        ser.write(packet)
        deadline = time.time() + 1.5
        while time.time() < deadline:
            result = read_packet(ser, timeout=deadline - time.time())
            if result is None:
                break
            desc_set, payload = result
            if desc_set == DESC_IMU:
                continue  # ignore stream data; keep waiting for ACK
            if len(payload) >= 4 and payload[1] == 0xF1:
                echo       = payload[2]
                error_code = payload[3]
                if echo == cmd_desc and error_code == 0x00:
                    print(f"  {label}: OK")
                    return True
                else:
                    print(f"  {label}: NACK echo=0x{echo:02X} error=0x{error_code:02X} (attempt {attempt}/{retries})")
                    break
            # ignore other responses and keep waiting
        else:
            print(f"  {label}: no response (attempt {attempt}/{retries})")

    return False


def main():
    print(f"Opening port {PORT}...")
    with serial.Serial(PORT, BAUDRATE, timeout=0.05, dsrdtr=False, rtscts=False) as ser:
        # wait for device to stabilize after DTR reset on port open
        time.sleep(1.5)
        ser.reset_input_buffer()

        # Ping
        if not send_and_ack(ser, cmd_ping(), 0x01, "Ping"):
            print("No device response — check connection and power")
            return

        # IMU format: accel + gyro + Euler @ 100 Hz
        fields = [FIELD_ACCEL, FIELD_GYRO, FIELD_EULER]
        if not send_and_ack(ser, cmd_set_imu_format(100, fields), 0x08, "IMU format"):
            print("Format setup failed — trying stream anyway")

        # start stream
        if not send_and_ack(ser, cmd_enable_imu_stream(True), 0x11, "Start stream"):
            print("Failed to start stream")
            return

        print("\nReceiving data... (Ctrl+C to quit)\n")
        try:
            while True:
                pkt = read_packet(ser, timeout=2.0)
                if pkt is None:
                    print("  (no data — timeout)")
                    continue

                desc_set, payload = pkt
                if desc_set != DESC_IMU:
                    continue  # process data packets only

                data = parse_imu_packet(payload)
                if not data:
                    continue

                parts = []
                if "accel_g" in data:
                    ax, ay, az = data["accel_g"]
                    parts.append(f"Accel(g)   x={ax:+.4f} y={ay:+.4f} z={az:+.4f}")
                if "gyro_rads" in data:
                    gx, gy, gz = data["gyro_rads"]
                    parts.append(f"Gyro(r/s)  x={gx:+.4f} y={gy:+.4f} z={gz:+.4f}")
                if "euler_deg" in data:
                    r, p, y = data["euler_deg"]
                    parts.append(f"Euler(deg) roll={r:+.2f} pitch={p:+.2f} yaw={y:+.2f}")

                if parts:
                    print("  |  ".join(parts))

        except KeyboardInterrupt:
            print("\nShutting down...")
            ser.write(cmd_enable_imu_stream(False))
            time.sleep(0.2)
            print("Stream stopped")


if __name__ == "__main__":
    main()