"""
MicrostrainIMU test suite
3DM-GX4-25 — run before integrating into the L1AC loop
"""

import time
import math
import threading
import unittest
from unittest.mock import patch, MagicMock, call
import struct

from microstrain_imu import (
    MicrostrainIMU, IMUReading,
    MIP_SYNC1, MIP_SYNC2,
    DESC_BASE, DESC_3DM, DESC_IMU,
    FIELD_ACCEL, FIELD_GYRO, FIELD_EULER,
    PORT, BAUDRATE,
)


# ── Helpers ────────────────────────────────────────────────────────────────────

def make_floats_be(*values: float) -> bytes:
    """Pack floats as big-endian for building fake IMU payloads."""
    return struct.pack(f">{len(values)}f", *values)


def build_mip_packet(desc_set: int, payload: bytes) -> bytes:
    """Build a valid MIP packet with correct Fletcher checksum."""
    header = bytes([MIP_SYNC1, MIP_SYNC2, desc_set, len(payload)])
    body   = header + payload
    b1 = b2 = 0
    for b in body:
        b1 = (b1 + b) & 0xFF
        b2 = (b2 + b1) & 0xFF
    return body + bytes([b1, b2])


def build_imu_field(field_desc: int, *floats: float) -> bytes:
    data   = make_floats_be(*floats)
    length = len(data) + 2
    return bytes([length, field_desc]) + data


def build_imu_packet(accel_g=(0.0, 0.0, 1.0),
                     gyro_rads=(0.0, 0.0, 0.0),
                     euler_rad=(0.0, 0.0, 0.0)) -> bytes:
    """Full IMU data packet with accel + gyro + Euler fields."""
    payload = (
        build_imu_field(FIELD_ACCEL, *accel_g)
      + build_imu_field(FIELD_GYRO,  *gyro_rads)
      + build_imu_field(FIELD_EULER, *euler_rad)
    )
    return build_mip_packet(DESC_IMU, payload)


def build_ack_packet(desc_set: int, cmd_echo: int, error: int = 0x00) -> bytes:
    payload = bytes([0x04, 0xF1, cmd_echo, error])
    return build_mip_packet(desc_set, payload)


def make_serial_mock(responses: list[bytes]) -> MagicMock:
    """
    Serial mock whose .read() drains a pre-loaded byte buffer.
    Each call to read(n) returns up to n bytes from the front.
    """
    buf = bytearray(b"".join(responses))

    mock = MagicMock()
    mock.is_open = True
    mock.in_waiting = 1

    def fake_read(n=1):
        chunk = bytes(buf[:n])
        del buf[:n]
        return chunk

    mock.read.side_effect = fake_read
    mock.reset_input_buffer = MagicMock()
    mock.write = MagicMock()
    mock.close = MagicMock()
    return mock


# ── 1. Unit tests: packet building ────────────────────────────────────────────

class TestChecksumAndPacketBuilding(unittest.TestCase):

    def test_fletcher_known_vector(self):
        # Known-good: single byte 0x01 → b1=1, b2=1
        b1, b2 = MicrostrainIMU._fletcher_checksum(bytes([0x01]))
        self.assertEqual((b1, b2), (0x01, 0x01))

    def test_fletcher_rollover(self):
        # 256 × 0xFF — both accumulators must stay in 0–255
        data = bytes([0xFF] * 256)
        b1, b2 = MicrostrainIMU._fletcher_checksum(data)
        self.assertLessEqual(b1, 0xFF)
        self.assertLessEqual(b2, 0xFF)

    def test_ping_packet_structure(self):
        imu  = MicrostrainIMU.__new__(MicrostrainIMU)
        ping = imu._cmd_ping()
        self.assertEqual(ping[0], MIP_SYNC1)
        self.assertEqual(ping[1], MIP_SYNC2)
        self.assertEqual(ping[2], DESC_BASE)

    def test_imu_format_packet_contains_decimation(self):
        imu    = MicrostrainIMU.__new__(MicrostrainIMU)
        packet = imu._cmd_set_imu_format(100, [FIELD_ACCEL, FIELD_GYRO])
        # decimation for 100 Hz = 500//100 = 5 → bytes [0x00, 0x05]
        self.assertIn(bytes([0x00, 0x05]), packet)

    def test_enable_stream_payload(self):
        imu = MicrostrainIMU.__new__(MicrostrainIMU)
        pkt = imu._cmd_enable_imu_stream(True)
        self.assertIn(bytes([0x01]), pkt)

    def test_disable_stream_payload(self):
        imu = MicrostrainIMU.__new__(MicrostrainIMU)
        pkt = imu._cmd_enable_imu_stream(False)
        self.assertIn(bytes([0x00]), pkt)

    def test_packet_checksum_roundtrip(self):
        """Any packet we build must pass its own checksum verification."""
        imu    = MicrostrainIMU.__new__(MicrostrainIMU)
        packet = imu._cmd_ping()
        cs_exp = MicrostrainIMU._fletcher_checksum(packet[:-2])
        cs_got = (packet[-2], packet[-1])
        self.assertEqual(cs_got, cs_exp)


# ── 2. Unit tests: payload parsing ────────────────────────────────────────────

class TestPayloadParsing(unittest.TestCase):

    def _make_imu(self):
        imu = MicrostrainIMU.__new__(MicrostrainIMU)
        imu._GRAVITY = 9.80665
        return imu

    def test_accel_parsed_and_converted_to_ms2(self):
        imu     = self._make_imu()
        payload = build_imu_field(FIELD_ACCEL, 0.0, 0.0, 1.0)   # 1 g upward
        reading = imu._parse_payload(payload)
        self.assertTrue(reading.valid)
        self.assertAlmostEqual(reading.accel_z, 9.80665, places=3)

    def test_gyro_parsed_correctly(self):
        imu     = self._make_imu()
        payload = build_imu_field(FIELD_GYRO, 0.1, -0.2, 0.3)
        reading = imu._parse_payload(payload)
        self.assertAlmostEqual(reading.gyro_x,  0.1, places=5)
        self.assertAlmostEqual(reading.gyro_y, -0.2, places=5)
        self.assertAlmostEqual(reading.gyro_z,  0.3, places=5)

    def test_euler_converted_to_degrees(self):
        imu     = self._make_imu()
        r, p, y = math.pi / 6, math.pi / 4, math.pi / 2
        payload = build_imu_field(FIELD_EULER, r, p, y)
        reading = imu._parse_payload(payload)
        self.assertAlmostEqual(reading.roll,  30.0, places=3)
        self.assertAlmostEqual(reading.pitch, 45.0, places=3)
        self.assertAlmostEqual(reading.yaw,   90.0, places=3)

    def test_all_fields_in_one_packet(self):
        imu     = self._make_imu()
        payload = (
            build_imu_field(FIELD_ACCEL, 0.0, 0.0, 1.0)
          + build_imu_field(FIELD_GYRO,  0.1, 0.2, 0.3)
          + build_imu_field(FIELD_EULER, 0.0, 0.0, math.pi)
        )
        reading = imu._parse_payload(payload)
        self.assertTrue(reading.valid)
        self.assertAlmostEqual(reading.accel_z, 9.80665, places=3)
        self.assertAlmostEqual(reading.gyro_x,  0.1, places=5)
        self.assertAlmostEqual(reading.yaw,     180.0, places=3)

    def test_empty_payload_returns_invalid(self):
        imu     = self._make_imu()
        reading = imu._parse_payload(b"")
        self.assertFalse(reading.valid)

    def test_truncated_field_does_not_crash(self):
        imu     = self._make_imu()
        payload = bytes([0x03, FIELD_ACCEL, 0x00])   # length=3 but no float data
        reading = imu._parse_payload(payload)
        self.assertFalse(reading.valid)

    def test_unknown_field_descriptor_ignored(self):
        imu     = self._make_imu()
        unknown = bytes([0x04, 0xFF, 0x00, 0x00])   # unknown descriptor 0xFF
        payload = unknown + build_imu_field(FIELD_GYRO, 1.0, 2.0, 3.0)
        reading = imu._parse_payload(payload)
        self.assertAlmostEqual(reading.gyro_x, 1.0, places=5)


# ── 3. Unit tests: packet reader / sync ───────────────────────────────────────

class TestPacketReader(unittest.TestCase):

    def _make_imu_with_serial(self, serial_mock):
        imu       = MicrostrainIMU.__new__(MicrostrainIMU)
        imu._ser  = serial_mock
        imu._buf  = b""
        return imu

    def test_reads_valid_imu_packet(self):
        raw  = build_imu_packet(accel_g=(0.0, 0.0, 1.0))
        mock = make_serial_mock([raw])
        imu  = self._make_imu_with_serial(mock)
        result = imu._read_packet(timeout=1.0)
        self.assertIsNotNone(result)
        desc_set, payload = result
        self.assertEqual(desc_set, DESC_IMU)

    def test_resyncs_after_garbage_prefix(self):
        garbage = bytes([0xDE, 0xAD, 0xBE, 0xEF])
        raw     = build_imu_packet()
        mock    = make_serial_mock([garbage + raw])
        imu     = self._make_imu_with_serial(mock)
        result  = imu._read_packet(timeout=1.0)
        self.assertIsNotNone(result)

    def test_bad_checksum_triggers_resync(self):
        raw        = bytearray(build_imu_packet())
        raw[-1]   ^= 0xFF                             # corrupt last checksum byte
        good       = build_imu_packet(gyro_rads=(1.0, 0.0, 0.0))
        mock       = make_serial_mock([bytes(raw) + good])
        imu        = self._make_imu_with_serial(mock)
        result     = imu._read_packet(timeout=1.0)
        # should skip the bad packet and return the good one
        self.assertIsNotNone(result)
        _, payload = result
        reading    = imu._parse_payload(payload)
        self.assertAlmostEqual(reading.gyro_x, 1.0, places=4)

    def test_timeout_on_empty_stream(self):
        mock       = make_serial_mock([])
        mock.read.side_effect = lambda n=1: b""
        imu        = self._make_imu_with_serial(mock)
        t0         = time.time()
        result     = imu._read_packet(timeout=0.2)
        elapsed    = time.time() - t0
        self.assertIsNone(result)
        self.assertGreaterEqual(elapsed, 0.18)

    def test_two_consecutive_packets_parsed(self):
        p1   = build_imu_packet(accel_g=(0.5, 0.0, 0.9))
        p2   = build_imu_packet(accel_g=(0.1, 0.2, 0.8))
        mock = make_serial_mock([p1 + p2])
        imu  = self._make_imu_with_serial(mock)

        r1 = imu._read_packet(timeout=1.0)
        r2 = imu._read_packet(timeout=1.0)
        self.assertIsNotNone(r1)
        self.assertIsNotNone(r2)


# ── 4. Integration tests: connect / disconnect ────────────────────────────────

class TestConnectDisconnect(unittest.TestCase):

    def _responses_for_connect(self):
        """ACKs for ping + format + stream-enable commands."""
        return [
            build_ack_packet(DESC_BASE, 0x01),   # ping ACK
            build_ack_packet(DESC_3DM,  0x08),   # format ACK
            build_ack_packet(DESC_3DM,  0x11),   # stream-enable ACK
        ]

    @patch("microstrain_imu.serial.Serial")
    @patch("microstrain_imu.time.sleep")
    def test_connect_success(self, mock_sleep, mock_serial_cls):
        mock_ser = make_serial_mock(self._responses_for_connect())
        mock_serial_cls.return_value = mock_ser

        imu = MicrostrainIMU(port=PORT, baudrate=BAUDRATE, background=False)
        ok  = imu.connect()

        self.assertTrue(ok)
        self.assertTrue(imu.is_connected())
        imu.disconnect()

    @patch("microstrain_imu.serial.Serial")
    @patch("microstrain_imu.time.sleep")
    def test_connect_fails_on_no_ping_response(self, mock_sleep, mock_serial_cls):
        mock_ser = make_serial_mock([])
        mock_ser.read.side_effect = lambda n=1: b""
        mock_serial_cls.return_value = mock_ser

        imu = MicrostrainIMU(port=PORT, baudrate=BAUDRATE, background=False)
        ok  = imu.connect()
        self.assertFalse(ok)

    @patch("microstrain_imu.serial.Serial")
    @patch("microstrain_imu.time.sleep")
    def test_disconnect_sends_stop_command(self, mock_sleep, mock_serial_cls):
        mock_ser = make_serial_mock(self._responses_for_connect())
        mock_serial_cls.return_value = mock_ser

        imu = MicrostrainIMU(background=False)
        imu.connect()
        imu.disconnect()

        written = b"".join(c.args[0] for c in mock_ser.write.call_args_list)
        # The disable-stream command (0x00 flag) must appear somewhere
        self.assertIn(bytes([0x00]), written)
        mock_ser.close.assert_called_once()


# ── 5. Integration tests: blocking read ───────────────────────────────────────

class TestBlockingRead(unittest.TestCase):

    @patch("microstrain_imu.serial.Serial")
    @patch("microstrain_imu.time.sleep")
    def test_read_returns_valid_reading(self, mock_sleep, mock_serial_cls):
        ack_responses = [
            build_ack_packet(DESC_BASE, 0x01),
            build_ack_packet(DESC_3DM,  0x08),
            build_ack_packet(DESC_3DM,  0x11),
        ]
        data_packet = build_imu_packet(
            accel_g=(0.0, 0.0, 1.0),
            gyro_rads=(0.05, -0.03, 0.01),
        )
        mock_ser = make_serial_mock(ack_responses + [data_packet])
        mock_serial_cls.return_value = mock_ser

        imu = MicrostrainIMU(background=False)
        imu.connect()
        reading = imu.read(timeout=1.0)
        imu.disconnect()

        self.assertTrue(reading.valid)
        self.assertAlmostEqual(reading.accel_z, 9.80665, places=3)
        self.assertAlmostEqual(reading.gyro_x,  0.05,    places=4)

    @patch("microstrain_imu.serial.Serial")
    @patch("microstrain_imu.time.sleep")
    def test_read_timeout_returns_invalid(self, mock_sleep, mock_serial_cls):
        ack_responses = [
            build_ack_packet(DESC_BASE, 0x01),
            build_ack_packet(DESC_3DM,  0x08),
            build_ack_packet(DESC_3DM,  0x11),
        ]
        mock_ser = make_serial_mock(ack_responses)
        mock_ser.read.side_effect = lambda n=1: b""   # no data after ACKs
        mock_serial_cls.return_value = mock_ser

        imu     = MicrostrainIMU(background=False)
        imu.connect()
        reading = imu.read(timeout=0.15)
        imu.disconnect()

        self.assertFalse(reading.valid)


# ── 6. Integration tests: background thread ───────────────────────────────────

class TestBackgroundThread(unittest.TestCase):

    def _make_streaming_serial(self, n_packets: int = 20) -> MagicMock:
        acks = [
            build_ack_packet(DESC_BASE, 0x01),
            build_ack_packet(DESC_3DM,  0x08),
            build_ack_packet(DESC_3DM,  0x11),
        ]
        packets = [
            build_imu_packet(
                accel_g=(0.0, 0.0, 1.0),
                gyro_rads=(0.01 * i, 0.0, 0.0),
            )
            for i in range(n_packets)
        ]
        return make_serial_mock(acks + packets)

    @patch("microstrain_imu.serial.Serial")
    @patch("microstrain_imu.time.sleep")
    def test_latest_updated_by_background_thread(self, mock_sleep, mock_serial_cls):
        mock_ser = self._make_streaming_serial()
        mock_serial_cls.return_value = mock_ser

        imu = MicrostrainIMU(background=True)
        imu.connect()
        time.sleep(0.05)   # give thread time to process a few packets
        reading = imu.latest
        imu.disconnect()

        self.assertTrue(reading.valid)
        self.assertAlmostEqual(reading.accel_z, 9.80665, places=3)

    @patch("microstrain_imu.serial.Serial")
    @patch("microstrain_imu.time.sleep")
    def test_latest_is_thread_safe(self, mock_sleep, mock_serial_cls):
        """Read .latest from multiple threads simultaneously — must not raise."""
        mock_ser = self._make_streaming_serial(n_packets=100)
        mock_serial_cls.return_value = mock_ser

        imu = MicrostrainIMU(background=True)
        imu.connect()

        errors = []
        def reader():
            for _ in range(50):
                try:
                    _ = imu.latest
                except Exception as e:
                    errors.append(e)

        threads = [threading.Thread(target=reader) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        imu.disconnect()
        self.assertEqual(errors, [], f"Thread-safety errors: {errors}")

    @patch("microstrain_imu.serial.Serial")
    @patch("microstrain_imu.time.sleep")
    def test_background_thread_stops_on_disconnect(self, mock_sleep, mock_serial_cls):
        mock_ser = self._make_streaming_serial()
        mock_serial_cls.return_value = mock_ser

        imu = MicrostrainIMU(background=True)
        imu.connect()
        thread = imu._thread
        imu.disconnect()

        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive(), "Background thread still alive after disconnect")


# ── 7. IMUReading dataclass ───────────────────────────────────────────────────

class TestIMUReading(unittest.TestCase):

    def test_default_is_invalid(self):
        r = IMUReading()
        self.assertFalse(r.valid)

    def test_accel_property_returns_tuple(self):
        r = IMUReading(accel_x=1.0, accel_y=2.0, accel_z=3.0)
        self.assertEqual(r.accel, (1.0, 2.0, 3.0))

    def test_gyro_property_returns_tuple(self):
        r = IMUReading(gyro_x=0.1, gyro_y=0.2, gyro_z=0.3)
        self.assertEqual(r.gyro, (0.1, 0.2, 0.3))

    def test_accel_np_shape(self):
        r = IMUReading(accel_x=1.0, accel_y=2.0, accel_z=3.0)
        self.assertEqual(r.accel_np.shape, (3,))

    def test_gyro_np_values(self):
        import numpy as np
        r = IMUReading(gyro_x=0.5, gyro_y=-0.5, gyro_z=1.0)
        np.testing.assert_array_almost_equal(r.gyro_np, [0.5, -0.5, 1.0])


# ── 8. Live hardware smoke test (skipped unless --live flag present) ───────────

class TestLiveHardware(unittest.TestCase):
    """
    Skipped by default.  Run with:
        python -m pytest test_imu.py -k live --live
    or:
        python test_imu.py --live
    Requires the IMU to be connected on PORT.
    """

    LIVE = False   # flipped to True by --live CLI flag

    def setUp(self):
        if not self.LIVE:
            self.skipTest("Live hardware test — pass --live to run")

    def test_connect_and_read_10_packets(self):
        imu = MicrostrainIMU(sample_rate_hz=100, background=False)
        self.assertTrue(imu.connect(), "IMU did not connect")
        try:
            readings = []
            for _ in range(10):
                r = imu.read(timeout=2.0)
                self.assertTrue(r.valid, "Got invalid reading from hardware")
                readings.append(r)
            # Static sensor: |accel| should be close to 1 g = 9.807 m/s²
            for r in readings:
                mag = (r.accel_x**2 + r.accel_y**2 + r.accel_z**2) ** 0.5
                self.assertAlmostEqual(mag, 9.80665, delta=0.5,
                                       msg=f"Accel magnitude {mag:.3f} far from 1 g")
        finally:
            imu.disconnect()

    def test_background_mode_sustained_rate(self):
        """Verify background thread delivers samples at ≥ 90% of requested rate."""
        imu = MicrostrainIMU(sample_rate_hz=100, background=True)
        self.assertTrue(imu.connect())
        try:
            samples, t0 = 0, time.time()
            last_ts = None
            while time.time() - t0 < 2.0:
                r = imu.latest
                if r.valid and r.timestamp != last_ts:
                    samples  += 1
                    last_ts   = r.timestamp
                time.sleep(0.005)
            elapsed  = time.time() - t0
            rate     = samples / elapsed
            self.assertGreater(rate, 90.0,
                               f"Sample rate {rate:.1f} Hz below 90 Hz threshold")
        finally:
            imu.disconnect()


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    if "--live" in sys.argv:
        TestLiveHardware.LIVE = True
        sys.argv.remove("--live")
    unittest.main(verbosity=2)