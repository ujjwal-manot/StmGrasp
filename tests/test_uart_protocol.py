"""
Tests for the UART protocol packet construction and parsing.
Ported from:
  - esp32_brain/comms.cpp  (ESP32 side: build + parse)
  - stm32_control/uart_protocol.c (STM32 side: build + parse)

Packet format: [0xAA] [CMD] [LEN] [PAYLOAD...] [XOR_CHECKSUM]
  - SYNC:     0xAA (1 byte)
  - CMD:      Command/response ID (1 byte)
  - LEN:      Number of payload bytes (1 byte, 0-200 for STM32, 0-72 for ESP32)
  - PAYLOAD:  LEN bytes of data
  - CHECKSUM: XOR of CMD ^ LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n-1]
"""

import struct
import pytest

# ---------------------------------------------------------------------------
# Constants ported from config.h and main.h
# ---------------------------------------------------------------------------

UART_SYNC_BYTE = 0xAA
UART_MAX_PAYLOAD_ESP32 = 72   # ESP32 limit (from config.h)
UART_MAX_PAYLOAD_STM32 = 200  # STM32 limit (from main.h)

# Command IDs (ESP32 -> STM32)
CMD_GRIP_OPEN = 0x01
CMD_GRIP_CLOSE = 0x02
CMD_SET_FORCE = 0x03
CMD_TAP = 0x04
CMD_SERVO_MOVE = 0x05
CMD_ESTOP = 0x06
CMD_REQUEST_DEPTH = 0x07
CMD_REQUEST_IMU_TAP = 0x24
CMD_REQUEST_MIC_TAP = 0x25

# Response IDs (STM32 -> ESP32)
RSP_ACK = 0x80
RSP_POSITION = 0x81
RSP_DEPTH_GRID = 0x82
RSP_STATUS = 0x83
RSP_ERROR = 0x84
RSP_IMU_TAP_DATA = 0x92
RSP_MIC_TAP_DATA = 0x93

# Error codes
ERR_CHECKSUM_FAIL = 0x02


# ---------------------------------------------------------------------------
# Python port of the UART protocol logic
# ---------------------------------------------------------------------------

def compute_checksum(cmd, data):
    """
    XOR checksum: CMD ^ LEN ^ DATA[0] ^ ... ^ DATA[n-1]
    Matches both ESP32 (_computeChecksum) and STM32 (sendResponse) implementations.
    """
    xor_val = cmd ^ len(data)
    for b in data:
        xor_val ^= b
    return xor_val & 0xFF


def build_packet(cmd, payload=b""):
    """
    Build a complete UART packet: [SYNC][CMD][LEN][PAYLOAD][CHECKSUM]
    """
    if not isinstance(payload, (bytes, bytearray)):
        payload = bytes(payload)
    length = len(payload)
    checksum = compute_checksum(cmd, payload)
    packet = bytearray()
    packet.append(UART_SYNC_BYTE)
    packet.append(cmd)
    packet.append(length)
    packet.extend(payload)
    packet.append(checksum)
    return bytes(packet)


class UARTParser:
    """
    Python port of the UART packet parser state machine.
    Matches the logic in both comms.cpp (ESP32) and uart_protocol.c (STM32).
    """

    # Parse states
    WAIT_SYNC = 0
    WAIT_CMD = 1
    WAIT_LEN = 2
    WAIT_DATA = 3
    WAIT_CHECKSUM = 4

    def __init__(self, max_payload=UART_MAX_PAYLOAD_STM32):
        self.state = self.WAIT_SYNC
        self.cmd = 0
        self.length = 0
        self.buffer = bytearray()
        self.idx = 0
        self.xor_val = 0
        self.max_payload = max_payload
        self.packets = []  # successfully parsed packets
        self.errors = []   # checksum errors

    def feed(self, data):
        """
        Feed bytes into the parser. Returns list of parsed packets.
        Each packet is a dict: {"cmd": int, "payload": bytes, "checksum": int}
        """
        new_packets = []

        for byte in data:
            byte = byte & 0xFF

            if self.state == self.WAIT_SYNC:
                if byte == UART_SYNC_BYTE:
                    self.xor_val = 0
                    self.state = self.WAIT_CMD

            elif self.state == self.WAIT_CMD:
                self.cmd = byte
                self.xor_val ^= byte
                self.state = self.WAIT_LEN

            elif self.state == self.WAIT_LEN:
                self.length = byte
                self.xor_val ^= byte
                self.idx = 0
                self.buffer = bytearray()
                if self.length == 0:
                    self.state = self.WAIT_CHECKSUM
                elif self.length > self.max_payload:
                    # Invalid length, reset
                    self.state = self.WAIT_SYNC
                else:
                    self.state = self.WAIT_DATA

            elif self.state == self.WAIT_DATA:
                self.buffer.append(byte)
                self.xor_val ^= byte
                self.idx += 1
                if self.idx >= self.length:
                    self.state = self.WAIT_CHECKSUM

            elif self.state == self.WAIT_CHECKSUM:
                if byte == self.xor_val:
                    pkt = {
                        "cmd": self.cmd,
                        "payload": bytes(self.buffer),
                        "checksum": byte,
                    }
                    new_packets.append(pkt)
                    self.packets.append(pkt)
                else:
                    self.errors.append({
                        "cmd": self.cmd,
                        "expected": self.xor_val,
                        "got": byte,
                    })
                self.state = self.WAIT_SYNC

        return new_packets


# ---------------------------------------------------------------------------
# Higher-level packet builders (matching ESP32 comms.cpp)
# ---------------------------------------------------------------------------

def build_set_force_packet(force_n):
    """Build CMD_SET_FORCE packet: force in milliNewtons as uint16 big-endian."""
    mn = int(force_n * 1000.0) & 0xFFFF
    payload = struct.pack(">H", mn)
    return build_packet(CMD_SET_FORCE, payload)


def build_servo_move_packet(servo_id, position_us):
    """Build CMD_SERVO_MOVE packet: [servo_id, pos_hi, pos_lo]."""
    payload = bytes([servo_id, (position_us >> 8) & 0xFF, position_us & 0xFF])
    return build_packet(CMD_SERVO_MOVE, payload)


# ===========================================================================
# TESTS
# ===========================================================================


class TestPacketConstruction:
    """Test that packets are built correctly."""

    def test_empty_payload(self):
        """Command with no payload (e.g., GRIP_OPEN)."""
        pkt = build_packet(CMD_GRIP_OPEN)
        assert pkt[0] == UART_SYNC_BYTE
        assert pkt[1] == CMD_GRIP_OPEN
        assert pkt[2] == 0  # length
        # Checksum = CMD ^ LEN = 0x01 ^ 0x00 = 0x01
        assert pkt[3] == CMD_GRIP_OPEN ^ 0x00

    def test_short_payload(self):
        """Command with 2-byte payload (SET_FORCE)."""
        pkt = build_set_force_packet(5.0)  # 5000 mN = 0x1388
        assert pkt[0] == UART_SYNC_BYTE
        assert pkt[1] == CMD_SET_FORCE
        assert pkt[2] == 2  # length
        assert pkt[3] == 0x13  # 5000 >> 8
        assert pkt[4] == 0x88  # 5000 & 0xFF
        expected_checksum = CMD_SET_FORCE ^ 2 ^ 0x13 ^ 0x88
        assert pkt[5] == expected_checksum

    def test_three_byte_payload(self):
        """Command with 3-byte payload (SERVO_MOVE)."""
        pkt = build_servo_move_packet(servo_id=0, position_us=1500)
        assert pkt[0] == UART_SYNC_BYTE
        assert pkt[1] == CMD_SERVO_MOVE
        assert pkt[2] == 3
        assert pkt[3] == 0        # servo_id
        assert pkt[4] == (1500 >> 8) & 0xFF  # 0x05
        assert pkt[5] == 1500 & 0xFF          # 0xDC
        expected_checksum = CMD_SERVO_MOVE ^ 3 ^ 0 ^ pkt[4] ^ pkt[5]
        assert pkt[6] == expected_checksum

    def test_packet_total_length(self):
        """Packet length = 4 + payload_len (SYNC + CMD + LEN + PAYLOAD + CHECKSUM)."""
        for payload_len in [0, 1, 5, 50, 72]:
            payload = bytes(range(payload_len))
            pkt = build_packet(0x10, payload)
            assert len(pkt) == 4 + payload_len


class TestPacketRoundTrip:
    """Test build-then-parse round trips."""

    def test_empty_payload_roundtrip(self):
        """Build empty payload packet -> parse -> verify."""
        pkt = build_packet(CMD_GRIP_OPEN)
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 1
        assert results[0]["cmd"] == CMD_GRIP_OPEN
        assert results[0]["payload"] == b""

    def test_short_payload_roundtrip(self):
        """Build SET_FORCE packet -> parse -> verify payload."""
        pkt = build_set_force_packet(7.5)
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 1
        assert results[0]["cmd"] == CMD_SET_FORCE
        # Decode payload
        mn = struct.unpack(">H", results[0]["payload"])[0]
        assert mn == 7500

    def test_servo_move_roundtrip(self):
        """Build SERVO_MOVE packet -> parse -> verify payload."""
        pkt = build_servo_move_packet(servo_id=2, position_us=2000)
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 1
        assert results[0]["cmd"] == CMD_SERVO_MOVE
        payload = results[0]["payload"]
        assert payload[0] == 2  # servo_id
        pos = (payload[1] << 8) | payload[2]
        assert pos == 2000

    def test_estop_roundtrip(self):
        """E-STOP command (no payload)."""
        pkt = build_packet(CMD_ESTOP)
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 1
        assert results[0]["cmd"] == CMD_ESTOP
        assert results[0]["payload"] == b""

    def test_multiple_packets_stream(self):
        """Parse a stream of multiple consecutive packets."""
        pkt1 = build_packet(CMD_GRIP_OPEN)
        pkt2 = build_set_force_packet(3.0)
        pkt3 = build_packet(CMD_GRIP_CLOSE)

        stream = pkt1 + pkt2 + pkt3
        parser = UARTParser()
        results = parser.feed(stream)

        assert len(results) == 3
        assert results[0]["cmd"] == CMD_GRIP_OPEN
        assert results[1]["cmd"] == CMD_SET_FORCE
        assert results[2]["cmd"] == CMD_GRIP_CLOSE


class TestMalformedPackets:
    """Test handling of malformed / invalid packets."""

    def test_wrong_sync_byte(self):
        """Packet with wrong sync byte should be ignored."""
        bad_pkt = bytearray(build_packet(CMD_GRIP_OPEN))
        bad_pkt[0] = 0xBB  # wrong sync
        parser = UARTParser()
        results = parser.feed(bad_pkt)
        assert len(results) == 0

    def test_bad_checksum(self):
        """Packet with corrupted checksum should be rejected."""
        pkt = bytearray(build_packet(CMD_GRIP_OPEN))
        pkt[-1] ^= 0xFF  # flip all bits of checksum
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 0
        assert len(parser.errors) == 1

    def test_corrupted_payload(self):
        """Packet with corrupted payload byte should fail checksum."""
        pkt = bytearray(build_set_force_packet(5.0))
        pkt[3] ^= 0x01  # flip one bit in payload
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 0
        assert len(parser.errors) == 1

    def test_truncated_packet(self):
        """Truncated packet (missing checksum) should not parse."""
        pkt = build_packet(CMD_SET_FORCE, b"\x13\x88")
        truncated = pkt[:-1]  # remove checksum byte
        parser = UARTParser()
        results = parser.feed(truncated)
        assert len(results) == 0

    def test_garbage_before_valid_packet(self):
        """Parser should skip garbage bytes and still find a valid packet."""
        garbage = bytes([0x55, 0x12, 0x34, 0xFF, 0x00])
        valid_pkt = build_packet(CMD_GRIP_OPEN)
        parser = UARTParser()
        results = parser.feed(garbage + valid_pkt)
        assert len(results) == 1
        assert results[0]["cmd"] == CMD_GRIP_OPEN

    def test_garbage_between_packets(self):
        """Garbage between valid packets should be skipped."""
        pkt1 = build_packet(CMD_GRIP_OPEN)
        garbage = bytes([0x55, 0x66, 0x77])
        pkt2 = build_packet(CMD_GRIP_CLOSE)
        parser = UARTParser()
        results = parser.feed(pkt1 + garbage + pkt2)
        assert len(results) == 2

    def test_overlength_payload_rejected(self):
        """Payload longer than max_payload should cause parser reset."""
        # Build a packet with declared length > max_payload
        # The parser should reject it (go back to WAIT_SYNC)
        payload = bytes(UART_MAX_PAYLOAD_STM32 + 10)  # Over-length
        cmd = 0x10
        # Manually construct the header with the over-length
        bad_pkt = bytearray()
        bad_pkt.append(UART_SYNC_BYTE)
        bad_pkt.append(cmd)
        bad_pkt.append(UART_MAX_PAYLOAD_STM32 + 10)  # Will be truncated to 0xFF if > 255
        # Even if len byte fits in uint8 (up to 255), UART_MAX_PAYLOAD=200
        # So len=210 should be rejected
        if UART_MAX_PAYLOAD_STM32 + 10 <= 255:
            bad_pkt.extend(payload)
            bad_pkt.append(0x00)  # dummy checksum
            parser = UARTParser(max_payload=UART_MAX_PAYLOAD_STM32)
            results = parser.feed(bad_pkt)
            assert len(results) == 0

    def test_esp32_max_payload_enforced(self):
        """ESP32 parser should reject packets over its 72-byte limit."""
        payload = bytes(73)  # 1 over ESP32 limit
        bad_pkt = bytearray([UART_SYNC_BYTE, 0x10, 73])
        bad_pkt.extend(payload)
        bad_pkt.append(0x00)  # dummy checksum
        parser = UARTParser(max_payload=UART_MAX_PAYLOAD_ESP32)
        results = parser.feed(bad_pkt)
        assert len(results) == 0


class TestMaxLengthPacket:
    """Test maximum-length valid packets."""

    def test_max_payload_stm32(self):
        """Build and parse a packet at the STM32 max payload (200 bytes)."""
        payload = bytes(range(UART_MAX_PAYLOAD_STM32))  # 200 bytes, values 0-199
        pkt = build_packet(0x42, payload)
        parser = UARTParser(max_payload=UART_MAX_PAYLOAD_STM32)
        results = parser.feed(pkt)
        assert len(results) == 1
        assert results[0]["cmd"] == 0x42
        assert results[0]["payload"] == payload

    def test_max_payload_esp32(self):
        """Build and parse a packet at the ESP32 max payload (72 bytes)."""
        payload = bytes(range(UART_MAX_PAYLOAD_ESP32))
        pkt = build_packet(0x42, payload)
        parser = UARTParser(max_payload=UART_MAX_PAYLOAD_ESP32)
        results = parser.feed(pkt)
        assert len(results) == 1
        assert results[0]["payload"] == payload


class TestEmptyPayload:
    """Test various commands with empty payloads."""

    @pytest.mark.parametrize("cmd", [
        CMD_GRIP_OPEN, CMD_GRIP_CLOSE, CMD_TAP, CMD_ESTOP,
        CMD_REQUEST_DEPTH, CMD_REQUEST_IMU_TAP, CMD_REQUEST_MIC_TAP,
    ])
    def test_empty_payload_commands(self, cmd):
        """All simple commands with no payload should round-trip correctly."""
        pkt = build_packet(cmd)
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 1
        assert results[0]["cmd"] == cmd
        assert results[0]["payload"] == b""
        assert len(pkt) == 4  # SYNC + CMD + LEN(0) + CHECKSUM


class TestChecksumComputation:
    """Test the checksum algorithm directly."""

    def test_checksum_empty_payload(self):
        """Checksum with no data = CMD ^ 0."""
        assert compute_checksum(0x01, b"") == 0x01
        assert compute_checksum(0xFF, b"") == 0xFF

    def test_checksum_single_byte(self):
        """Checksum with 1-byte payload."""
        # CMD=0x03, LEN=1, DATA=0x42
        # 0x03 ^ 0x01 ^ 0x42 = 0x40
        assert compute_checksum(0x03, b"\x42") == 0x03 ^ 0x01 ^ 0x42

    def test_checksum_known_values(self):
        """Verify checksum against hand-computed values."""
        # CMD=0x05, DATA=[0x00, 0x05, 0xDC] (servo_id=0, pos=1500)
        # LEN=3
        # XOR = 0x05 ^ 0x03 ^ 0x00 ^ 0x05 ^ 0xDC
        expected = 0x05 ^ 0x03 ^ 0x00 ^ 0x05 ^ 0xDC
        assert compute_checksum(0x05, b"\x00\x05\xDC") == expected

    def test_checksum_all_zeros(self):
        """Checksum where all payload bytes are zero."""
        # CMD=0x10, DATA=[0x00, 0x00, 0x00, 0x00]
        # XOR = 0x10 ^ 0x04 ^ 0 ^ 0 ^ 0 ^ 0 = 0x14
        assert compute_checksum(0x10, b"\x00\x00\x00\x00") == 0x10 ^ 0x04

    def test_checksum_all_ff(self):
        """Checksum where all payload bytes are 0xFF."""
        # CMD=0x01, DATA=[0xFF, 0xFF]
        # XOR = 0x01 ^ 0x02 ^ 0xFF ^ 0xFF = 0x03
        assert compute_checksum(0x01, b"\xFF\xFF") == 0x01 ^ 0x02 ^ 0xFF ^ 0xFF


class TestResponseParsing:
    """Test parsing of response packets (STM32 -> ESP32)."""

    def test_ack_response(self):
        """Parse RSP_ACK response with acked command ID."""
        pkt = build_packet(RSP_ACK, bytes([CMD_GRIP_OPEN]))
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 1
        assert results[0]["cmd"] == RSP_ACK
        assert results[0]["payload"] == bytes([CMD_GRIP_OPEN])

    def test_error_response(self):
        """Parse RSP_ERROR response with error code."""
        pkt = build_packet(RSP_ERROR, bytes([ERR_CHECKSUM_FAIL]))
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 1
        assert results[0]["cmd"] == RSP_ERROR
        assert results[0]["payload"][0] == ERR_CHECKSUM_FAIL

    def test_status_response(self):
        """Parse RSP_STATUS 7-byte response."""
        # Payload: [status, estop, connected, force_hi, force_lo, limit_hi, limit_lo]
        payload = bytes([0x0F, 0x00, 0x01, 0x00, 0x1E, 0x00, 0x64])
        pkt = build_packet(RSP_STATUS, payload)
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 1
        assert results[0]["cmd"] == RSP_STATUS
        p = results[0]["payload"]
        assert p[0] == 0x0F        # status
        assert p[1] == 0x00        # estop = false
        assert p[2] == 0x01        # connected = true
        force_x10 = (p[3] << 8) | p[4]  # 30 = 3.0N
        assert force_x10 == 30
        limit_x10 = (p[5] << 8) | p[6]  # 100 = 10.0N
        assert limit_x10 == 100

    def test_imu_tap_response(self):
        """Parse RSP_IMU_TAP_DATA response."""
        # peak_accel=9.8 -> int16 980, vib_rms=0.5 -> uint16 500, freq=1000 -> uint16 1000
        peak = int(9.8 * 100)   # 980
        vib = int(0.5 * 1000)   # 500
        freq = 1000
        payload = struct.pack(">hHH", peak, vib, freq)
        pkt = build_packet(RSP_IMU_TAP_DATA, payload)
        parser = UARTParser()
        results = parser.feed(pkt)
        assert len(results) == 1
        p = results[0]["payload"]
        parsed_peak = struct.unpack(">h", p[0:2])[0] / 100.0
        parsed_vib = struct.unpack(">H", p[2:4])[0] / 1000.0
        parsed_freq = struct.unpack(">H", p[4:6])[0]
        assert parsed_peak == pytest.approx(9.8, abs=0.01)
        assert parsed_vib == pytest.approx(0.5, abs=0.001)
        assert parsed_freq == 1000
