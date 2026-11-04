"""
MTF-02P optical flow + rangefinder sensor, refactored from
mtf-02p_test.py into a class you construct once (which opens the
serial port) and then call .read() on whenever you want the most
recent reading -- no internal loop or blocking wait, so the caller
controls the timing.

Because the sensor streams data on its own schedule (about 50 Hz)
rather than answering on demand, .read() doesn't wait for a fresh
message: it drains whatever has arrived on the wire since the last
call and hands back the newest rangefinder and optic-flow values seen
so far. A field stays None until at least one message of that type
has arrived.
"""

import struct

import serial

MSP2_SENSOR_RANGEFINDER = 0x1F01
MSP2_SENSOR_OPTIC_FLOW = 0x1F02


def crc8_dvb_s2(crc, byte):
    """MSPv2's checksum algorithm, applied one byte at a time."""
    crc ^= byte
    for _ in range(8):
        if crc & 0x80:
            crc = ((crc << 1) ^ 0xD5) & 0xFF
        else:
            crc = (crc << 1) & 0xFF
    return crc


class MspV2Parser:
    """Byte-at-a-time state machine for MSPv2 frames:
    $ X <direction> <flag> <function:u16 LE> <size:u16 LE>
    <payload:size bytes> <crc8>

    Feed it one byte at a time via .feed(). Returns (function, payload)
    once a complete, checksum-valid frame has arrived, else None.
    """

    (WAIT_DOLLAR, WAIT_X, WAIT_DIR, WAIT_FLAG, FUNC_LO, FUNC_HI,
     SIZE_LO, SIZE_HI, PAYLOAD, CRC) = range(10)

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = self.WAIT_DOLLAR
        self.flag = 0
        self.function = 0
        self.size = 0
        self.payload = bytearray()
        self.crc = 0

    def feed(self, byte):
        if self.state == self.WAIT_DOLLAR:
            if byte == 0x24:  # '$'
                self.state = self.WAIT_X

        elif self.state == self.WAIT_X:
            if byte == 0x58:  # 'X' -- MSPv2 marker
                self.state = self.WAIT_DIR
            elif byte == 0x24:  # another '$', restart from here
                self.state = self.WAIT_X
            else:
                self.state = self.WAIT_DOLLAR

        elif self.state == self.WAIT_DIR:  # '<', '>', or '!'
            self.flag = 0
            self.function = 0
            self.size = 0
            self.payload = bytearray()
            self.crc = 0
            self.state = self.WAIT_FLAG

        elif self.state == self.WAIT_FLAG:
            self.flag = byte
            self.crc = crc8_dvb_s2(self.crc, byte)
            self.state = self.FUNC_LO

        elif self.state == self.FUNC_LO:
            self.function = byte
            self.crc = crc8_dvb_s2(self.crc, byte)
            self.state = self.FUNC_HI

        elif self.state == self.FUNC_HI:
            self.function |= byte << 8
            self.crc = crc8_dvb_s2(self.crc, byte)
            self.state = self.SIZE_LO

        elif self.state == self.SIZE_LO:
            self.size = byte
            self.crc = crc8_dvb_s2(self.crc, byte)
            self.state = self.SIZE_HI

        elif self.state == self.SIZE_HI:
            self.size |= byte << 8
            self.crc = crc8_dvb_s2(self.crc, byte)
            self.state = self.PAYLOAD if self.size else self.CRC

        elif self.state == self.PAYLOAD:
            self.payload.append(byte)
            self.crc = crc8_dvb_s2(self.crc, byte)
            if len(self.payload) == self.size:
                self.state = self.CRC

        elif self.state == self.CRC:
            valid = byte == self.crc
            function, payload = self.function, bytes(self.payload)
            self.reset()
            if valid:
                return function, payload

        return None


def decode_sensor_message(function, payload):
    """Turn a raw (function, payload) pair into a readable dict, for
    the two message types this sensor sends."""
    if function == MSP2_SENSOR_RANGEFINDER and len(payload) >= 5:
        quality = payload[0]
        distance_mm = struct.unpack("<i", payload[1:5])[0]
        return {
            "type": "rangefinder",
            "quality": quality,
            "distance_m": distance_mm / 1000.0 if distance_mm >= 0 else None,
        }

    if function == MSP2_SENSOR_OPTIC_FLOW and len(payload) >= 9:
        quality = payload[0]
        motion_x, motion_y = struct.unpack("<ii", payload[1:9])
        return {
            "type": "optic_flow",
            "quality": quality,
            "motion_x": motion_x,
            "motion_y": motion_y,
        }

    return None


class Mtf02pSensor:
    """Construct once (opens the serial port), then call .read() as
    often as you like."""

    def __init__(self, port="/dev/ttyAMA0", baud=115200):
        self._ser = serial.Serial(port, baud, timeout=0)  # non-blocking
        self._parser = MspV2Parser()
        self._distance_m = None
        self._rangefinder_quality = None
        self._motion_x = None
        self._motion_y = None
        self._flow_quality = None

    def _pump(self):
        """Feed every byte currently sitting in the OS serial buffer
        through the parser, updating the cached latest readings."""
        waiting = self._ser.in_waiting
        if not waiting:
            return
        for byte in self._ser.read(waiting):
            result = self._parser.feed(byte)
            if result is None:
                continue
            reading = decode_sensor_message(*result)
            if reading is None:
                continue
            if reading["type"] == "rangefinder":
                self._distance_m = reading["distance_m"]
                self._rangefinder_quality = reading["quality"]
            elif reading["type"] == "optic_flow":
                self._motion_x = reading["motion_x"]
                self._motion_y = reading["motion_y"]
                self._flow_quality = reading["quality"]

    def read(self):
        """The most recent rangefinder and optic-flow reading seen so
        far. A field stays None until a message of that type has
        arrived at least once."""
        self._pump()
        return {
            "distance_m": self._distance_m,
            "rangefinder_quality": self._rangefinder_quality,
            "motion_x": self._motion_x,
            "motion_y": self._motion_y,
            "flow_quality": self._flow_quality,
        }