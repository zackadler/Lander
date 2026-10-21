"""
Read and decode MicoAir MTF-02P sensor data over UART using the
MSPv2 sensor-message protocol -- specifically MSP2_SENSOR_RANGEFINDER
(function 0x1F01) and MSP2_SENSOR_OPTIC_FLOW (function 0x1F02), which
is what your sensor is currently outputting.

This reads it directly, as-is -- no need to touch the protocol jumper.
If you switch the sensor to Micolink later, use mtf02p_reader.py or
micolink.py instead; this script is for MSP specifically.

Wiring is the same as before: 5V, GND, sensor TX -> Pi RX (pin 10),
sensor RX -> Pi TX (pin 8).
"""

import struct

import serial

SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

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


def main():
    parser = MspV2Parser()
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as exc:
        print(f"Couldn't open {SERIAL_PORT}: {exc}")
        print("Check that the UART is enabled and your user is in 'dialout'.")
        return

    with ser:
        print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud. Ctrl+C to stop.")
        while True:
            byte = ser.read(1)
            if not byte:
                continue
            result = parser.feed(byte[0])
            if result is None:
                continue
            reading = decode_sensor_message(*result)
            if reading is None:
                continue

            if reading["type"] == "rangefinder":
                dist = (
                    f"{reading['distance_m']:.2f} m"
                    if reading["distance_m"] is not None
                    else "out of range"
                )
                print(f"[rangefinder] distance={dist:>10}  quality={reading['quality']:3d}/255")
            elif reading["type"] == "optic_flow":
                print(
                    f"[optic flow]  motion=({reading['motion_x']:+5d}, "
                    f"{reading['motion_y']:+5d})  quality={reading['quality']:3d}/255"
                )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")