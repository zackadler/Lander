#!/usr/bin/env python3
"""
ESC + motor bench test through a PCA9685 PWM board on a Raspberry Pi.

  python3 esc_test.py              arm the ESC, then raise the throttle step by step
  python3 esc_test.py calibrate    teach the ESC this board's 1000/2000 us (PROPELLER OFF)

  Options: --channel N   --osc HZ   --min US   --cap US

If the ESC beeps fast and won't arm, type a lower zero (e.g. 950) at the
arming prompt; the script tells you which --min to use next time.

Run with the propeller off, or with the motor clamped down and nobody in line
with the prop disc, until it starts and stops exactly when you expect.
Ctrl-C at any point stops the motor and cuts the signal.
"""

import argparse
import time

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

# ---- settings ---------------------------------------------------------------
I2C_BUS = 1
I2C_ADDR = 0x40
CHANNEL = 6              # 0-15 as printed on the board (counting starts at 0)

PWM_HZ = 50
# The PCA9685 clock is nominally 25 MHz but varies from chip to chip (about
# 23-27 MHz), so every pulse can be several percent longer or shorter than
# asked for. To correct it, measure the output frequency on the channel
# (multimeter Hz mode, scope or logic analyser) and set
#     OSC_HZ = measured_hz * 4096 * (prescale + 1)
# using the prescale this script prints when it starts (or pass it as --osc).
# Calibrating the ESC ("calibrate" mode) also takes care of it, because the
# ESC then learns whatever this board really sends.
OSC_HZ = 25_000_000

MIN_US = 1000            # zero throttle
MAX_US = 2000            # full throttle, only sent by "calibrate"
CAP_US = 1400            # the test never goes above this unless --cap says so
STEP_US = 10             # Enter = +10 us
LOWEST_US = 800          # lowest zero-throttle pulse you can type while arming

# ---- PCA9685 ----------------------------------------------------------------
MODE1, MODE2, PRE_SCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, AUTO_INC, SLEEP, ALLCALL = 0x80, 0x20, 0x10, 0x01
OUTDRV = 0x04            # totem-pole outputs (the power-up default, set anyway)
FULL_OFF = 0x10          # LEDn_OFF_H bit 4: output held low, no pulses at all


class PCA9685:
    def __init__(self, bus, addr, osc_hz):
        self.bus, self.addr, self.osc_hz = bus, addr, osc_hz
        self.prescale = None

    def _write(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value)

    def _read(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def start(self, pwm_hz, channel, pulse_us):
        """Set the PWM rate, with a known pulse on `channel` before the outputs run."""
        prescale = max(3, min(255, round(self.osc_hz / (4096 * pwm_hz)) - 1))
        self._write(MODE1, SLEEP | AUTO_INC | ALLCALL)   # prescale only changes while asleep
        self._write(PRE_SCALE, prescale)
        if self._read(PRE_SCALE) != prescale:
            raise RuntimeError(f"the device at {self.addr:#04x} did not keep the prescale "
                               "value - is it really the PCA9685?")
        self.prescale = prescale
        self._write(MODE2, OUTDRV)
        self.set_us(channel, pulse_us)        # replaces whatever a previous run left there
        self._write(MODE1, AUTO_INC | ALLCALL)           # wake the oscillator
        time.sleep(0.005)                                # it needs 500 us to settle
        self._write(MODE1, RESTART | AUTO_INC | ALLCALL)
        if self._read(MODE1) & SLEEP:
            raise RuntimeError("the PCA9685 is still asleep after wake-up")

    @property
    def us_per_count(self):
        return 1e6 * (self.prescale + 1) / self.osc_hz

    def counts(self, pulse_us):
        return max(1, min(4095, round(pulse_us / self.us_per_count)))

    def set_us(self, channel, pulse_us):
        off = self.counts(pulse_us)
        # One 4-byte write with auto-increment: the output switches to the new
        # value at the end of the transfer and is never half-updated.
        self.bus.write_i2c_block_data(self.addr, LED0_ON_L + 4 * channel,
                                      [0, 0, off & 0xFF, off >> 8])

    def no_pulses(self, channel):
        self.bus.write_i2c_block_data(self.addr, LED0_ON_L + 4 * channel,
                                      [0, 0, 0, FULL_OFF])


def ramp(pca, channel, start_us, end_us, step_us=5, frame_s=0.02):
    """Raise the throttle gradually instead of jumping."""
    us = start_us
    while us < end_us:
        us = min(end_us, us + step_us)
        pca.set_us(channel, us)
        time.sleep(frame_s)


def arm(pca, args):
    """Hold zero throttle until the ESC arms. The zero can be lowered on the
    spot, for ESCs that read this board's 1000 us as a little throttle."""
    start_us = args.min
    print(f"\nZero throttle ({start_us} us) is on channel {args.channel}.")
    print("Connect the ESC battery now and listen:")
    print("  start-up tones, a confirmation beep or two, then quiet = ARMED: press Enter")
    print("  fast beeping that doesn't stop = it doesn't accept this as zero:")
    print(f"      type a lower value such as {start_us - 50}, press Enter and listen again")
    print("  slow beeping, about every 2 s = no valid signal (wiring, or the value is too low)")
    while True:
        cmd = input(f"[zero {args.min} us] Enter = armed, or a lower value > ").strip()
        if cmd == "":
            break
        try:
            value = int(float(cmd))
        except ValueError:
            print("  ?")
            continue
        if not LOWEST_US <= value <= start_us:
            print(f"  use a value from {LOWEST_US} to {start_us} us")
            continue
        args.min = value                 # 's' and the shutdown use the new zero from now on
        pca.set_us(args.channel, value)
    if args.min != start_us:
        print(f"  Armed at {args.min} us. Next time start with: python3 esc_test.py --min {args.min}")


def run_test(pca, args):
    arm(pca, args)
    channel, min_us, cap_us = args.channel, args.min, args.cap
    print(f"\n  Enter = +{STEP_US} us   - = -{STEP_US} us   1200 = go to 1200 us"
          f"   s = stop   q = quit   (cap {cap_us} us)")
    print("  Note the value where the motor starts turning.")
    current = min_us
    while True:
        try:
            cmd = input(f"[{current} us] > ").strip().lower()
        except EOFError:
            cmd = "q"
        if cmd == "q":
            return
        if cmd in ("", "+"):
            target = current + STEP_US
        elif cmd == "-":
            target = current - STEP_US
        elif cmd in ("s", "0"):
            target = min_us
        else:
            try:
                target = int(float(cmd))
            except ValueError:
                print("  ?")
                continue
        if target > cap_us:
            print(f"  limited to {cap_us} us (raise it with --cap once the motor is clamped down)")
            target = cap_us
        target = max(min_us, target)
        if target > current:
            ramp(pca, channel, current, target)
        else:
            pca.set_us(channel, target)
        current = target


def run_calibrate(pca, channel, min_us):
    pca.set_us(channel, MAX_US)
    print(f"\nFull throttle ({MAX_US} us) is on channel {channel}.")
    print("Connect the ESC battery now. When it beeps to say the top of the range")
    print("is stored (often two short beeps), press Enter straight away.")
    print("If it starts playing a long series of tunes instead, it has gone into")
    print("programming mode: unplug the battery and run this again.")
    input("> ")
    pca.set_us(channel, min_us)
    print(f"Zero throttle ({min_us} us) sent. The ESC should beep to confirm")
    print("(often one beep per battery cell, then a long beep).")
    input("Press Enter when it has finished... ")
    print("If you heard those confirmation beeps, the ESC now uses this board's")
    print("real pulse widths as its throttle range.")


def shutdown(pca, channel, min_us):
    """Zero throttle first, then no pulses at all so the ESC disarms."""
    try:
        pca.set_us(channel, min_us)
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        pca.no_pulses(channel)
    except OSError as err:
        print(f"\n!!! Could not stop the signal ({err}).")
        print("!!! The board may still be sending the last throttle: UNPLUG THE ESC BATTERY NOW.")
        return
    print("Signal off, ESC disarmed. It may beep every so often until you unplug the battery.")
    print("Unplug the battery before touching the propeller.")


def main():
    p = argparse.ArgumentParser(description="ESC bench test through a PCA9685")
    p.add_argument("mode", nargs="?", choices=["test", "calibrate"], default="test")
    p.add_argument("--channel", type=int, default=CHANNEL, choices=range(16), metavar="0-15")
    p.add_argument("--osc", type=float, default=OSC_HZ,
                   help="PCA9685 clock in Hz (see the note at the top of the file)")
    p.add_argument("--min", type=int, default=MIN_US, help="zero-throttle pulse in us")
    p.add_argument("--cap", type=int, default=CAP_US, help="highest pulse the test sends, in us")
    args = p.parse_args()

    if args.mode == "calibrate":
        print("Calibration sends FULL THROTTLE. Propeller OFF, ESC battery DISCONNECTED.")
        if input("Type yes to continue: ").strip().lower() != "yes":
            return
    else:
        print("ESC battery disconnected, motor secured, propeller clear?")
        input("Press Enter to start... ")

    bus = SMBus(I2C_BUS)
    pca = PCA9685(bus, I2C_ADDR, args.osc)
    try:
        pca.start(PWM_HZ, args.channel, args.min)
        print(f"PCA9685 ok: prescale {pca.prescale}, {pca.us_per_count:.2f} us per step, "
              f"{args.min} us = {pca.counts(args.min)} steps")
        if args.mode == "calibrate":
            run_calibrate(pca, args.channel, args.min)
        else:
            run_test(pca, args)
    except (KeyboardInterrupt, EOFError):
        print("\nStopped.")
    except (RuntimeError, OSError) as err:
        print(f"\nError: {err}")
    finally:
        # The PCA9685 keeps sending its last pulse after this script exits, so
        # always bring the throttle down and cut the signal on the way out.
        if pca.prescale is not None:
            shutdown(pca, args.channel, args.min)
        bus.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()