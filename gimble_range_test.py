"""
Gimbal range-of-motion test

Finds how far each gimbal axis can deflect before the thruster/propeller
touches (or gets too close to) the structure, then checks the diagonal
corners where roll and pitch are deflected together.

Hobby servos have no position feedback, so YOU are the collision sensor:
the script steps the gimbal out slowly and asks you after each step
whether there is still clearance.


SAFETY: disconnect the propeller or unplug the ESC/motor battery before
running this. Only the servos should be powered.

Controls at each prompt:
  Enter  = still clear, keep going
  x      = touching / too close (the previous step is taken as the limit)
  q      = abort, return to center and exit
"""

import json
import math
import time
from time import sleep
from adafruit_servokit import ServoKit

# ---------------- Hardware ----------------
servo_roll = 5
servo_pitch = 4

kit = ServoKit(channels=16, frequency=100)

CENTER = 15          # servo angle at neutral (same as your flight script)

# ---------------- Test settings ----------------
SEARCH_LIMIT = 30.0  # furthest offset (deg) to try in each direction
STEP = 1.0           # deg per step while searching
SAFETY_MARGIN = 1.0  # desubtracted from the last clear position
DIAG_STEPS = 10      # number of steps along each diagonal (0 -> 100%)
MOVE_STEP = 0.5      # deg per micro-step when moving (keeps motion slow)
MOVE_DELAY = 0.03    # s between micro-steps
SETTLE = 0.3         # s to wait after a move before asking

RESULTS_FILE = "gimbal_limits.json"

pos = {servo_roll: 0.0, servo_pitch: 0.0}  # current commanded offsets


class Abort(Exception):
    pass


def servo_name(channel):
    return {servo_roll: "roll", servo_pitch: "pitch"}.get(channel, f"ch{channel}")


def set_offset(channel, offset):
    """Command a servo to CENTER + offset, clamped to the servo's valid range."""
    angle = CENTER + offset
    angle = max(0.0, min(kit.servo[channel].actuation_range, angle))
    kit.servo[channel].angle = angle
    pos[channel] = angle - CENTER


def log_angles():
    r = kit.servo[servo_roll].angle
    p = kit.servo[servo_pitch].angle
    print(f"[{time.strftime('%H:%M:%S')}] roll offset {pos[servo_roll]:+.1f}° (angle {r:.1f}°) | "
          f"pitch offset {pos[servo_pitch]:+.1f}° (angle {p:.1f}°)", flush=True)


def move_to(roll, pitch):
    """Move both axes together in small increments so nothing slams."""
    r0, p0 = pos[servo_roll], pos[servo_pitch]
    dist = max(abs(roll - r0), abs(pitch - p0))
    n = max(1, math.ceil(dist / MOVE_STEP))
    for i in range(1, n + 1):
        t = i / n
        set_offset(servo_roll, r0 + (roll - r0) * t)
        set_offset(servo_pitch, p0 + (pitch - p0) * t)
        sleep(MOVE_DELAY)
    sleep(SETTLE)
    log_angles()


def ask():
    cmd = input("   Clear? [Enter]=yes  x=contact/too close  q=quit > ").strip().lower()
    if cmd == "q":
        raise Abort
    return cmd


def find_axis_limit(channel, direction):
    """Step one axis outward until the user reports contact. Returns (safe_limit, hit)."""
    label = f"{servo_name(channel)} {'+' if direction > 0 else '-'}"
    print(f"\n=== Testing {label} direction ===")
    move_to(0, 0)

    last_clear = 0.0
    while True:
        target = last_clear + direction * STEP
        if abs(target) > SEARCH_LIMIT + 1e-9:
            print(f"   Reached search limit ({SEARCH_LIMIT}°) with no contact.")
            move_to(0, 0)
            return direction * SEARCH_LIMIT, False

        if channel == servo_roll:
            move_to(target, 0)
        else:
            move_to(0, target)

        # Detect when the servo clamp stopped us short of the target
        if abs(pos[channel] - target) > 0.05:
            print(f"   Servo hit its angle limit at offset {pos[channel]:+.1f}°.")
            move_to(0, 0)
            return pos[channel], False

        if ask() == "x":
            break
        last_clear = target

    # Pull the margin back toward center, but never past center
    safe = last_clear - direction * SAFETY_MARGIN
    if safe * direction < 0:
        safe = 0.0
    print(f"   Last clear: {last_clear:+.1f}°  ->  safe limit with margin: {safe:+.1f}°")
    move_to(0, 0)
    return safe, True


def test_diagonal(roll_target, pitch_target):
    """Move toward a corner (both axes at their limits) and find how far is clear."""
    print(f"\n=== Testing corner roll {roll_target:+.1f}°, pitch {pitch_target:+.1f}° ===")
    move_to(0, 0)
    last_clear = 0.0
    for i in range(1, DIAG_STEPS + 1):
        s = i / DIAG_STEPS
        move_to(roll_target * s, pitch_target * s)
        if ask() == "x":
            move_to(0, 0)
            print(f"   Corner only clear to {last_clear * 100:.0f}% of the single-axis limits.")
            return last_clear
        last_clear = s
    move_to(0, 0)
    print("   Corner fully clear.")
    return 1.0


def main():
    print(__doc__)
    input("Propeller disconnected / motor unpowered? Press Enter to start...")

    move_to(0, 0)
    results = {"center": CENTER, "step": STEP, "safety_margin": SAFETY_MARGIN, "axes": {}, "corners": {}}

    try:
        # --- Single-axis limits ---
        for ch in (servo_roll, servo_pitch):
            pos_lim, pos_hit = find_axis_limit(ch, +1)
            neg_lim, neg_hit = find_axis_limit(ch, -1)
            results["axes"][servo_name(ch)] = {
                "min_offset": neg_lim, "max_offset": pos_lim,
                "min_angle": CENTER + neg_lim, "max_angle": CENTER + pos_lim,
                "contact_found_min": neg_hit, "contact_found_max": pos_hit,
            }

        # --- Combined corners ---
        r = results["axes"]["roll"]
        p = results["axes"]["pitch"]
        for r_off in (r["max_offset"], r["min_offset"]):
            for p_off in (p["max_offset"], p["min_offset"]):
                if r_off == 0 or p_off == 0:
                    continue
                key = f"roll{r_off:+.1f}_pitch{p_off:+.1f}"
                results["corners"][key] = test_diagonal(r_off, p_off)

    except Abort:
        print("\nAborted by user.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Returning to center...")
        move_to(0, 0)

    # --- Summary ---
    print("\n================ RESULTS ================")
    for axis, a in results["axes"].items():
        note = "" if (a["contact_found_min"] and a["contact_found_max"]) else "  (no contact on at least one side)"
        print(f"{axis:>5}: offset {a['min_offset']:+.1f}° to {a['max_offset']:+.1f}°  "
              f"(servo angle {a['min_angle']:.1f}° to {a['max_angle']:.1f}°){note}")
    for key, s in results["corners"].items():
        print(f"corner {key}: clear to {s * 100:.0f}%")

    if results["corners"] and min(results["corners"].values()) < 1.0:
        print("\nWARNING: at least one corner collides before both axes reach their limits.")
        print("Combined roll+pitch commands need to be scaled down, not just clamped per axis.")

    if results["axes"]:
        results["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(RESULTS_FILE, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nSaved to {RESULTS_FILE}")


if __name__ == "__main__":
    main()