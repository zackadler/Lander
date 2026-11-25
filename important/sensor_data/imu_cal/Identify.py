"""
Step 0: find out what the IMU actually provides.

Phase A: hold the board still -> baseline for gyro and magnetometer.
Phase B: rotate it slowly in all directions -> does the gyro respond,
         does the magnetometer vector change direction at ~constant magnitude?
"""

import math
import statistics
import time

from IMU_raw_function import Icm20948Raw

RATE_HZ = 100.0


def collect(imu, seconds):
    period = 1.0 / RATE_HZ
    out = []
    nxt = time.perf_counter()
    end = nxt + seconds
    while time.perf_counter() < end:
        out.append(imu.read())
        nxt += period
        d = nxt - time.perf_counter()
        if d > 0:
            time.sleep(d)
    return out


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def gyro_dps(s):
    return tuple(math.degrees(s[k]) for k in ("gyro_x", "gyro_y", "gyro_z"))


def mag_vec(s):
    return (s["mag_x"], s["mag_y"], s["mag_z"])


def main():
    imu = Icm20948Raw()
    print(f"WHO_AM_I = 0x{imu.who_am_i:02X} (ICM-20948 confirmed)")
    print(f"Magnetometer (AK09916) found on bypass bus: {imu.has_mag}")
    if not imu.has_mag:
        print("  -> no magnetometer reachable; continuing with accel+gyro.")

    input("\nPhase A: put the board flat and still, then press Enter...")
    still = collect(imu, 4.0)
    g_still = [norm(gyro_dps(s)) for s in still]
    gx = [math.degrees(s["gyro_x"]) for s in still]
    gy = [math.degrees(s["gyro_y"]) for s in still]
    gz = [math.degrees(s["gyro_z"]) for s in still]
    print(f"  gyro mean (deg/s): x={statistics.mean(gx):+.3f} y={statistics.mean(gy):+.3f} "
          f"z={statistics.mean(gz):+.3f}")
    print(f"  gyro std  (deg/s): x={statistics.pstdev(gx):.3f} y={statistics.pstdev(gy):.3f} "
          f"z={statistics.pstdev(gz):.3f}")
    still_noise = max(statistics.pstdev(gx), statistics.pstdev(gy), statistics.pstdev(gz))
    mags_still = [norm(mag_vec(s)) for s in still if s["mag_new"]]
    if mags_still:
        print(f"  mag |B| still: mean={statistics.mean(mags_still):.1f} uT "
              f"std={statistics.pstdev(mags_still):.2f} uT")

    input("\nPhase B: press Enter, then slowly rotate the board in all directions for 10 s...")
    t0 = time.time()
    moving = collect(imu, 10.0)
    print(f"  collected {len(moving)} samples in {time.time() - t0:.1f} s")

    peak = max(norm(gyro_dps(s)) for s in moving)
    print(f"\nGyro: peak rate while rotating = {peak:.1f} deg/s "
          f"(still noise ~{still_noise:.3f} deg/s)")
    gyro_ok = peak > max(20.0, 50 * still_noise)
    print(f"  -> gyro responds to rotation: {'YES' if gyro_ok else 'NO / inconclusive'}")

    mv = [mag_vec(s) for s in moving if s["mag_new"]]
    if imu.has_mag and len(mv) > 10:
        mags = [norm(v) for v in mv]
        ref = mv[0]
        max_angle = max(
            math.degrees(math.acos(max(-1.0, min(1.0,
                sum(a * b for a, b in zip(ref, v)) / (norm(ref) * norm(v))))))
            for v in mv if norm(v) > 1e-6
        )
        print(f"\nMag: |B| min={min(mags):.1f} max={max(mags):.1f} uT "
              f"(Earth field is typically ~25-65 uT)")
        print(f"     max direction change vs first sample = {max_angle:.0f} deg")
        mag_ok = max_angle > 45 and 10 < statistics.mean(mags) < 100
        print(f"  -> magnetometer responds like a compass: {'YES' if mag_ok else 'NO / inconclusive'}")
        print("     (indoors, large |B| swings mean nearby metal/motors are distorting it)")
    else:
        print("\nMag: no usable samples.")


if __name__ == "__main__":
    main()