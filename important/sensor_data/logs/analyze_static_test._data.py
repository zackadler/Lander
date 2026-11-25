"""
Quick summary of a log from log_raw_data.py using ONLY the Python standard
library (no pandas / numpy needed).

Usage:  python3 analyze_log.py logs/20261124_202801_static_flat.csv
Keep this script outside the logs/ folder.
"""

import csv
import math
import statistics
import sys
from collections import Counter

G = 9.80665


def col(rows, name):
    out = []
    for r in rows:
        v = r.get(name, "")
        out.append(float(v) if v not in ("", None) else float("nan"))
    return out


def clean(x):
    return [v for v in x if not math.isnan(v)]


def mean_std(x):
    x = clean(x)
    return statistics.fmean(x), statistics.pstdev(x)


def adev(x, fs, taus):
    out = []
    for tau in taus:
        m = int(tau * fs)
        n = len(x) // m
        if m < 1 or n < 3:
            continue
        y = [statistics.fmean(x[i * m:(i + 1) * m]) for i in range(n)]
        d2 = [(y[i + 1] - y[i]) ** 2 for i in range(n - 1)]
        out.append((tau, math.sqrt(0.5 * statistics.fmean(d2)), n))
    return out


def main(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    n = len(rows)
    t = col(rows, "t_ns")
    dt_ms = [(t[i + 1] - t[i]) / 1e6 for i in range(n - 1)]
    fs = 1e9 / statistics.median([d * 1e6 for d in dt_ms])
    print(f"{path}\n{n} samples, {t[-1] / 1e9:.1f} s, median rate {fs:.2f} Hz\n")

    print("--- IMU, FRD frame (mean +/- std) ---")
    for c in ["frd_accel_x", "frd_accel_y", "frd_accel_z"]:
        m, s = mean_std(col(rows, c))
        print(f"{c:14s} {m:+9.4f} +/- {s:.4f} m/s^2")
    for c in ["frd_gyro_x", "frd_gyro_y", "frd_gyro_z"]:
        m, s = mean_std(col(rows, c))
        print(f"{c:14s} {math.degrees(m):+9.4f} +/- {math.degrees(s):.4f} deg/s")

    ax, ay, az = (statistics.fmean(clean(col(rows, f"imu_accel_{k}"))) for k in "xyz")
    mag_a = math.sqrt(ax * ax + ay * ay + az * az)
    print(f"\n|accel| mean = {mag_a:.4f} m/s^2 (g = {G:.4f}; "
          f"{(mag_a / G - 1) * 100:+.2f}% off)")
    print(f"tilt if no bias (native): {math.degrees(math.atan2(ax, az)):+.2f} deg about y, "
          f"{math.degrees(math.atan2(ay, az)):+.2f} deg about x")

    k = 6000 if n > 12000 else n // 4
    print(f"\n--- stability: first {k} vs last {k} samples ---")
    for c in ["frd_accel_x", "frd_accel_y", "frd_accel_z",
              "frd_gyro_x", "frd_gyro_y", "frd_gyro_z"]:
        x = col(rows, c)
        a, b = statistics.fmean(x[:k]), statistics.fmean(x[-k:])
        unit = 1.0 if "accel" in c else 180 / math.pi
        print(f"{c:14s} {a * unit:+9.4f} -> {b * unit:+9.4f}")
    tc = clean(col(rows, "imu_temp_c"))
    print(f"temp: {tc[0]:.2f} -> {tc[-1]:.2f} C (min {min(tc):.2f}, max {max(tc):.2f})")

    mx, my, mz = (statistics.fmean(clean(col(rows, f"imu_mag_{k}"))) for k in "xyz")
    print(f"\n--- magnetometer (native axes) ---\nmean = ({mx:.1f}, {my:.1f}, {mz:.1f}) uT, "
          f"|B| = {math.sqrt(mx*mx+my*my+mz*mz):.1f} uT, horizontal = {math.hypot(mx, my):.1f} uT")

    print("\n--- laser / flow ---")
    for c in ["flow_distance_m", "flow_rangefinder_quality", "flow_flow_quality",
              "flow_motion_x", "flow_motion_y"]:
        x = clean(col(rows, c))
        if x:
            print(f"{c:26s} mean {statistics.fmean(x):9.3f} std {statistics.pstdev(x):8.3f} "
                  f"min {min(x):g} max {max(x):g}  distinct={len(set(x))}")
    changed = [i for i, r in enumerate(rows) if r.get("flow_new") == "1"]
    gaps = Counter(changed[i + 1] - changed[i] for i in range(len(changed) - 1))
    print(f"flow changed in {len(changed) / n * 100:.1f}% of cycles; "
          f"most common gaps (cycles): {gaps.most_common(6)}")

    print("\n--- Allan deviation ---")
    taus = [0.1, 0.3, 1, 3, 10, 30, 100]
    for c in ["frd_gyro_x", "frd_gyro_y", "frd_gyro_z"]:
        x = [math.degrees(v) for v in col(rows, c)]
        print(c, " ".join(f"{tau:g}s:{a:.4f}(n={nn})" for tau, a, nn in adev(x, fs, taus)), "deg/s")
    for c in ["frd_accel_x", "frd_accel_y", "frd_accel_z"]:
        x = [v / G * 1000 for v in col(rows, c)]
        print(c, " ".join(f"{tau:g}s:{a:.3f}(n={nn})" for tau, a, nn in adev(x, fs, taus)), "mg")
    print("(few clusters at long tau => wide error bars; need hours of data for bias instability)")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python3 analyze_log.py <logfile.csv>")
    main(sys.argv[1])