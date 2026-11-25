"""
Raw data logger: ICM-20948 (accel, gyro, mag, temp) + MTF-02P (flow + laser).

Needs, in the same folder:  IMU_raw_function.py  and  MTF_02P_function.py

Frames
------
Body frame is FRD: x Forward, y Right, z Down (right-handed).
Every IMU row contains BOTH:
  * imu_*      : native chip axes, exactly as the sensor reports them
                 (never discard these; they let you fix a wrong mapping later)
  * frd_*      : accel and gyro mapped into FRD with NATIVE_TO_FRD
                 (defined in IMU_raw_function.py)
The mapping is an ASSUMPTION (native x=right, y=forward, z=up) until you
verify it with the six-position and rotation tests. The matrix used is
saved in each run's .json file.

Not converted (yet):
  * Magnetometer: its axes on the ICM-20948 may differ from accel/gyro.
    Logged in native mag axes only.
  * MTF-02P flow / laser: logged as reported. Flow x/y signs and scale come
    from the known-slide test; the laser measures range along its beam
    (FRD +z direction if it points straight down).

FRD sanity checks
-----------------
  * Flat and still: frd_accel_z ~ -9.81 m/s^2 (specific force points up,
    z points down), frd_accel_x and frd_accel_y ~ 0.
  * Roll (right side down): frd_gyro_x positive.
  * Pitch (nose up):        frd_gyro_y positive.
  * Yaw (nose turns right): frd_gyro_z positive.

Usage examples
--------------
  python3 log_raw_data.py --label static_flat --duration 600
  python3 log_raw_data.py --label sixpos_zup --duration 30 --notes "native +z up"
  python3 log_raw_data.py --label rot_roll --duration 20 --notes "right side down, slow"
"""

import argparse
import csv
import json
import os
import statistics
import time
from datetime import datetime

from IMU_raw_function import Icm20948Raw, NATIVE_TO_FRD, to_frd
from MTF_02P_function import Mtf02pSensor


def parse_args():
    p = argparse.ArgumentParser(description="Raw IMU + MTF-02P logger (FRD)")
    p.add_argument("--label", required=True,
                   help="test name, used in the filename (e.g. static_flat)")
    p.add_argument("--duration", type=float, default=60.0,
                   help="seconds to log (default 60)")
    p.add_argument("--rate", type=float, default=100.0,
                   help="target loop rate in Hz (default 100)")
    p.add_argument("--outdir", default="logs", help="output directory")
    p.add_argument("--notes", default="",
                   help="free text: orientation, surface, height, etc.")
    return p.parse_args()


FRD_COLS = [
    "frd_accel_x", "frd_accel_y", "frd_accel_z",
    "frd_gyro_x", "frd_gyro_y", "frd_gyro_z",
]


def main():
    args = parse_args()
    period_ns = int(1e9 / args.rate)
    os.makedirs(args.outdir, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(args.outdir, f"{stamp}_{args.label}")
    csv_path = base + ".csv"
    meta_path = base + ".json"

    print("Initialising sensors...")
    imu = Icm20948Raw()
    flow = Mtf02pSensor()

    # First reads are used only to discover keys for the CSV header.
    imu_keys = list(imu.read().keys())
    flow_keys = list(flow.read().keys())

    meta = {
        "label": args.label,
        "notes": args.notes,
        "target_rate_hz": args.rate,
        "duration_s": args.duration,
        "start_wall_time": datetime.now().isoformat(),
        "imu_keys": imu_keys,
        "flow_keys": flow_keys,
        "body_frame": "FRD (x Forward, y Right, z Down)",
        "native_to_frd_matrix": [list(r) for r in NATIVE_TO_FRD],
        "native_to_frd_verified": False,
        "frd_columns_cover": "accel and gyro only (mag, flow, laser not converted)",
    }

    header = (
        ["cycle", "t_ns", "wall_time_s",
         "imu_t_start_ns", "imu_t_end_ns", "imu_new",
         "flow_t_start_ns", "flow_t_end_ns", "flow_new"]
        + [f"imu_{k}" for k in imu_keys]
        + FRD_COLS
        + [f"flow_{k}" for k in flow_keys]
    )

    print(f"IMU keys : {imu_keys}")
    print(f"Flow keys: {flow_keys}")
    print(f"Logging {args.duration:.0f} s at {args.rate:.0f} Hz -> {csv_path}")
    print("Ctrl+C to stop early.\n")

    prev_imu_raw = None
    prev_flow = None
    overruns = 0
    loop_dts = []
    imu_read_ms = []
    flow_read_ms = []
    imu_new_count = 0
    flow_new_count = 0
    n = 0

    t0 = time.perf_counter_ns()
    wall0 = time.time()
    next_deadline = t0
    end_ns = t0 + int(args.duration * 1e9)
    last_t = None

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        try:
            while True:
                if time.perf_counter_ns() >= end_ns:
                    break

                ti0 = time.perf_counter_ns()
                imu_r = imu.read()
                ti1 = time.perf_counter_ns()
                flow_r = flow.read()
                tf1 = time.perf_counter_ns()
                tf0 = ti1

                # IMU "new" = raw accel/gyro counts changed since last cycle.
                raw_now = (imu_r["raw_ax"], imu_r["raw_ay"], imu_r["raw_az"],
                           imu_r["raw_gx"], imu_r["raw_gy"], imu_r["raw_gz"])
                imu_new = int(raw_now != prev_imu_raw)
                flow_new = int(flow_r != prev_flow)
                prev_imu_raw, prev_flow = raw_now, flow_r
                imu_new_count += imu_new
                flow_new_count += flow_new

                fa = to_frd((imu_r["accel_x"], imu_r["accel_y"], imu_r["accel_z"]))
                fg = to_frd((imu_r["gyro_x"], imu_r["gyro_y"], imu_r["gyro_z"]))

                t_ns = ti0 - t0
                row = (
                    [n, t_ns, wall0 + t_ns / 1e9,
                     ti0 - t0, ti1 - t0, imu_new,
                     tf0 - t0, tf1 - t0, flow_new]
                    + [imu_r.get(k) for k in imu_keys]
                    + list(fa) + list(fg)
                    + [flow_r.get(k) for k in flow_keys]
                )
                writer.writerow(row)

                if last_t is not None:
                    loop_dts.append((ti0 - last_t) / 1e6)
                last_t = ti0
                imu_read_ms.append((ti1 - ti0) / 1e6)
                flow_read_ms.append((tf1 - tf0) / 1e6)
                n += 1

                if n % 500 == 0:
                    f.flush()

                # absolute-deadline sleep: no cumulative drift
                next_deadline += period_ns
                sleep_ns = next_deadline - time.perf_counter_ns()
                if sleep_ns > 0:
                    time.sleep(sleep_ns / 1e9)
                else:
                    overruns += 1
                    next_deadline = time.perf_counter_ns()
        except KeyboardInterrupt:
            print("\nStopped early.")

    elapsed = (time.perf_counter_ns() - t0) / 1e9

    def pct(data, q):
        if not data:
            return float("nan")
        s = sorted(data)
        return s[min(len(s) - 1, int(q * len(s)))]

    summary = {
        "cycles": n,
        "elapsed_s": elapsed,
        "achieved_loop_rate_hz": n / elapsed if elapsed else None,
        "imu_unique_sample_rate_hz": imu_new_count / elapsed if elapsed else None,
        "flow_unique_sample_rate_hz": flow_new_count / elapsed if elapsed else None,
        "overruns": overruns,
        "loop_dt_ms_mean": statistics.mean(loop_dts) if loop_dts else None,
        "loop_dt_ms_p99": pct(loop_dts, 0.99),
        "loop_dt_ms_max": max(loop_dts) if loop_dts else None,
        "imu_read_ms_mean": statistics.mean(imu_read_ms) if imu_read_ms else None,
        "imu_read_ms_max": max(imu_read_ms) if imu_read_ms else None,
        "flow_read_ms_mean": statistics.mean(flow_read_ms) if flow_read_ms else None,
        "flow_read_ms_max": max(flow_read_ms) if flow_read_ms else None,
    }
    meta["summary"] = summary
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)

    print("\n--- Summary ---")
    for k, v in summary.items():
        print(f"{k:28s} {v:.3f}" if isinstance(v, float) else f"{k:28s} {v}")
    print(f"\nSaved: {csv_path}\n       {meta_path}")
    print(
        "\nFlow 'unique sample rate' compares consecutive reads, so noisy "
        "values may overcount and repeated values undercount; the sensor's "
        "own hardware timestamp (if exposed) is better."
    )


if __name__ == "__main__":
    main()