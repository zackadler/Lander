"""
Interactive laser (MTF-02P rangefinder) calibration run.

Needs, in the same folder:  MTF_02P_function.py

For each planned height the script prompts you. Put the target at that
true distance, hold the rig steady, press Enter, and it records for
--settle + --duration seconds. At the end it prints a table and fits
    measured = scale * true + offset
and saves everything.

Prompt keys at each height:
  Enter          record at the suggested height
  <number>       record, but use YOUR measured true height instead
                 (meters by default; "48cm" and "480mm" also work)
  s              skip this height
  q              stop and fit what has been collected so far

Measure the true height from the sensor's window (the lens face) to the
target, with the rig pointing straight down and the target flat, matte
and perpendicular to the beam.

Saved to  <outdir>/<timestamp>_<surface>/ :
  messages.csv   EVERY rangefinder and optic-flow message received
                 (raw mm incl. invalid/negative values, quality, payload hex,
                 which point and true height it belongs to, settled flag)
  summary.csv    per-height statistics
  summary.json   settings, notes, per-height statistics and the fits

Timestamps: t_ns is the time the message was drained from the serial
port, not the time the sensor measured it. Several messages drained in
one poll share a timestamp. Use message counts/rates, not these times,
for latency work.

Usage:
  python3 laser_collect.py --surface white_board
  python3 laser_collect.py --surface dark_carpet --heights 0.3 0.5 1.0 --duration 15
"""

import argparse
import csv
import json
import os
import statistics
import struct
import sys
import time
from datetime import datetime

from MTF_02P_function import (
    MSP2_SENSOR_RANGEFINDER,
    Mtf02pSensor,
    decode_sensor_message,
)

DEFAULT_HEIGHTS = [0.05, 0.10, 0.20, 0.30, 0.50, 0.75, 1.00, 1.50, 2.00]

MSG_COLUMNS = [
    "point_idx", "true_m", "settled", "msg_type", "t_ns", "wall_time_s",
    "distance_m", "raw_mm", "rangefinder_quality",
    "motion_x", "motion_y", "flow_quality", "payload_hex",
]

SUMMARY_FIELDS = [
    "point_idx", "true_m", "partial", "window_s",
    "n_range", "n_valid", "n_invalid", "range_rate_hz",
    "mean_m", "median_m", "std_m", "min_m", "max_m", "distinct", "ratio",
    "range_quality_mean",
    "n_flow", "flow_rate_hz", "flow_quality_mean",
    "motion_x_mean", "motion_x_std", "motion_y_mean", "motion_y_std",
]


class Mtf02pStream(Mtf02pSensor):
    """Same sensor, but hands back EVERY decoded message instead of
    only the latest value."""

    def flush(self):
        self._ser.reset_input_buffer()
        self._parser.reset()

    def poll(self):
        out = []
        waiting = self._ser.in_waiting
        if not waiting:
            return out
        t_ns = time.perf_counter_ns()
        for byte in self._ser.read(waiting):
            result = self._parser.feed(byte)
            if result is None:
                continue
            function, payload = result
            reading = decode_sensor_message(function, payload)
            if reading is None:
                continue
            reading["t_ns"] = t_ns
            reading["payload_hex"] = payload.hex()
            if function == MSP2_SENSOR_RANGEFINDER and len(payload) >= 5:
                reading["raw_mm"] = struct.unpack("<i", payload[1:5])[0]
            out.append(reading)
        return out


def parse_args():
    p = argparse.ArgumentParser(description="Interactive laser calibration run")
    p.add_argument("--heights", type=float, nargs="+", default=DEFAULT_HEIGHTS,
                   help="planned true heights in meters")
    p.add_argument("--duration", type=float, default=10.0,
                   help="seconds recorded per height after settling (default 10)")
    p.add_argument("--settle", type=float, default=1.0,
                   help="seconds at the start of each height flagged as "
                        "'not settled' and left out of the statistics (default 1)")
    p.add_argument("--surface", default="unspecified",
                   help="target surface label, used in the folder name")
    p.add_argument("--notes", default="", help="free text saved with the run")
    p.add_argument("--fit-min-height", type=float, default=0.10,
                   help="second fit uses only heights >= this (default 0.10 m), "
                        "since very short range is expected to misbehave")
    p.add_argument("--port", default="/dev/ttyAMA0")
    p.add_argument("--outdir", default="laser_data")
    return p.parse_args()


def parse_height(text):
    t = text.strip().lower().replace(" ", "")
    try:
        if t.endswith("cm"):
            v = float(t[:-2]) / 100.0
        elif t.endswith("mm"):
            v = float(t[:-2]) / 1000.0
        elif t.endswith("m"):
            v = float(t[:-1])
        else:
            v = float(t)
    except ValueError:
        return None
    return v if v > 0 else None


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return "q"


def record_point(sensor, writer, fh, idx, true_m, duration, settle, t_run0):
    """Record one height. Returns the point dict and whether it ran to the end."""
    pt = {"idx": idx, "true_m": true_m, "range": [], "flow": [],
          "window_s": 0.0, "partial": False}
    sensor.flush()
    start_ns = time.perf_counter_ns()
    settle_end_ns = start_ns + int(settle * 1e9)
    end_ns = settle_end_ns + int(duration * 1e9)
    total_s = settle + duration
    last_mm = None
    last_print = 0.0
    completed = True

    try:
        while True:
            now_ns = time.perf_counter_ns()
            if now_ns >= end_ns:
                break
            msgs = sensor.poll()
            wall = time.time()
            for m in msgs:
                settled = int(m["t_ns"] >= settle_end_ns)
                if m["type"] == "rangefinder":
                    raw = m.get("raw_mm")
                    dist = m["distance_m"]
                    last_mm = raw
                    pt["range"].append((settled, dist, raw, m["quality"]))
                    writer.writerow([idx, f"{true_m:.4f}", settled, "rangefinder",
                                     m["t_ns"] - t_run0, f"{wall:.6f}",
                                     "" if dist is None else f"{dist:.4f}",
                                     "" if raw is None else raw, m["quality"],
                                     "", "", "", m["payload_hex"]])
                elif m["type"] == "optic_flow":
                    pt["flow"].append((settled, m["motion_x"], m["motion_y"], m["quality"]))
                    writer.writerow([idx, f"{true_m:.4f}", settled, "optic_flow",
                                     m["t_ns"] - t_run0, f"{wall:.6f}",
                                     "", "", "", m["motion_x"], m["motion_y"],
                                     m["quality"], m["payload_hex"]])
            if time.monotonic() - last_print > 0.2:
                last_print = time.monotonic()
                el = (now_ns - start_ns) / 1e9
                tag = "settling" if now_ns < settle_end_ns else "recording"
                print(f"\r  {tag:9s} {el:5.1f}/{total_s:.1f} s   "
                      f"range msgs: {len(pt['range']):4d}   last: {last_mm} mm   ",
                      end="", flush=True)
            time.sleep(0.004)
    except KeyboardInterrupt:
        completed = False
        pt["partial"] = True

    pt["window_s"] = max(0.0, (min(time.perf_counter_ns(), end_ns) - settle_end_ns) / 1e9)
    print("\r" + " " * 78 + "\r", end="")
    fh.flush()
    return pt, completed


def _ms(x):
    if not x:
        return None, None
    return statistics.fmean(x), (statistics.pstdev(x) if len(x) > 1 else 0.0)


def point_stats(pt):
    win = pt["window_s"]
    rng = [r for r in pt["range"] if r[0]]
    vals = [r[1] for r in rng if r[1] is not None]
    quals = [r[3] for r in rng if r[3] is not None]
    fl = [f for f in pt["flow"] if f[0]]
    st = {k: None for k in SUMMARY_FIELDS}
    st.update({
        "point_idx": pt["idx"], "true_m": pt["true_m"], "partial": int(pt["partial"]),
        "window_s": round(win, 3),
        "n_range": len(rng), "n_valid": len(vals), "n_invalid": len(rng) - len(vals),
        "range_rate_hz": len(rng) / win if win > 0 else None,
        "n_flow": len(fl), "flow_rate_hz": len(fl) / win if win > 0 else None,
    })
    if vals:
        mean, std = _ms(vals)
        st.update({
            "mean_m": mean, "median_m": statistics.median(vals), "std_m": std,
            "min_m": min(vals), "max_m": max(vals), "distinct": len(set(vals)),
            "ratio": mean / pt["true_m"],
            "range_quality_mean": statistics.fmean(quals) if quals else None,
        })
    if fl:
        st["flow_quality_mean"] = statistics.fmean([f[3] for f in fl])
        st["motion_x_mean"], st["motion_x_std"] = _ms([f[1] for f in fl])
        st["motion_y_mean"], st["motion_y_std"] = _ms([f[2] for f in fl])
    return st


def linear_fit(points):
    """Least squares measured = scale * true + offset."""
    if len(points) < 2:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    xm, ym = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - xm) ** 2 for x in xs)
    if sxx == 0:
        return None
    scale = sum((x - xm) * (y - ym) for x, y in zip(xs, ys)) / sxx
    offset = ym - scale * xm
    res = [(x, y - (scale * x + offset)) for x, y in points]
    rms = (sum(r * r for _, r in res) / len(res)) ** 0.5
    return {"scale": scale, "offset_m": offset, "rms_residual_m": rms,
            "n_points": len(points), "residuals_m": res}


def fmt(v, spec, width):
    return f"{'-':>{width}}" if v is None else f"{v:{width}{spec}}"


def print_table(stats):
    print(f"\n{'true_m':>7} {'mean_m':>8} {'median':>8} {'std_m':>8} {'min_m':>7} {'max_m':>7} "
          f"{'distinct':>8} {'ratio':>6} {'valid/n':>9} {'rate_Hz':>7} {'qual':>5}")
    for s in stats:
        vn = f"{s['n_valid']}/{s['n_range']}"
        flag = "  (partial)" if s["partial"] else ""
        print(f"{s['true_m']:7.3f} {fmt(s['mean_m'], '.4f', 8)} {fmt(s['median_m'], '.4f', 8)} "
              f"{fmt(s['std_m'], '.4f', 8)} {fmt(s['min_m'], '.3f', 7)} {fmt(s['max_m'], '.3f', 7)} "
              f"{fmt(s['distinct'], 'd', 8)} {fmt(s['ratio'], '.3f', 6)} {vn:>9} "
              f"{fmt(s['range_rate_hz'], '.1f', 7)} {fmt(s['range_quality_mean'], '.0f', 5)}{flag}")


def print_fit(title, fit):
    if fit is None:
        print(f"{title}: not enough distinct valid points to fit.")
        return
    print(f"{title}: measured = {fit['scale']:.4f} * true {fit['offset_m']:+.4f} m   "
          f"(rms residual {fit['rms_residual_m'] * 1000:.1f} mm, {fit['n_points']} points)")
    for x, r in fit["residuals_m"]:
        print(f"    true {x:6.3f} m -> residual {r * 1000:+7.1f} mm")


def main():
    args = parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(args.outdir, f"{stamp}_{args.surface}")
    os.makedirs(run_dir, exist_ok=True)

    sensor = Mtf02pStream(port=args.port)
    print(f"Checking for sensor data on {args.port} ...")
    deadline = time.time() + 3.0
    got = False
    while time.time() < deadline and not got:
        got = bool(sensor.poll())
        time.sleep(0.05)
    if not got:
        sys.exit("No messages from the MTF-02P in 3 s. Check power, wiring and port "
                 "(--port), then try again.")
    print("Sensor is streaming.")
    print(f"Saving to {run_dir}/")
    print("Keys at each prompt: Enter = record | <number> = use that true height "
          "(m; or '48cm') | s = skip | q = finish and fit")

    queue = list(args.heights)
    points = []
    idx = 0
    t_run0 = time.perf_counter_ns()
    start_wall = datetime.now().isoformat()

    msg_path = os.path.join(run_dir, "messages.csv")
    with open(msg_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(MSG_COLUMNS)
        try:
            while True:
                if queue:
                    suggested = queue.pop(0)
                    ans = ask(f"\nNext height {suggested:.3f} m. Place target, hold steady. "
                              f"[Enter]=record [number]=other height [s]=skip [q]=finish > ")
                    if ans == "q":
                        break
                    if ans == "s":
                        continue
                    true_m = suggested if ans == "" else parse_height(ans)
                    if true_m is None:
                        print("  Couldn't read that height; skipping this one.")
                        continue
                else:
                    ans = ask("\nPlanned heights done. Type another height to record "
                              "(e.g. 0.8 or 80cm), or press Enter to finish and fit > ")
                    if ans in ("", "q"):
                        break
                    true_m = parse_height(ans)
                    if true_m is None:
                        print("  Couldn't read that height.")
                        continue

                idx += 1
                print(f"Point {idx}: true height {true_m:.3f} m")
                pt, completed = record_point(sensor, writer, fh, idx, true_m,
                                             args.duration, args.settle, t_run0)
                points.append(pt)
                st = point_stats(pt)
                if st["mean_m"] is not None:
                    print(f"  done: mean {st['mean_m']:.4f} m, std {st['std_m']:.4f} m, "
                          f"ratio measured/true {st['ratio']:.3f}, "
                          f"{st['n_valid']}/{st['n_range']} valid, "
                          f"{st['range_rate_hz']:.1f} range msgs/s")
                else:
                    print(f"  done: no valid range readings ({st['n_range']} messages).")
                if not completed:
                    print("  (interrupted; partial point kept)")
                    break
        except KeyboardInterrupt:
            print("\nStopped.")

    if not points:
        print("No points recorded.")
        return

    stats = [point_stats(p) for p in points]
    print_table(stats)

    pts_all = [(s["true_m"], s["mean_m"]) for s in stats if s["mean_m"] is not None]
    pts_min = [p for p in pts_all if p[0] >= args.fit_min_height]
    fit_all = linear_fit(pts_all)
    fit_min = linear_fit(pts_min) if len(pts_min) != len(pts_all) else None
    print()
    print_fit("Fit (all heights)", fit_all)
    if len(pts_min) != len(pts_all):
        print_fit(f"Fit (heights >= {args.fit_min_height:g} m)", fit_min)

    if all(s["std_m"] == 0 for s in stats if s["std_m"] is not None):
        print("\nNote: every height has zero spread. The output is clamped or heavily quantized.")
    print("\nA healthy sensor gives scale ~ 1.00, offset ~ 0 and small residuals. "
          "A ratio that changes with height means nonlinearity (often the minimum range); "
          "a constant ratio far from 1 suggests a unit/scale problem.")

    with open(os.path.join(run_dir, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
        w.writeheader()
        w.writerows(stats)

    def clean_fit(fit):
        return None if fit is None else {
            k: ([list(r) for r in v] if k == "residuals_m" else v) for k, v in fit.items()}

    meta = {
        "start_wall_time": start_wall,
        "surface": args.surface,
        "notes": args.notes,
        "port": args.port,
        "planned_heights_m": args.heights,
        "settle_s": args.settle,
        "duration_s": args.duration,
        "fit_min_height_m": args.fit_min_height,
        "distance_conversion": "raw_mm / 1000 (negative raw_mm is invalid and has no distance_m)",
        "points": stats,
        "fit_all": clean_fit(fit_all),
        "fit_min_height": clean_fit(fit_min),
    }
    with open(os.path.join(run_dir, "summary.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print(f"\nSaved: {run_dir}/messages.csv, summary.csv, summary.json")


if __name__ == "__main__":
    main()