"""
Compare MTF-02P laser readings to known true heights (stdlib only).

Usage:
  python3 laser_fit.py 0.10:logs/a.csv 0.20:logs/b.csv 0.50:logs/c.csv ...

Each argument is  true_height_in_meters:path_to_log.csv.
Prints per-height mean/std and a straight-line fit
    measured = scale * true + offset
A good sensor gives scale ~ 1.00, offset ~ 0, and small residuals.
Keep this script outside the logs/ folder.
"""

import csv
import math
import statistics
import sys


def load(path):
    vals, quals = [], []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            d = r.get("flow_distance_m", "")
            if d not in ("", None):
                vals.append(float(d))
            q = r.get("flow_rangefinder_quality", "")
            if q not in ("", None):
                quals.append(float(q))
    return vals, quals


def main(args):
    pts = []
    print(f"{'true_m':>8} {'mean_m':>9} {'std_m':>8} {'min_m':>8} {'max_m':>8} "
          f"{'distinct':>8} {'qual_mean':>9} {'n':>6}")
    for a in args:
        true_s, path = a.split(":", 1)
        true = float(true_s)
        vals, quals = load(path)
        m, s = statistics.fmean(vals), statistics.pstdev(vals)
        q = statistics.fmean(quals) if quals else float("nan")
        print(f"{true:8.3f} {m:9.4f} {s:8.4f} {min(vals):8.3f} {max(vals):8.3f} "
              f"{len(set(vals)):8d} {q:9.1f} {len(vals):6d}")
        pts.append((true, m))

    if len(pts) < 2:
        print("\nNeed at least 2 heights for a fit.")
        return
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    xm, ym = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - xm) ** 2 for x in xs)
    if sxx == 0:
        print("\nAll true heights are identical; can't fit.")
        return
    scale = sum((x - xm) * (y - ym) for x, y in zip(xs, ys)) / sxx
    offset = ym - scale * xm
    print(f"\nFit: measured = {scale:.4f} * true + {offset:+.4f} m")
    for x, y in pts:
        print(f"  true {x:.3f} m -> residual {y - (scale * x + offset):+.4f} m")
    print("\nIf the scale is far from 1 or residuals are large, suspect the driver's "
          "unit conversion, the sensor's minimum range, or the target surface.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])