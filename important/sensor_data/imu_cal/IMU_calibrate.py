#!/usr/bin/env python3
"""
Guided ICM-20948 calibration and FRD (Forward-Right-Down) axis mapping.

FRD body frame:  x = NOSE (forward), y = RIGHT side, z = BOTTOM (down).
Define which physical face of your board is "nose / right / bottom" once and
keep that definition for every step.

Stages (each step waits for you to press Enter):
  1. Six-position test  (30 s per face)
        -> accel bias, scale, misalignment, native->FRD axis map, gyro bias
  2. Gyro sign tests    (20 s each: roll right-side-down, nose up, nose right)
        -> each must give a POSITIVE FRD gyro x, y, z respectively
  3. Magnetometer map   (30 s slow rotation about each FRD axis)
        -> hard-iron offset and the mag->FRD axis map (only needed if you use the mag)

Everything is written to an output directory:
  raw_sixpos_*.csv, raw_gyro_*.csv, raw_mag_*.csv   raw samples (SI units + raw counts)
  calibration.json                                  results (machine readable)
  calibration_report.txt                            everything printed on screen

Usage:
  python3 imu_calibrate.py                          # run all stages
  python3 imu_calibrate.py --stages gyro,mag --outdir imu_cal_20260101_120000
                                                    # re-run stages, re-using earlier results
Requires: numpy, and IMU_raw_function.py (your Icm20948Raw driver) on the path.

Applying the result later:
  from imu_calibrate import CalibratedFRD
  cal = CalibratedFRD("imu_cal_.../calibration.json")
  ax, ay, az = cal.accel(sample)     # m/s^2 in FRD, bias/scale/alignment removed
  gx, gy, gz = cal.gyro(sample)      # rad/s  in FRD, bias removed
  mx, my, mz = cal.mag(sample)       # uT     in FRD, hard-iron removed
"""

import argparse
import csv
import itertools
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

from IMU_raw_function import Icm20948Raw, NATIVE_TO_FRD, G

# ----------------------------------------------------------------------------
# Settings
# ----------------------------------------------------------------------------
RATE_HZ = 100.0          # logging rate
SETTLE_S = 3.0           # pause after Enter (hands off) before logging starts
SIX_POS_S = 30.0
GYRO_TEST_S = 20.0
MAG_TEST_S = 30.0

ACC_STILL_STD = 0.20          # m/s^2   max accel noise for a "still" position
GYRO_STILL_STD_DPS = 1.0      # deg/s   max gyro noise for a "still" position
ACC_NORM_TOL = 0.20           # |a| must be within +-20 % of g
ALIGN_REJECT = 0.90           # dominant-axis fraction below this -> redo (~26 deg off)
ALIGN_WARN = 0.985            # below this -> warning (~10 deg off)
GYRO_MIN_ANGLE_DEG = 30.0     # net rotation needed for a gyro test
GYRO_DOMINANCE = 0.75         # fraction of the rotation that must be on the expected axis
MAG_MIN_PATH_DEG = 150.0      # total rotation needed in each mag run
MAG_CONFIDENCE_RATIO = 2.0    # 2nd-best / best score needed to call the mag map "confident"

ACC = ("accel_x", "accel_y", "accel_z")
GYR = ("gyro_x", "gyro_y", "gyro_z")
MAG = ("mag_x", "mag_y", "mag_z")
AXN = "xyz"

# ----------------------------------------------------------------------------
# Test definitions.  f = expected specific force (what an accel reads) in FRD, in g.
# An accelerometer reads +1 g on the axis that points UP.
# ----------------------------------------------------------------------------
SIX_POS = [
    dict(key="bottom_down", down="BOTTOM", f=(0, 0, -1),
         title="BOTTOM face down (level, normal attitude)",
         how="Lay it flat, nose pointing away from you. FRD +Z (down) points at the floor."),
    dict(key="top_down", down="TOP", f=(0, 0, 1),
         title="TOP face down (upside down)",
         how="Flip it over. FRD +Z now points at the ceiling."),
    dict(key="tail_down", down="TAIL", f=(1, 0, 0),
         title="TAIL face down (nose pointing straight UP)",
         how="Stand it on its tail end, nose at the ceiling."),
    dict(key="nose_down", down="NOSE", f=(-1, 0, 0),
         title="NOSE face down (nose pointing straight DOWN)",
         how="Stand it on its nose end, tail at the ceiling."),
    dict(key="right_down", down="RIGHT", f=(0, -1, 0),
         title="RIGHT side down",
         how="Stand it on its right-hand side face."),
    dict(key="left_down", down="LEFT", f=(0, 1, 0),
         title="LEFT side down",
         how="Stand it on its left-hand side face."),
]

GYRO_TESTS = [
    dict(key="roll_right_down", axis=0, title="ROLL: right side DOWN (expect +FRD gyro x)",
         how=("Start level, nose forward. After the beep, roll ONCE about the nose axis so the\n"
              "   RIGHT side goes down, ~60-90 deg, then HOLD still there until the end beep.")),
    dict(key="pitch_nose_up", axis=1, title="PITCH: nose UP (expect +FRD gyro y)",
         how=("Start level, nose forward. After the beep, pitch ONCE so the NOSE rises,\n"
              "   ~60-90 deg, then HOLD still there until the end beep.")),
    dict(key="yaw_nose_right", axis=2, title="YAW: nose RIGHT (expect +FRD gyro z)",
         how=("Start level, nose forward. After the beep, spin ONCE flat so the NOSE swings to the\n"
              "   RIGHT (clockwise seen from above), ~90 deg, then HOLD still until the end beep.")),
]

MAG_RUNS = [
    dict(key="x", axis=0, title="slow rotation about FRD X (the nose axis)",
         how="Like a barrel roll / rolling pin: the nose stays pointed at one spot while the board spins."),
    dict(key="y", axis=1, title="slow rotation about FRD Y (the right-wing axis)",
         how="Like a cartwheel / pitching loop: the right-wing axis stays fixed while nose and tail go round."),
    dict(key="z", axis=2, title="slow rotation about FRD Z (the vertical axis)",
         how="Like a lazy Susan: keep it flat and spin it around the vertical."),
]

# ----------------------------------------------------------------------------
# Small utilities
# ----------------------------------------------------------------------------
_report_fh = None


def say(msg=""):
    print(msg, flush=True)
    if _report_fh:
        _report_fh.write(msg + "\n")
        _report_fh.flush()


def banner(title):
    say("\n" + "=" * 72)
    say(title)
    say("=" * 72)


def beep():
    sys.stdout.write("\a")
    sys.stdout.flush()


def ask(prompt):
    ans = input(prompt).strip().lower()
    if ans in ("q", "quit", "exit"):
        raise KeyboardInterrupt
    return ans


def arr(samples, keys):
    return np.array([[s[k] for k in keys] for s in samples], dtype=float)


def times(samples):
    return np.array([s["t"] for s in samples], dtype=float)


def integrate(t, w):
    """Trapezoidal integral of an (N,3) signal over time stamps t."""
    dt = np.diff(t)
    return ((w[1:] + w[:-1]) * 0.5 * dt[:, None]).sum(axis=0)


def describe_map(P, src="native"):
    parts = []
    for i in range(3):
        j = int(np.argmax(np.abs(P[i])))
        parts.append(f"FRD {AXN[i]} = {'+' if P[i, j] > 0 else '-'}{src} {AXN[j]}")
    return ",  ".join(parts)


def snap_to_signed_perm(M):
    """Nearest signed permutation matrix to M (None if two rows pick the same column)."""
    P = np.zeros((3, 3))
    cols = set()
    for i in range(3):
        j = int(np.argmax(np.abs(M[i])))
        P[i, j] = 1.0 if M[i, j] > 0 else -1.0
        cols.add(j)
    return P if len(cols) == 3 else None


def fmt_mat(M, fmt="{:+9.5f}"):
    return "\n".join("      [" + " ".join(fmt.format(x) for x in row) + "]" for row in M)


def get_P(cal):
    a = cal.get("accel", {})
    if a.get("axis_map_P") is not None:
        return np.array(a["axis_map_P"], float), "six-position fit"
    return np.array(NATIVE_TO_FRD, float), "driver NATIVE_TO_FRD (six-position stage not run)"


def accel_to_frd(cal, a):
    """a: (N,3) native accel in m/s^2 -> (N,3) FRD (calibrated if available)."""
    acc = cal.get("accel", {})
    if acc.get("matrix_native_to_frd") is not None:
        M = np.array(acc["matrix_native_to_frd"], float)
        b = np.array(acc["bias_native_m_s2"], float)
        return (a - b) @ M.T
    return a @ np.array(NATIVE_TO_FRD, float).T


def fallback_gyro_bias(w):
    n = np.linalg.norm(w - np.median(w, axis=0), axis=1)
    return w[n <= np.percentile(n, 20)].mean(axis=0)


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    raise TypeError(f"not serialisable: {type(o)}")


def save_json(outdir, cal):
    with open(os.path.join(outdir, "calibration.json"), "w") as fh:
        json.dump(cal, fh, indent=2, default=_json_default)


def save_raw(outdir, name, samples):
    cols = ["t"] + [k for k in samples[0] if k != "t"]
    with open(os.path.join(outdir, name), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(samples)
    say(f"  raw data saved: {name}")


# ----------------------------------------------------------------------------
# Data collection
# ----------------------------------------------------------------------------
def countdown(seconds, msg):
    for k in range(int(seconds), 0, -1):
        print(f"\r  {msg} {k}... ", end="", flush=True)
        time.sleep(1.0)
    print("\r" + " " * 64 + "\r", end="", flush=True)
    beep()


def collect(imu, seconds, label):
    period = 1.0 / RATE_HZ
    samples = []
    t0 = time.perf_counter()
    nxt = t0
    end = t0 + seconds
    last_sec = -1
    while time.perf_counter() < end:
        s = imu.read()
        s["t"] = time.perf_counter() - t0
        samples.append(s)
        sec = int(s["t"])
        if sec != last_sec:
            last_sec = sec
            print(f"\r  [{label}] recording {sec:3d}/{seconds:.0f} s", end="", flush=True)
        nxt += period
        d = nxt - time.perf_counter()
        if d > 0:
            time.sleep(d)
        else:
            nxt = time.perf_counter()      # fell behind: resync instead of bursting
    beep()
    rate = len(samples) / seconds
    print(f"\r  [{label}] recording done: {len(samples)} samples ({rate:.0f} Hz)        ")
    if rate < 0.8 * RATE_HZ:
        say(f"  WARNING: effective rate {rate:.0f} Hz is well below the requested {RATE_HZ:.0f} Hz")
    return samples


# ----------------------------------------------------------------------------
# Stage 1: six-position test
# ----------------------------------------------------------------------------
def analyse_position(samples, used):
    a = arr(samples, ACC)
    g = arr(samples, GYR)
    mean, std = a.mean(0), a.std(0)
    gstd = np.degrees(g.std(0))
    norm = float(np.linalg.norm(mean))
    j = int(np.argmax(np.abs(mean)))
    s = 1 if mean[j] > 0 else -1
    frac = abs(mean[j]) / norm
    problems, warnings = [], []
    if len(samples) < 100:
        problems.append("too few samples")
    if std.max() > ACC_STILL_STD:
        problems.append(f"board was moving (accel std {std.max():.2f} m/s^2 > {ACC_STILL_STD})")
    if gstd.max() > GYRO_STILL_STD_DPS:
        problems.append(f"board was moving (gyro std {gstd.max():.2f} deg/s > {GYRO_STILL_STD_DPS})")
    if abs(norm - G) > ACC_NORM_TOL * G:
        problems.append(f"|accel| = {norm:.2f} m/s^2, expected about {G:.2f}")
    if frac < ALIGN_REJECT:
        problems.append(f"not resting on a face: only {frac * 100:.0f} % of gravity is on one axis")
    elif frac < ALIGN_WARN:
        warnings.append(f"about {math.degrees(math.acos(min(frac, 1.0))):.0f} deg off square; "
                        f"tilt errors leak into the misalignment estimate")
    if (j, s) in used:
        problems.append(f"this is the same face as position {used[(j, s)]} "
                        f"(native {'+' if s > 0 else '-'}{AXN[j]} is already pointing up there)")
    return dict(mean=mean, std=std, gyro_mean=g.mean(0), temp=float(np.mean([x["temp_c"] for x in samples])),
                dom=(j, s), frac=frac, norm=norm, n=len(samples), problems=problems, warnings=warnings)


def solve_accel(pos):
    """Fit f_frd = M (m_native - b) from the six positions."""
    m = np.array([p["mean"] for p in pos])
    f = np.array([p["f_exp"] for p in pos]) * G
    A = np.hstack([m, np.ones((len(m), 1))])
    X, _, rank, _ = np.linalg.lstsq(A, f, rcond=None)
    if rank < 4:
        return None
    M = X[:3].T
    c = X[3]
    b = -np.linalg.solve(M, c)
    rms = float(np.sqrt(np.mean((A @ X - f) ** 2)))
    P = snap_to_signed_perm(M)
    out = dict(matrix_native_to_frd=M, bias_native_m_s2=b, rms_residual_m_s2=rms, axis_map_P=P)
    if P is not None:
        K = M @ P.T
        out["K_scale_misalign_frd"] = K
        out["scale_factors_frd"] = np.diag(K)
        out["misalignment_deg"] = {
            f"{AXN[i]}_from_{AXN[j]}": math.degrees(K[i, j] / K[i, i])
            for i in range(3) for j in range(3) if i != j}
        out["driver_NATIVE_TO_FRD_matches"] = bool(np.array_equal(P, np.array(NATIVE_TO_FRD, float)))
    # classic per-native-axis bias / scale from the +/- pair on each axis
    bias_n, scale_n = np.zeros(3), np.zeros(3)
    for j in range(3):
        plus = next(p for p in pos if p["dom"] == (j, 1))["mean"][j]
        minus = next(p for p in pos if p["dom"] == (j, -1))["mean"][j]
        bias_n[j] = 0.5 * (plus + minus)
        scale_n[j] = (plus - minus) / (2.0 * G)
    out["classic_bias_native_m_s2"] = bias_n
    out["classic_scale_native"] = scale_n
    return out


def stage_six(imu, outdir, cal):
    banner("STAGE 1/3  SIX-POSITION TEST  (accel bias / scale / alignment, gyro bias)")
    say("FRD body frame: x = NOSE (forward), y = RIGHT side, z = BOTTOM (down).")
    say("Decide which physical faces of your board are NOSE / RIGHT / BOTTOM and keep that fixed.")
    say(f"Each position is held {SIX_POS_S:.0f} s. Rest it against a book or box corner as a square")
    say("reference: tilt errors of a few degrees become alignment errors in the result.")
    say("Type q at any prompt to quit (partial results are kept).")

    pos, used, attempts = [], {}, {}
    i = 0
    while i < len(SIX_POS):
        p = SIX_POS[i]
        tag = f"SIX {i + 1}/{len(SIX_POS)}"
        say(f"\n[{tag}] Put the {p['down']} face DOWN  ->  {p['title']}")
        say("   " + p["how"])
        ask("   Press Enter when it is in position and you have let go (q = quit)... ")
        countdown(SETTLE_S, "Hands off and hold still. Recording in")
        samples = collect(imu, SIX_POS_S, tag)
        attempts[i] = attempts.get(i, 0) + 1
        suffix = "" if attempts[i] == 1 else f"_try{attempts[i]}"
        save_raw(outdir, f"raw_sixpos_{i + 1}_{p['key']}{suffix}.csv", samples)

        r = analyse_position(samples, used)
        r.update(key=p["key"], f_exp=p["f"])
        for w in r["warnings"]:
            say(f"  warning: {w}")
        if r["problems"]:
            for pr in r["problems"]:
                say(f"  PROBLEM: {pr}")
            say("  -> this position is rejected; please repeat it.")
            continue
        j, s = r["dom"]
        say(f"  mean accel (native, m/s^2): x={r['mean'][0]:+.3f} y={r['mean'][1]:+.3f} z={r['mean'][2]:+.3f}")
        say(f"  native {'+' if s > 0 else '-'}{AXN[j]} points UP   (|a| = {r['norm']:.3f} m/s^2)")
        say(f"  [{tag}] DONE")
        used[(j, s)] = i + 1
        pos.append(r)
        i += 1

    say("\nSolving accelerometer calibration...")
    sol = solve_accel(pos)
    if sol is None:
        say("  FAILED: the six positions do not span all axes (singular fit). Re-run this stage.")
        return
    gyro_means = np.array([p["gyro_mean"] for p in pos])
    gyro_bias = gyro_means.mean(0)
    gyro_spread = np.degrees(gyro_means.std(0))

    acc = dict(sol)
    acc["positions"] = [dict(key=p["key"], mean_native_m_s2=p["mean"], std_m_s2=p["std"],
                             up_axis_native=f"{'+' if p['dom'][1] > 0 else '-'}{AXN[p['dom'][0]]}",
                             n=p["n"]) for p in pos]
    cal["accel"] = acc
    cal["gyro"] = dict(cal.get("gyro", {}), bias_native_rad_s=gyro_bias,
                       bias_native_dps=np.degrees(gyro_bias), bias_spread_dps=gyro_spread,
                       temp_c_during_bias=float(np.mean([p["temp"] for p in pos])))

    say("\n  Accelerometer result")
    say(f"    bias (native axes, m/s^2): {np.array2string(sol['bias_native_m_s2'], precision=4)}")
    say(f"    per-axis scale (native)  : {np.array2string(sol['classic_scale_native'], precision=4)}"
        f"   (1.0000 = perfect)")
    say(f"    fit residual RMS         : {sol['rms_residual_m_s2']:.4f} m/s^2"
        + ("   (good)" if sol["rms_residual_m_s2"] < 0.05 else
           "   (OK)" if sol["rms_residual_m_s2"] < 0.2 else "   (POOR: repeat with a steadier setup)"))
    P = sol["axis_map_P"]
    if P is None:
        say("    axis map: could not be resolved to a signed permutation (check the positions).")
        cal["accel"]["axis_map_P"] = None
    else:
        say(f"    native -> FRD axis map   : {describe_map(P)}")
        say(f"    det(P) = {np.linalg.det(P):+.0f}"
            + ("" if np.linalg.det(P) > 0 else
               "   WARNING: mirror image. A position was probably mislabelled (e.g. left/right swapped)."))
        say("    scale + misalignment matrix K (FRD, M = K*P):")
        say(fmt_mat(sol["K_scale_misalign_frd"]))
        say("    driver NATIVE_TO_FRD "
            + ("MATCHES the measured map." if sol["driver_NATIVE_TO_FRD_matches"] else
               "DOES NOT MATCH the measured map -> use the one in calibration.json, not the driver's."))
        # provisional gyro mapping until stage 2 verifies it
        cal["gyro"]["native_to_frd"] = P
        cal["gyro"]["mapping_verified"] = False
    say("\n  Gyro bias (still, averaged over six positions)")
    say(f"    native deg/s: {np.array2string(np.degrees(gyro_bias), precision=3)}"
        f"   (spread between positions {np.array2string(gyro_spread, precision=3)})")
    banner("STAGE 1 COMPLETE: six-position test done")


# ----------------------------------------------------------------------------
# Stage 2: gyro sign tests
# ----------------------------------------------------------------------------
def eval_gyro(samples, bias, P, cal, test):
    t = times(samples)
    w = arr(samples, GYR)
    if bias is None:
        bias = fallback_gyro_bias(w)
    wc = w - bias
    ang_n = np.degrees(integrate(t, wc))            # net rotation, native axes (deg)
    ang_f = P @ ang_n                                # net rotation, FRD axes (deg)
    total = float(np.linalg.norm(ang_n))
    peak = float(np.degrees(np.linalg.norm(wc, axis=1).max()))
    axis = test["axis"]
    dom = int(np.argmax(np.abs(ang_f)))
    frac = abs(ang_f[axis]) / total if total > 1e-9 else 0.0

    # independent accelerometer cross-check that the board really moved as instructed
    a = arr(samples, ACC)
    first, last = t <= 1.0, t >= t[-1] - 1.0
    d = accel_to_frd(cal, a[last].mean(0, keepdims=True))[0] - accel_to_frd(cal, a[first].mean(0, keepdims=True))[0]
    proj = {0: -d[1], 1: d[0]}.get(axis)            # roll right-down: fy falls; nose up: fx rises
    if proj is None:
        accel_note = "accelerometer cannot confirm yaw (no gravity change)"
    elif proj > 0.3 * G:
        accel_note = "accelerometer confirms the board physically moved the requested way"
    elif proj < -0.3 * G:
        accel_note = "accelerometer says the board moved the OPPOSITE way to the one requested"
    else:
        accel_note = "accelerometer saw little tilt change"

    if total < GYRO_MIN_ANGLE_DEG:
        status = "FAIL"
        msg = (f"no clear rotation (net {total:.0f} deg < {GYRO_MIN_ANGLE_DEG:.0f}). Rotate once, "
               f"then HOLD; do not rotate back before the end beep.")
    elif dom != axis:
        status = "FAIL"
        msg = (f"rotation was mostly about FRD {AXN[dom]} ({ang_f[dom]:+.0f} deg), not FRD {AXN[axis]}. "
               f"Either the wrong motion was done or the gyro axes differ from the accel axes.")
    elif frac < GYRO_DOMINANCE:
        status = "FAIL"
        msg = f"only {frac * 100:.0f} % of the rotation was about the expected axis; keep the motion cleaner."
    elif ang_f[axis] <= 0:
        status = "FAIL"
        msg = (f"SIGN WRONG: FRD gyro {AXN[axis]} integrated to {ang_f[axis]:+.0f} deg, should be positive. "
               f"If the accelerometer line above confirms the motion, that gyro axis is inverted.")
    else:
        status = "PASS"
        msg = f"FRD gyro {AXN[axis]} is positive ({ang_f[axis]:+.0f} deg) as required."
    return dict(status=status, message=msg, angle_native_deg=ang_n, angle_frd_deg=ang_f,
                peak_dps=peak, total_deg=total, accel_note=accel_note, bias_used=bias,
                accel_opposite=bool(proj is not None and proj < -0.3 * G))


def stage_gyro(imu, outdir, cal):
    banner("STAGE 2/3  GYRO SIGN TESTS  (each must give a POSITIVE FRD gyro x / y / z)")
    P, src = get_P(cal)
    gc = cal.setdefault("gyro", {})
    bias = np.array(gc["bias_native_rad_s"], float) if gc.get("bias_native_rad_s") is not None else None
    say(f"Axis map used for the sign check: {src}")
    if bias is None:
        say("No gyro bias from stage 1: estimating it from the quietest part of each run.")
    say(f"Each test is {GYRO_TEST_S:.0f} s. Make ONE smooth rotation and hold still at the end;")
    say("rotating back inside the window cancels the net angle the check relies on.")

    outcomes, attempts = [], {}
    k = 0
    while k < len(GYRO_TESTS):
        t = GYRO_TESTS[k]
        tag = f"GYRO {k + 1}/{len(GYRO_TESTS)}"
        say(f"\n[{tag}] {t['title']}")
        say("   " + t["how"])
        ask("   Start level, nose forward, then press Enter (q = quit)... ")
        countdown(SETTLE_S, "Hold level and still. Go at the beep, starting in")
        samples = collect(imu, GYRO_TEST_S, tag)
        attempts[k] = attempts.get(k, 0) + 1
        suffix = "" if attempts[k] == 1 else f"_try{attempts[k]}"
        save_raw(outdir, f"raw_gyro_{k + 1}_{t['key']}{suffix}.csv", samples)

        r = eval_gyro(samples, bias, P, cal, t)
        r["key"] = t["key"]
        a = r["angle_frd_deg"]
        say(f"  net rotation (FRD, deg): x={a[0]:+.1f}  y={a[1]:+.1f}  z={a[2]:+.1f}   (peak {r['peak_dps']:.0f} deg/s)")
        say(f"  {r['accel_note']}")
        say(f"  RESULT: {r['status']}: {r['message']}")
        if r["status"] == "PASS":
            say(f"  [{tag}] DONE")
            outcomes.append(r)
            k += 1
        else:
            ans = ask("  Enter = repeat this test, s = keep this result and continue (q = quit)... ")
            if ans == "s":
                outcomes.append(r)
                k += 1

    # derive the gyro's own native->FRD map: test k rotates about FRD axis k, so the dominant
    # native axis (and its sign) of test k is row k of the matrix.
    rows = []
    for r in outcomes:
        v = r["angle_native_deg"]
        if np.linalg.norm(v) < GYRO_MIN_ANGLE_DEG:
            rows.append(None)
            continue
        j = int(np.argmax(np.abs(v)))
        row = np.zeros(3)
        row[j] = 1.0 if v[j] > 0 else -1.0
        rows.append(row)
    Pg = np.vstack(rows) if all(r is not None for r in rows) else None
    if Pg is not None and (np.abs(Pg).sum(0) != 1).any():
        Pg = None

    say("\n  Gyro summary")
    all_pass = all(r["status"] == "PASS" for r in outcomes)
    for r, t in zip(outcomes, GYRO_TESTS):
        say(f"    {t['key']:<18} {r['status']}   FRD angle on expected axis: {r['angle_frd_deg'][t['axis']]:+.1f} deg")
    gc["sign_check"] = {r["key"]: dict(status=r["status"], angle_frd_deg=r["angle_frd_deg"],
                                       angle_native_deg=r["angle_native_deg"]) for r in outcomes}
    differs = False
    if Pg is not None:
        det = np.linalg.det(Pg)
        say(f"    gyro native -> FRD map measured from the tests: {describe_map(Pg)}")
        clean = all(np.abs(r["angle_native_deg"]).max() / np.linalg.norm(r["angle_native_deg"]) >= GYRO_DOMINANCE
                    for r in outcomes)
        if det < 0:
            say("    WARNING: that map is a mirror image (det = -1). Either a motion was done the wrong way round")
            say("             or one gyro axis is inverted (see the test marked SIGN WRONG). It was NOT saved.")
        elif np.array_equal(Pg, P):
            say("    gyro axes agree with the accelerometer axes.")
        else:
            say("    NOTE: gyro axes DIFFER from the accel axes found in stage 1.")
            if clean and not any(r["accel_opposite"] for r in outcomes):
                differs = True
                say("    The motions were clean and the accelerometer confirmed them, so the map above was saved")
                say("    as gyro native_to_frd; with it all three motions give positive FRD x, y, z.")
            else:
                say("    The motions were not clean enough to trust this map: repeat the stage.")
        if det > 0 and (np.array_equal(Pg, P) or differs):
            gc["native_to_frd"] = Pg
    elif "native_to_frd" not in gc:
        gc["native_to_frd"] = P
    gc["mapping_verified"] = bool(all_pass and Pg is not None and np.array_equal(Pg, P))
    gc["map_differs_from_accel"] = differs
    gc["signs_ok"] = bool(all_pass or differs)
    if all_pass:
        verdict = "gyro signs ALL PASS"
    elif differs:
        verdict = "gyro axes differ from accel axes; corrected gyro map saved (signs OK with it)"
    else:
        verdict = "gyro signs NOT all passed, see above"
    banner(f"STAGE 2 COMPLETE: {verdict}")


# ----------------------------------------------------------------------------
# Stage 3: magnetometer mapping
# ----------------------------------------------------------------------------
def analyse_mag_run(samples, cal, axis):
    t = times(samples)
    a = arr(samples, ACC)
    g = arr(samples, GYR)
    m = arr(samples, MAG)
    gc = cal.get("gyro", {})
    Pg = np.array(gc["native_to_frd"], float) if gc.get("native_to_frd") is not None else get_P(cal)[0]
    bias = np.array(gc["bias_native_rad_s"], float) if gc.get("bias_native_rad_s") is not None \
        else fallback_gyro_bias(g)
    valid = np.array([bool(s["mag_new"]) and not s["mag_ovf"] for s in samples]) & np.isfinite(m).all(1)
    wf = (g - bias) @ Pg.T
    dt = np.diff(t)
    path = float(np.degrees((np.linalg.norm(wf, axis=1)[1:] * dt).sum()))
    rms = np.sqrt((wf ** 2).mean(0))
    share = float(rms[axis] / np.linalg.norm(rms))
    mv = m[valid]
    problems, warnings = [], []
    if valid.sum() < 0.3 * len(samples) or valid.sum() < 100:
        problems.append(f"only {int(valid.sum())} fresh mag samples")
    if path < MAG_MIN_PATH_DEG:
        problems.append(f"total rotation only {path:.0f} deg (< {MAG_MIN_PATH_DEG:.0f}); rotate more")
    if share < 0.7:
        problems.append(f"rotation was only {share * 100:.0f} % about FRD {AXN[axis]}; keep the axis fixed")
    elif share < 0.85:
        warnings.append(f"rotation was {share * 100:.0f} % about FRD {AXN[axis]} (some wobble)")
    stds = mv.std(0) if len(mv) else np.zeros(3)
    quiet = int(np.argmin(stds)) if len(mv) else -1
    a_frd = accel_to_frd(cal, a)
    return dict(m=mv, a_frd=a_frd[valid], path=path, share=share, stds=stds, quiet=quiet,
                problems=problems, warnings=warnings)


def fit_hard_iron(m):
    A = np.hstack([2.0 * m, np.ones((len(m), 1))])
    y = (m ** 2).sum(1)
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    c = x[:3]
    r = math.sqrt(max(x[3] + c @ c, 1e-12))
    dev = np.linalg.norm(m - c, axis=1) / r
    return c, r, float(dev.std())


def search_mag_map(m, a_frd, c):
    """Find the signed permutation Q (mag native -> FRD) that keeps the angle between the
    gravity direction and the magnetic field constant while the board is turned."""
    keep = np.abs(np.linalg.norm(a_frd, axis=1) - G) < 0.1 * G          # drop samples with linear accel
    u = a_frd[keep] / np.linalg.norm(a_frd[keep], axis=1, keepdims=True)
    b = m[keep] - c
    b = b / np.linalg.norm(b, axis=1, keepdims=True)
    cands = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            Q = np.zeros((3, 3))
            for i in range(3):
                Q[i, perm[i]] = signs[i]
            cosv = np.einsum("ij,ij->i", u, b @ Q.T)
            cands.append((float(cosv.std()), float(cosv.mean()), Q))
    cands.sort(key=lambda x: x[0])
    # -Q always scores the same as Q (it only flips the sign of the cosine), so a std score cannot
    # separate them. The AK09916 is right-handed, so only proper rotations (det = +1) are candidates.
    proper = [c for c in cands if np.linalg.det(c[2]) > 0]
    best = proper[0]
    improper = [c for c in cands if np.linalg.det(c[2]) < 0 and not np.array_equal(c[2], -best[2])]
    return proper, improper[0], int(keep.sum())


def stage_mag(imu, outdir, cal):
    banner("STAGE 3/3  MAGNETOMETER MAPPING  (only matters if you will use the mag)")
    if not imu.has_mag:
        say("No magnetometer was found on the bypass bus: stage skipped.")
        return
    say(f"Each run is {MAG_TEST_S:.0f} s. Rotate SLOWLY (about 1-2 full turns, roughly 10-20 deg/s)")
    say("about the named axis, keeping that axis as fixed as you can. Stay away from laptops,")
    say("monitors, speakers and steel furniture, and keep the same place for all three runs.")

    runs, attempts = [], {}
    k = 0
    while k < len(MAG_RUNS):
        r_def = MAG_RUNS[k]
        tag = f"MAG {k + 1}/{len(MAG_RUNS)}"
        say(f"\n[{tag}] {r_def['title']}")
        say("   " + r_def["how"])
        ask("   Start from level, nose forward, then press Enter (q = quit)... ")
        countdown(SETTLE_S, "Hold still. Start rotating at the beep, starting in")
        samples = collect(imu, MAG_TEST_S, tag)
        attempts[k] = attempts.get(k, 0) + 1
        suffix = "" if attempts[k] == 1 else f"_try{attempts[k]}"
        save_raw(outdir, f"raw_mag_{k + 1}_{r_def['key']}{suffix}.csv", samples)
        r = analyse_mag_run(samples, cal, r_def["axis"])
        for w in r["warnings"]:
            say(f"  warning: {w}")
        if r["problems"]:
            for pr in r["problems"]:
                say(f"  PROBLEM: {pr}")
            say("  -> run rejected; please repeat it.")
            continue
        say(f"  rotated {r['path']:.0f} deg total, {r['share'] * 100:.0f} % about FRD {AXN[r_def['axis']]}")
        say(f"  mag std per native channel (uT): x={r['stds'][0]:.1f} y={r['stds'][1]:.1f} z={r['stds'][2]:.1f}"
            f"   -> quietest = mag {AXN[r['quiet']]}")
        say(f"  [{tag}] DONE")
        runs.append(r)
        k += 1

    say("\nFitting magnetometer...")
    m_all = np.vstack([r["m"] for r in runs])
    a_all = np.vstack([r["a_frd"] for r in runs])
    c, radius, dev = fit_hard_iron(m_all)
    say(f"  hard-iron offset (native, uT): {np.array2string(c, precision=2)}")
    say(f"  field magnitude after offset : {radius:.1f} uT   (sphere-fit scatter {dev * 100:.1f} %)")
    if dev > 0.08:
        say("  warning: scatter > 8 %: soft-iron distortion or nearby metal/electronics. Expect a poor mag.")

    cands, mirror_best, n_used = search_mag_map(m_all, a_all, c)
    best, second = cands[0], cands[1]
    ratio = second[0] / max(best[0], 1e-9)
    Q = best[2]
    dip = math.degrees(math.asin(max(-1.0, min(1.0, -best[1]))))
    confident = ratio >= MAG_CONFIDENCE_RATIO and best[0] < 0.15
    say(f"  mag native -> FRD map: {describe_map(Q, 'mag')}")
    if mirror_best[0] < 0.5 * best[0]:
        say("  WARNING: a mirrored (left-handed) axis map fits clearly better than any right-handed one.")
        say("           Either the mag really is mirrored or an earlier accel/gyro step was labelled wrongly.")
    say(f"  score (std of gravity/field angle cosine) best={best[0]:.3f}, runner-up={second[0]:.3f}, "
        f"ratio={ratio:.1f}   -> {'CONFIDENT' if confident else 'LOW CONFIDENCE (rotate through more orientations)'}")
    say(f"  implied magnetic dip angle {dip:+.0f} deg (positive = field points down; use as a sanity check only)")
    same_as_accel = bool(np.array_equal(Q, np.eye(3)))
    say("  mag axes " + ("ARE aligned with the FRD-mapped accel/gyro axes (Q = identity)."
                         if same_as_accel else "are NOT aligned with the FRD-mapped accel/gyro axes: apply Q."))

    # cross-check with the simple "axis that stays constant while rotating" test
    agree = []
    for r, r_def in zip(runs, MAG_RUNS):
        agree.append(bool(Q[r_def["axis"], r["quiet"]] != 0))
    say(f"  quiet-channel test agrees with Q for {sum(agree)}/3 runs"
        + ("" if all(agree) else "  (a disagreement usually means a poorly chosen rotation axis or a disturbed field)"))

    cal["mag"] = dict(hard_iron_offset_native_uT=c, field_magnitude_uT=radius, sphere_fit_scatter=dev,
                      native_to_frd=Q, aligned_with_accel_gyro=same_as_accel, confident=confident,
                      score_best=best[0], score_runner_up=second[0], dip_deg=dip,
                      quiet_channel_agrees=agree,
                      samples_used=int(n_used))
    banner("STAGE 3 COMPLETE: magnetometer mapping done")


# ----------------------------------------------------------------------------
# Applying the calibration
# ----------------------------------------------------------------------------
class CalibratedFRD:
    """Convert raw Icm20948Raw.read() samples to calibrated FRD vectors."""

    def __init__(self, path):
        with open(path) as fh:
            c = json.load(fh)
        a, g, m = c.get("accel", {}), c.get("gyro", {}), c.get("mag")
        self._M = np.array(a["matrix_native_to_frd"], float)
        self._ba = np.array(a["bias_native_m_s2"], float)
        self._Pg = np.array(g["native_to_frd"], float)
        self._bg = np.array(g["bias_native_rad_s"], float)
        self._mag = None
        if m:
            self._mag = (np.array(m["native_to_frd"], float), np.array(m["hard_iron_offset_native_uT"], float))

    def accel(self, s):
        return tuple(self._M @ (np.array([s[k] for k in ACC]) - self._ba))

    def gyro(self, s):
        return tuple(self._Pg @ (np.array([s[k] for k in GYR]) - self._bg))

    def mag(self, s):
        if self._mag is None:
            return None
        Q, c = self._mag
        return tuple(Q @ (np.array([s[k] for k in MAG]) - c))


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def final_summary(cal, outdir):
    banner("SUMMARY")
    a, g, m = cal.get("accel"), cal.get("gyro"), cal.get("mag")
    if a:
        P = a.get("axis_map_P")
        say(f"Accel  : residual {a['rms_residual_m_s2']:.4f} m/s^2; "
            + (describe_map(np.array(P)) if P is not None else "axis map unresolved"))
    if g and g.get("sign_check"):
        res = ", ".join(f"{k}={v['status']}" for k, v in g["sign_check"].items())
        say(f"Gyro   : {res}; signs OK = {g.get('signs_ok')}; map differs from accel = {g.get('map_differs_from_accel')}")
    if m:
        say(f"Mag    : {describe_map(np.array(m['native_to_frd']), 'mag')}; confident = {m['confident']}")
    say(f"\nFiles are in: {os.path.abspath(outdir)}")
    say("  raw_*.csv (raw data), calibration.json (results), calibration_report.txt (this log)")


def main():
    global _report_fh
    ap = argparse.ArgumentParser(description="Guided ICM-20948 calibration + FRD mapping")
    ap.add_argument("--outdir", default=None, help="output directory (reuse one to re-run stages)")
    ap.add_argument("--stages", default="six,gyro,mag", help="comma list from: six,gyro,mag")
    ap.add_argument("--bus", type=int, default=3)
    ap.add_argument("--address", type=lambda x: int(x, 0), default=0x68)
    args = ap.parse_args()

    stages = [s.strip() for s in args.stages.split(",") if s.strip()]
    bad = [s for s in stages if s not in ("six", "gyro", "mag")]
    if bad:
        ap.error(f"unknown stage(s): {bad}")

    outdir = args.outdir or datetime.now().strftime("imu_cal_%Y%m%d_%H%M%S")
    os.makedirs(outdir, exist_ok=True)
    _report_fh = open(os.path.join(outdir, "calibration_report.txt"), "a")

    cal = {}
    cal_path = os.path.join(outdir, "calibration.json")
    if os.path.exists(cal_path):
        with open(cal_path) as fh:
            cal = json.load(fh)
        say(f"Loaded earlier results from {cal_path}")

    imu = Icm20948Raw(bus_num=args.bus, address=args.address)
    say(f"WHO_AM_I = 0x{imu.who_am_i:02X} (ICM-20948), magnetometer found: {imu.has_mag}")
    say(f"Output directory: {os.path.abspath(outdir)}")
    cal["meta"] = dict(created=datetime.now().isoformat(timespec="seconds"), rate_hz=RATE_HZ,
                       accel_lsb_per_g=getattr(imu, "accel_scale", None),
                       gyro_lsb_per_dps=getattr(imu, "gyro_scale", None),
                       frd="x=forward(nose), y=right, z=down; accel reads +g on the axis pointing up")

    try:
        if "six" in stages:
            stage_six(imu, outdir, cal)
            save_json(outdir, cal)
        if "gyro" in stages:
            stage_gyro(imu, outdir, cal)
            save_json(outdir, cal)
        if "mag" in stages:
            stage_mag(imu, outdir, cal)
            save_json(outdir, cal)
    except KeyboardInterrupt:
        say("\nInterrupted: partial results are saved.")
    finally:
        save_json(outdir, cal)
        final_summary(cal, outdir)
        _report_fh.close()


if __name__ == "__main__":
    main()
