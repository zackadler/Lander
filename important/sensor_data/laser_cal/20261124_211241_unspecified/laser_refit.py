"""
Correct mistyped true heights in a laser_collect.py run and redo the fit.
Standard library only. Your original files are never modified.

Put this file in the laser_data/ folder, next to the run folders.

Usage (from inside laser_data/):
  python3 laser_refit.py --auto-cm                            newest run folder
  python3 laser_refit.py 20261124_211241_unspecified --auto-cm   a specific run folder
  python3 laser_refit.py --set 128=1.28 --set 109=1.09 --set 85=0.85
  python3 laser_refit.py --auto-cm --fit-min 0.1 0.3 0.5

If you give a run folder, put it BEFORE the options (--fit-min takes several
numbers and would otherwise swallow it).

Corrections:
  --auto-cm      any saved true height above --cm-threshold (default 3 m) is
                 assumed to be centimeters typed without a unit, and divided by 100
  --set OLD=NEW  change every point whose saved true height equals OLD to NEW
                 (OLD and NEW accept units: 128=1.28, 128=128cm, 0.9=90cm)
Both can be combined; --set wins over --auto-cm for a matching point.

Reads:   messages.csv (required), summary.csv (optional, for the recording window)
Writes into the run folder:
  messages_corrected.csv   copy of messages.csv with true_m fixed
  summary_corrected.csv    per-height statistics with the corrected heights
  summary_corrected.json   corrections applied, statistics and fits
"""

import argparse
import csv
import json
import os
import statistics
import sys


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


def parse_args():
    p = argparse.ArgumentParser(description="Fix saved laser heights and refit")
    p.add_argument("path", nargs="?", default=None,
                   help="run folder or its messages.csv (default: newest run folder next to this script)")
    p.add_argument("--auto-cm", action="store_true",
                   help="treat saved heights above --cm-threshold as centimeters")
    p.add_argument("--cm-threshold", type=float, default=3.0,
                   help="meters; saved heights above this are suspect (default 3)")
    p.add_argument("--set", action="append", default=[], metavar="OLD=NEW",
                   help="replace a saved height OLD with NEW (repeatable)")