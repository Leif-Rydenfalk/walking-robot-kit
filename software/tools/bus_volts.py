#!/usr/bin/env python3
"""bus_volts.py: every servo's supply voltage, and a verdict on whether the bus is safe.

Read-only. Nothing is torqued and nothing moves. Exit 0 safe, 1 unsafe, 2 could not measure.

    python3 bus_volts.py           # the table and a verdict
    python3 bus_volts.py --quiet   # the verdict line only, for a script to gate on

The window is the HD-1910 datasheet range, narrowed at the bottom by what a pack does under load:

  above 8.4 V   out of spec, and the servos will be damaged. Two packs in series read about 13 V
  below 5.0 V   too flat to hold a gait; a pack that sags here stops in its first seconds

A servo that does not answer is not a pass. An unread voltage is not a safe one.
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fte  # noqa: E402

V_MAX, V_MIN = 8.4, 5.0
IDS = [11, 13, 14, 21, 22, 23, 24, 30, 31, 32, 33, 34, 112, 120, 121]


def read_volts(bus, ids=None, tries=2):
    """-> ({id: volts}, [ids that did not answer]). Read-only: REG_TELEM is a RAM read."""
    ids = list(ids or IDS)
    out, missing = {}, []
    for i in ids:
        v = None
        for _ in range(tries):
            raw, _ = bus.batch([fte.read_pkt(i, fte.REG_TELEM, 15)])
            d = bus.payload(raw[0], i, 15)
            if d is not None:
                v = d[6] / 10.0
                break
        if v is None:
            missing.append(i)
        else:
            out[i] = v
    return out, missing


def verdict(volts, missing):
    """-> (exit code, one line). Over-voltage outranks everything; nothing read is never a pass."""
    if not volts:
        return 2, "unknown: no servo answered, so the bus voltage is unknown. Not safe to torque."
    hi, lo = max(volts.values()), min(volts.values())
    over = sorted(i for i, v in volts.items() if v > V_MAX)
    if over:
        return 1, ("UNSAFE over-voltage: %.1f V on the bus (limit %.1f V), ids %s. Unplug the pack now. "
                   "Nothing may be torqued." % (hi, V_MAX, over))
    if lo < V_MIN:
        return 1, ("UNSAFE flat pack: %.1f V (floor %.1f V). Charge it; a run started here stops in "
                   "seconds." % (lo, V_MIN))
    if missing:
        return 2, ("unknown: %.1f-%.1f V on the %d servos that answered, but %s did not answer, "
                   "and an unread voltage is not a safe one." % (lo, hi, len(volts), missing))
    return 0, "safe: %.1f-%.1f V on all %d servos, inside %.1f-%.1f V." % (lo, hi, len(volts), V_MIN, V_MAX)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--ids", default=None, help="comma separated, default every known the robot servo")
    ap.add_argument("--json", default=None, help="write the reading to this file as well")
    a = ap.parse_args()
    bus = fte.Bus(0, "bus_volts")
    ids = [int(x) for x in a.ids.split(",")] if a.ids else IDS
    volts, missing = read_volts(bus, ids)
    code, line = verdict(volts, missing)
    if not a.quiet:
        for i in sorted(volts):
            print("id %3d  %4.1f V%s" % (i, volts[i], "   OVER LIMIT" if volts[i] > V_MAX else
                                         ("   under floor" if volts[i] < V_MIN else "")))
        for i in missing:
            print("id %3d  no reply" % i)
    print(line, flush=True)
    if a.json:
        json.dump({"t": time.strftime("%F %T %Z"), "volts": volts, "missing": missing,
                   "v_max": V_MAX, "v_min": V_MIN, "code": code, "verdict": line}, open(a.json, "w"), indent=1)
    raise SystemExit(code)


if __name__ == "__main__":
    main()
