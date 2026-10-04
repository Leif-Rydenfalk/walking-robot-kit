#!/usr/bin/env python3
"""limits_table.py: the per-servo angle limits as a markdown table. Read-only.

Reads /api/limits and /api/bus on the daemon, plus the hand-turn recording from record_limits.py
if you pass one, and writes a table: the saved limits per ID, the EEPROM read-back, the ranges
recorded so far, and whether the servo can be moved at all. No limits means no motion.

    python3 limits_table.py OUT.md [--recorder recorded.json] [--url http://127.0.0.1:8938]
"""
import argparse
import json
import time
import urllib.request

DEG = 360 / 4096


def get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.load(r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--url", default="http://127.0.0.1:8938")
    ap.add_argument("--recorder")
    a = ap.parse_args()
    lim = get(a.url + "/api/limits")
    bus = get(a.url + "/api/bus")
    rec = {}
    if a.recorder:
        try:
            rec = json.load(open(a.recorder)).get("servos", {})
        except OSError:
            pass
    rows = {r["id"]: r for r in bus["rows"]}
    ids = sorted(set(rows) | {int(k) for k in lim["servos"]} | {int(k) for k in rec})
    stamp = time.strftime("%Y-%m-%d %H:%M", time.localtime())
    out = [
        "| ID | joint | now (counts) | saved limits | span | EEPROM 9/11 read back | recorded by hand (tools/record_limits.py) | recorded in the daemon | can move |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for i in ids:
        r = rows.get(i, {})
        s = lim["servos"].get(str(i))
        h = rec.get(str(i))
        sn = r.get("seen")
        ee = r.get("ee") or {}
        name = r.get("name") or (h or {}).get("name", "")
        lims = f"{s['min']}..{s['max']} (ends {s['rec_min']}..{s['rec_max']}, by {s['who']} {s['when'][5:16]})" if s else "none"
        span = f"{(s['max'] - s['min']) * DEG:.0f} deg" if s else ""
        eep = s["eeprom"] if s and s.get("eeprom") else (f"factory {ee.get('min_angle')}..{ee.get('max_angle')}" if ee else "")
        hand = f"{h['min']}..{h['max']} ({(h['max'] - h['min']) * DEG:.0f} deg{', torque ON' if h.get('torque_on_samples') else ''})" if h else ""
        dmn = f"{sn['min']}..{sn['max']}{' WRAPPED' if sn.get('wrapped') else ''}" if sn else ""
        move = "yes, clamped" if r.get("band") else "no (refused)"
        out.append(f"| {i} | {name} | {r.get('pos', '')} | {lims} | {span} | {eep} | {hand} | {dmn} | {move} |")
    body = "\n".join(out)
    open(a.out, "w").write(f"<!-- generated {stamp} by servo-bus/tools/limits_table.py from {a.url}; do not edit the table by hand -->\n{body}\n")
    print(body)


if __name__ == "__main__":
    main()
