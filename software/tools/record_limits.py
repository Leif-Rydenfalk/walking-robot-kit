#!/usr/bin/env python3
"""record_limits.py: keep each servo's lowest and highest position while you turn the joints by hand.

Torque stays off. Reads the daemon's /api/bus only and never writes to a servo. Writes <out>
once a second: {id: {name, min, max, n, torque_on_samples}}.

    python3 record_limits.py <out.json> [seconds=900]

Turn one joint at a time, slowly, from one end stop to the other. Then save the band on the
page, or hand this file to limits_table.py to see what was recorded.
"""
import json, sys, time, urllib.request
out, dur = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 900
seen, t0, last = {}, time.time(), 0
while time.time() - t0 < dur:
    try:
        d = json.load(urllib.request.urlopen("http://127.0.0.1:8938/api/bus", timeout=2))
        for r in d.get("rows", []):
            if r.get("stale") or r.get("verdict") != "ok" or not (0 <= r["pos"] <= 4095):
                continue
            s = seen.setdefault(str(r["id"]), {"name": r["name"], "min": r["pos"], "max": r["pos"], "n": 0, "torque_on_samples": 0})
            s["min"], s["max"], s["n"] = min(s["min"], r["pos"]), max(s["max"], r["pos"]), s["n"] + 1
            s["torque_on_samples"] += bool(r.get("torque"))
    except Exception as e:
        pass
    if time.time() - last > 1:
        json.dump({"started": t0, "updated": time.time(), "servos": seen}, open(out, "w"), indent=1); last = time.time()
    time.sleep(0.1)
