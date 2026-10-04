#!/usr/bin/env python3
"""zero_now.py: capture the zero pose while you hold the robot, and write it down.

The convention is count 2048 = 0 rad on every joint. Servo horns never go on at exactly 2048, so
the difference goes into one number per joint and the runtime keeps seeing its own zero.

Nothing is written to a servo's EEPROM except its angle band, and nothing moves: torque stays off
throughout and the tool refuses to record a pose that is not still.

    python3 zero_now.py            # wait for stillness, print what it would write
    python3 zero_now.py --write    # write it
"""
import argparse
import json
import os
import statistics
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fte  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = os.environ.get("SERVO_BUS_URL", "http://127.0.0.1:8938")
CENTRE, HALF, INSET = 2048, 512, 34
STILL_COUNTS, STILL_S, HZ = 12, 2.0, 20.0
# Side of the two hip yaws (120, 121) is NOT confirmed, so they are left out: a hip yaw driven to the
# wrong side is worse than one not driven, and it moves 3 deg of a 20 deg gait.
JOINTS = {"left_hip_roll": 21, "left_hip_pitch": 22, "left_knee": 23, "left_ankle": 24,
          "right_hip_roll": 11, "right_hip_pitch": 112, "right_knee": 13, "right_ankle": 14,
          "neck_pitch": 30, "head_pitch": 31, "head_yaw": 32, "head_roll": 33}
HOMED = {21, 120, 121}          # bands measured from real mechanical stalls: shift, never replace
HEAD_HALF = 342                 # the head is held, not walked, so it gets 30 deg rather than 45


def post(path, body):
    req = urllib.request.Request(D + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        return {"ok": False, "why": "HTTP %d %s" % (e.code, e.read()[:160].decode("utf8", "replace"))}
    except Exception as e:                                        # noqa: BLE001
        return {"ok": False, "why": repr(e)}


def limits():
    with urllib.request.urlopen(D + "/api/limits", timeout=10) as r:
        return json.load(r)["servos"]


def wait_still(bus, ids, timeout):
    """-> ({id: median count}, note). Refuses rather than recording a pose that is moving."""
    n = int(STILL_S * HZ)
    hist = {i: [] for i in ids}
    t_end = time.time() + timeout
    said = 0.0
    while time.time() < t_end:
        frames = [fte.read_pkt(i, fte.REG_TELEM, 15) for i in ids]
        reps, _ = bus.batch(frames)
        for i, raw in zip(ids, reps):
            d = bus.payload(raw, i, 15)
            if d is not None:
                hist[i].append(fte.s16(d[0], d[1]))
                del hist[i][:-n]
        time.sleep(1.0 / HZ)
        full = {i: h for i, h in hist.items() if len(h) >= n}
        if len(full) < len(ids):
            continue
        spread = {i: max(h) - min(h) for i, h in full.items()}
        worst = max(spread, key=spread.get)
        if spread[worst] <= STILL_COUNTS:
            return {i: int(statistics.median(h)) for i, h in full.items()}, \
                   "still: worst spread %d counts on id %d" % (spread[worst], worst)
        if time.time() - said > 1.5:
            said = time.time()
            print("hold still - id %d is moving %d counts (%.1f deg), needs %d"
                  % (worst, spread[worst], spread[worst] * 360 / 4096, STILL_COUNTS), flush=True)
    return None, "never still for %.0f s within %.0f s: nothing recorded" % (STILL_S, timeout)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="save the bands and write the calib file")
    ap.add_argument("--said", default="", help="the words of whoever says the robot is in the zero pose")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--calib", default="/home/arduino/robot-control/zero.json")
    a = ap.parse_args()
    bus = fte.Bus(0, "zero_now")
    bus.stopall()
    time.sleep(0.6)
    ids = sorted(set(JOINTS.values()))

    on = []
    for i in ids:
        raw, _ = bus.batch([fte.read_pkt(i, 40, 1)])
        d = bus.payload(raw[0], i, 1)
        if d and d[0]:
            on.append(i)
    if on:
        raise SystemExit("torque is still ON at %s: refusing to call this a zero" % on)

    print("torque off everywhere. Hold the robot in the zero pose and hold STILL for %.0f s." % STILL_S, flush=True)
    zero, note = wait_still(bus, ids, a.timeout)
    if zero is None:
        raise SystemExit(note)
    print(note, flush=True)

    lim = limits()
    rows, usable = [], {}
    for j, sid in sorted(JOINTS.items(), key=lambda kv: kv[1]):
        z = zero[sid]
        half = HEAD_HALF if sid >= 30 else HALF
        saved = lim.get(str(sid)) or {}
        if z > 4095 or z < 0:
            rows.append({"joint": j, "id": sid, "zero": z, "band": None,
                         "why": "reads past the encoder wrap, so no band can be saved (src/limits.rs "
                                "from_ends refuses an end above 4095) and the door will give it no goal. "
                                "Power-cycle the servos, or recentre register 31."})
            continue
        if sid in HOMED and saved.get("min") is not None and saved["min"] <= z <= saved["max"]:
            lo, hi = saved["min"], saved["max"]
            why = "homed band kept: the zero is inside it"
        else:
            lo, hi = max(0, z - half), min(4095, z + half)
            why = "band = zero +/- %d counts (%.0f deg), clipped to one turn" % (half, half * 360 / 4096)
        if hi - lo < 200:
            rows.append({"joint": j, "id": sid, "zero": z, "band": [lo, hi],
                         "why": "only %d counts of band around the zero: too little to walk in" % (hi - lo)})
            continue
        rows.append({"joint": j, "id": sid, "zero": z, "band": [lo, hi], "why": why})
        usable[j] = (sid, z, lo, hi)
    for r in rows:
        print("%-16s id %3d zero %5d (%+7.1f deg from 2048)  band %s  %s"
              % (r["joint"], r["id"], r["zero"], (r["zero"] - CENTRE) * 360 / 4096, r["band"], r["why"]),
              flush=True)
    if not a.write:
        print("\nREAD ONLY. Rerun with --write.")
        return

    out = {}
    for j, (sid, z, lo, hi) in usable.items():
        res = post("/api/limits/save", {"id": sid, "min": lo - INSET, "max": hi + INSET,
                                        "who": "",
                                        "source": "zero_now %s: band around the zero pose the robot was held in "
                                                  "(count %d). Said: %s" % (time.strftime("%F %H:%M"), z, a.said or "-")})
        back = bus.eeprom_limits(sid)
        ok = bool(res.get("ok")) and back == (lo, hi)
        print("id %3d band %s %s" % (sid, back, "OK" if ok else "REFUSED (%s): joint left out" % str(res.get("why"))[:70]),
              flush=True)
        if ok:
            out[j] = {"id": sid, "zero_count": z, "band": list(back)}

    calib = {"what": "the robot zero pose, %s. Official convention count 2048 = 0 rad (model.rs:206 "
                     "counts_to_rad); this file carries the difference between that and where each horn "
                     "actually sits, one number per joint. Nothing was written into any servo." % time.strftime("%F %T %Z"),
             "said": a.said, "counts_per_rad": 4096 / (2 * 3.141592653589793),
             "counts_per_rad_source": "model.rs:206 counts_to_rad(raw) = 2*pi*raw/4096 - pi",
             "stillness": note, "joints": {}}
    for j, v in out.items():
        calib["joints"][j] = {"id": v["id"], "sign": 1, "zero_count": v["zero_count"], "zero_pose": "INIT",
                              "id_source": "servo-bus registry + .34's 21:35 bus read",
                              "zero_source": "zero_now %s, torque off, median of %.0f s still within %d counts. "
                                             "Said: %s" % (time.strftime("%F %H:%M"), STILL_S, STILL_COUNTS, a.said or "-"),
                              "hand_range_counts": v["band"]}
    for j in ("left_hip_yaw", "right_hip_yaw", "left_knee", "left_ankle", "left_hip_roll", "left_hip_pitch",
              "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
              "neck_pitch", "head_pitch", "head_yaw", "head_roll"):
        calib["joints"].setdefault(j, {"id": None, "sign": 1, "zero_count": None, "zero_pose": "INIT",
                                       "id_source": None, "zero_source": None})
    json.dump(calib, open(a.calib, "w"), indent=1)
    print("\nZERO WRITTEN for %d joints: %s\nnot usable: %s\ncalib %s"
          % (len(out), sorted(out), [r["joint"] for r in rows if r["joint"] not in out], a.calib), flush=True)
    print("MAP=" + ",".join("%s=%d" % (j, v["id"]) for j, v in sorted(out.items())), flush=True)


if __name__ == "__main__":
    main()
