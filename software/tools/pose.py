#!/usr/bin/env python3
"""pose.py: record a pose you hold by hand, then have the robot go to it by itself.

With no inertial sensor the robot can hold a pose, not balance, and the quickest honest stand
pose is the one you make with your hands. Record it once with torque off, then `go` ramps every
leg servo from where it lies to that pose, slowly, at a capped force, through the daemon's raw
door.

    python3 pose.py record stand         # torque off; waits until every joint is still for 3 s
    python3 pose.py go stand --secs 4 --cap 600 [--hold 20]
    python3 pose.py limp                 # torque off everywhere

`go` logs position, load, voltage and temperature every tick, stops at 50 C or below 4.3 V twice,
and turns torque off at the end unless --hold keeps it holding. Servos with no saved limits are
skipped by name; the daemon gives them no goal.
"""
import argparse
import json
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fte  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEGS = [21, 22, 23, 24, 120, 11, 112, 13, 14, 121]
TICK = 0.1


def telem_all(bus, ids):
    frames = [fte.read_pkt(i, fte.REG_TELEM, 15) for i in ids]
    replies, _ = bus.batch(frames)
    out = {}
    for i, raw in zip(ids, replies):
        d = bus.payload(raw, i, 15)
        if d is not None:
            out[i] = {"pos": fte.s16(d[0], d[1]), "load": fte.load10(d[4], d[5]), "volt": d[6] / 10, "temp": d[7]}
    return out


def record(a):
    bus = fte.Bus(100, "pose")
    bus.stopall()
    time.sleep(0.5)  # a raw batch in the same daemon cycle as STOP ALL is refused
    print("torque off. Hold the robot in the pose; recording when every joint is still for 3 s", flush=True)
    hist = []
    first = telem_all(bus, LEGS)
    posed = False  # the robot lies still before we picks it up: wait until a joint has moved 150+ counts first
    t_end = time.time() + a.timeout
    while time.time() < t_end:
        try:
            tm = telem_all(bus, LEGS)
        except Exception as e:  # noqa: BLE001  we power-cycling the servos drops the bus for a moment
            print("bus: %s; waiting" % e, flush=True)
            fte.wait_healthy(600, restart_after=10 ** 9)  # do not restart the daemon from here
            time.sleep(1)
            continue
        hist.append((time.time(), tm))
        if not posed:
            posed = any(abs(tm[i]["pos"] - first[i]["pos"]) >= 150 for i in tm if i in first)
            if posed:
                print("movement seen: recording when it is still", flush=True)
            time.sleep(TICK)
            continue
        hist = [h for h in hist if h[0] >= time.time() - 3.0]
        if hist[-1][0] - hist[0][0] >= 2.8:
            ids = [i for i in LEGS if all(i in h[1] for h in hist)]
            moved = {i: max(h[1][i]["pos"] for h in hist) - min(h[1][i]["pos"] for h in hist) for i in ids}
            if ids and max(moved.values()) <= 6:
                pose = {str(i): int(statistics.median(h[1][i]["pos"] for h in hist)) for i in ids}
                path = os.path.join(HERE, "evidence", "pose-%s.json" % a.name)
                json.dump({"name": a.name, "t": time.strftime("%Y-%m-%d %H:%M:%S"), "pose": pose,
                           "missing": [i for i in LEGS if i not in ids], "still_counts": moved,
                           "who": ""}, open(path, "w"), indent=1)
                print("saved", path, pose, flush=True)
                return
        time.sleep(TICK)
    print("NOT SAVED: the joints never held still for 3 s within %d s" % a.timeout, flush=True)
    sys.exit(1)


def go(a):
    pose = {int(k): v for k, v in json.load(open(os.path.join(HERE, "evidence", "pose-%s.json" % a.name)))["pose"].items()}
    bus = fte.Bus(a.cap, "pose")
    lim = json.load(open_url("/api/limits"))
    have = lim.get("servos", {})
    ids = []
    for i, target in pose.items():
        row = have.get(str(i)) if isinstance(have, dict) else None
        if not 0 <= target <= 4095:
            print("skip ID %d: pose %d outside 0..4095" % (i, target))
        elif not row:
            print("skip ID %d: no saved limits, the daemon gives it no goal" % i)
        elif not row["min"] <= target <= row["max"]:
            print("skip ID %d: pose %d outside its limits %d..%d" % (i, target, row["min"], row["max"]))
        else:
            ids.append(i)
    now = telem_all(bus, ids)
    # a servo that STARTS outside its limits is past its zero point: driving it to the pose goes the long way round,
    # into the hard stop (ID 13 on 2026-10-05 00:18 read 285 against limits 1690..4061 and sat at load -700)
    for i in [i for i in ids if i in now and not have[str(i)]["min"] <= now[i]["pos"] <= have[str(i)]["max"]]:
        print("skip ID %d: starts at %d, outside its limits %d..%d" % (i, now[i]["pos"], have[str(i)]["min"], have[str(i)]["max"]))
    ids = [i for i in ids if i in now and have[str(i)]["min"] <= now[i]["pos"] <= have[str(i)]["max"]]
    start = {i: now[i]["pos"] for i in ids}
    stamp = time.strftime("%Y%m%d-%H%M%S")
    log = open(os.path.join(HERE, "evidence", "pose-go-%s.jsonl" % stamp), "w")
    # goal = where it is, then torque on, so nothing jumps
    bus.batch([fte.write_pkt(i, fte.REG_GOAL, fte.u16(start[i])) for i in ids])
    bus.batch([fte.write_pkt(i, fte.REG_TORQUE, [1]) for i in ids])
    low_v = 0
    fails = 0
    prog, sagging = 0.0, False
    t0 = last_t = time.time()
    try:
        while True:
            #
            # Progress advances by real time only while the supply holds; under 5.0 V it advances at a quarter speed.
            now_t = time.time()
            prog += (now_t - last_t) * (0.25 if sagging else 1.0)
            last_t = now_t
            f = min(1.0, prog / a.secs)
            s = f * f * (3 - 2 * f)  # smoothstep: slow start, slow finish
            goals = {i: int(round(start[i] + s * (pose[i] - start[i]))) for i in ids}
            frames = [fte.write_pkt(i, fte.REG_GOAL, fte.u16(g)) for i, g in goals.items()]
            try:
                bus.batch(frames)
                tm = telem_all(bus, ids)
                fails = 0
            except RuntimeError as e:
                # one router timeout or 504 is a hiccup, not a reason to drop the robot (00:18, 00:23, 00:33 runs all died
                # on a single one). STOP ALL from someone else, or 5 misses in a row (0.5 s), still stops.
                fails += 1
                if "STOP ALL" in str(e) or fails >= 5:
                    raise
                print("bus hiccup %d: %s" % (fails, e), flush=True)
                time.sleep(TICK)
                continue
            log.write(json.dumps({"t": round(time.time(), 3), "f": round(f, 3), "goals": goals, "tm": tm}) + "\n")
            log.flush()
            hot = [i for i, v in tm.items() if v["temp"] >= 50]
            if hot:
                raise RuntimeError("hot: %s" % {i: tm[i]["temp"] for i in hot})
            vmin = min(v["volt"] for v in tm.values()) if tm else 0
            if vmin < 5.0 and not sagging:
                print("supply %.1f V: slowing down, not stopping" % vmin, flush=True)
            sagging = vmin < 5.0
            if f >= 1.0 and time.time() - t0 >= a.secs + a.hold:
                break
            time.sleep(TICK)
        err = {i: tm[i]["pos"] - pose[i] for i in tm}
        print("reached: error per joint (counts) %s" % err, flush=True)
    finally:
        bus.stopall()
        print("torque off", flush=True)


def open_url(path):
    import urllib.request
    return urllib.request.urlopen(fte.DAEMON + path, timeout=5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["record", "go", "limp"])
    ap.add_argument("name", nargs="?", default="stand")
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--cap", type=int, default=600)
    ap.add_argument("--hold", type=float, default=15.0)
    ap.add_argument("--timeout", type=float, default=120.0)
    a = ap.parse_args()
    if a.cmd == "limp":
        print(fte.Bus(100, "pose").stopall())
    elif a.cmd == "record":
        record(a)
    else:
        go(a)


if __name__ == "__main__":
    main()
