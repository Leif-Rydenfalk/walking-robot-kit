#!/usr/bin/env python3
"""fte.py: Feetech STS packets through the bus daemon's raw door, POST /api/batch.

The daemon allows ping, read and sync-read, and writes to RAM 40 to 49 only. It clamps every
goal to that servo's saved limits and caps the torque limit, so nothing here can drive a joint
past what was measured (src/rawgate.rs). Point it somewhere else with SERVO_BUS_URL.
"""
import json
import time
import os
import urllib.request

DAEMON = os.environ.get("SERVO_BUS_URL", "http://127.0.0.1:8938")
REG_MIN, REG_MAX = 9, 11            # EEPROM angle limits (2 bytes each)
REG_TORQUE, REG_GOAL, REG_SPEED, REG_TLIM = 40, 42, 46, 48
REG_TELEM = 56                      # 15 bytes: pos speed load volt temp ... current


def packet(sid, instr, params=b""):
    body = bytes([sid, len(params) + 2, instr]) + bytes(params)
    return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])


def read_pkt(sid, addr, n):
    return packet(sid, 0x02, bytes([addr, n])), 6 + n


def write_pkt(sid, addr, data):
    return packet(sid, 0x03, bytes([addr]) + bytes(data)), 6  # the servo acks a write with a 6-byte status; reading it keeps the next reply aligned    


def u16(v):
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def s16(lo, hi):
    v = lo | hi << 8
    return -(v & 0x7FFF) if v & 0x8000 else v


def load10(lo, hi):
    v = lo | hi << 8
    return -(v & 0x3FF) if v & 0x400 else (v & 0x3FF)


class Bus:
    """The raw door. The FIRST argument is the torque cap the daemon enforces on every reg-48 write.

    cap=0 is what a read-only tool passes (bus_volts.py, the register probes), and it is correct there. What
    it is not is a way to drive a servo gently: with cap 0 the daemon clamps the torque limit to 0, so the
    servo takes the goal, reports torque on, and holds NO force. Nothing moves and nothing complains.
    Measured 2026-10-05 00:1x on id 34: torque read back 1, goal accepted and clamped into its band, reg 48
    read back 0, position unchanged after a second of goals. Ten seconds of a sine on seven servos moved
    every one of them zero counts, and what you see is a robot that does nothing. So a motion write with
    cap 0 is refused by name instead of being silently obeyed.
    """

    # Writing any of these with no torque to back it is the trap above: the goal, the torque enable, and the
    # torque limit itself. Reads, pings and sync-reads are untouched, because cap 0 is right for them.
    MOTION_REGS = (REG_TORQUE, REG_GOAL, REG_TLIM)

    def __init__(self, cap, who):
        self.cap, self.who = cap, who

    def _refuse_capless_motion(self, frames):
        """Raise before the batch leaves, naming the register, when cap 0 would make it a no-op."""
        if self.cap:
            return
        for p, _ in frames:
            # packet(): ff ff id len instr [params...] checksum. instr 0x03 is WRITE, param 0 is the address.
            if len(p) >= 7 and p[4] == 0x03 and p[5] == REG_TORQUE and p[6] == 0:
                continue        # torque OFF is a safety action and is always allowed, cap or no cap
            if len(p) >= 7 and p[4] == 0x03 and p[5] in self.MOTION_REGS:
                raise RuntimeError(
                    "fte.Bus(cap=0) cannot drive: a write to reg %d (%s) on id %d with a torque cap of 0 makes "
                    "the daemon clamp the torque limit to 0, so the servo would take the goal and hold no "
                    "force. Nothing would move and nothing would say so. Pass the cap you mean, e.g. "
                    "fte.Bus(300, %r). cap=0 stays correct for reading." % (
                        p[5], {REG_TORQUE: "torque enable", REG_GOAL: "goal position",
                               REG_TLIM: "torque limit"}[p[5]], p[2], self.who))

    def batch(self, frames, wait_us=3000):
        self._refuse_capless_motion(frames)
        body = json.dumps({"frames": [p.hex() for p, _ in frames], "expects": [min(int(e), 250) for _, e in frames],
                           "wait_us": int(wait_us), "cap": self.cap, "who": self.who}).encode()
        req = urllib.request.Request(DAEMON + "/api/batch", body, {"Content-Type": "application/json"})
        for attempt in range(3):
            try:
                with urllib.request.urlopen(req, timeout=2.0) as r:
                    d = json.load(r)
            except OSError as e:  # timeout while the daemon restarts a wedged MCU app
                d = {"ok": False, "why": "no reply: %r" % (e,)}
            # the MCU router answers EAGAIN now and then (19:50, 19:55): a retry 0.2 s later usually gets through
            # "router timeout: bus_batch did not answer in 416 ms" is the same transient as the others and was
            # NOT retried until 2026-10-05 00:3x, when it killed a stand on its first batch, 10 ms in, right
            # after an MCU app restart. The MCU router has a few calls in flight at a time and answers late
            # while the daemon's own 49 Hz poll is running.
            why = str(d.get("why"))
            transient = any(t in why for t in ("router read", "no reply", "router timeout", "EAGAIN"))
            if d.get("ok") or not transient:
                break
            time.sleep(0.2)
        if not d.get("ok"):
            raise RuntimeError("daemon refused the batch: %s" % d.get("why"))
        return [bytes.fromhex(h) for h in d["replies"]], int(d.get("clamped", 0))

    @staticmethod
    def payload(raw, sid, n):
        """data bytes of a status reply from sid, or None (bad header, wrong id, short, checksum)."""
        i = raw.find(b"\xff\xff" + bytes([sid]))
        if i < 0 or len(raw) < i + 6 + n:
            return None
        r = raw[i:i + 6 + n]
        if (~sum(r[2:-1])) & 0xFF != r[-1]:
            return None
        return r[5:5 + n]

    def telem(self, sid):
        (raw,), _ = self.batch([read_pkt(sid, REG_TELEM, 15)])
        d = self.payload(raw, sid, 15)
        if d is None:
            return None
        return {"pos": s16(d[0], d[1]), "speed": s16(d[2], d[3]), "load": load10(d[4], d[5]), "volt": d[6] / 10,
                "temp": d[7], "moving": d[10], "current": s16(d[13], d[14])}

    def eeprom_limits(self, sid):
        (raw,), _ = self.batch([read_pkt(sid, REG_MIN, 4)])
        d = self.payload(raw, sid, 4)
        return None if d is None else (d[0] | d[1] << 8, d[2] | d[3] << 8)

    def stopall(self):
        req = urllib.request.Request(DAEMON + "/api/cmd", json.dumps({"op": "stopall"}).encode(),
                                     {"Content-Type": "application/json"})
        return urllib.request.urlopen(req, timeout=3).read()


def restart_daemon(serial=os.environ.get("UNOQ_SERIAL")):
    """Restart the daemon over adb.

    Twice now the daemon's own fix, restarting the MCU app, did not bring the bus back, and
    restarting the daemon did. The unit is systemd --user for uid 1000.
    """
    import subprocess
    cmd = "XDG_RUNTIME_DIR=/run/user/1000 systemctl --user restart servo-bus"
    sel = ["-s", serial] if serial else []
    subprocess.run(["adb", *sel, "shell", cmd], timeout=30)
    subprocess.run(["adb", *sel, "forward", "tcp:8938", "tcp:8938"], timeout=10)


def wait_healthy(timeout=90, restart_after=25):
    """wait for the daemon to say the bus answers again (restart the daemon once after restart_after s)."""
    t0 = time.time()
    restarted = False
    t_end = t0 + timeout
    while time.time() < t_end:
        try:
            with urllib.request.urlopen(DAEMON + "/api/bus", timeout=4) as r:
                if json.load(r)["health"]["ok"]:
                    return True
        except Exception:  # noqa: BLE001
            pass
        if not restarted and time.time() - t0 > restart_after:
            restarted = True
            print("bus still down after %d s: restarting the daemon" % restart_after, flush=True)
            restart_daemon()
        time.sleep(2)
    return False
