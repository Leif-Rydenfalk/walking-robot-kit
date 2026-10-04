#!/usr/bin/env python3
"""sch.py - the UNO Q head hat schematic, drawn from hat_parts.py.

    ce-pcb/bin/sch all electronics/unoq-hat/sch.py -o electronics/unoq-hat/out
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "ce-pcb"))
import hat_parts as H                                   # noqa: E402
from cepcb.schematic import Schematic                   # noqa: E402

GROUP = {
    "J1": "uno q", "J2": "uno q", "H1": "uno q", "H2": "uno q", "H3": "uno q", "H4": "uno q",
    "J3": "camera", "U1": "camera", "U2": "camera", "R1": "camera", "R2": "camera",
    "C1": "camera", "C2": "camera", "C3": "camera", "C4": "camera", "C5": "camera",
    "TP1": "camera", "TP6": "camera",
}
for r in ("J4", "R3", "R4", "R5", "R6", "R7", "R8"):
    GROUP[r] = "tof"
for r in ("J5", "U5", "U6", "Q1", "JP1", "R15", "R16", "R17", "R18", "R19", "R20",
          "R21", "D1", "C20", "C21", "TP4"):
    GROUP[r] = "servo bus"
for r in ("J6", "Q2", "R11", "F1", "U3", "C6", "C7", "C8", "C9", "C10", "R12", "R13",
          "L1", "C11", "C12", "TP2", "TP3", "TP5"):
    GROUP[r] = "power"
for r in ("U4", "R9", "C13", "C14", "C15", "C16", "J7", "J8", "R14", "C19"):
    GROUP[r] = "audio"
for r in ("U7", "C22", "C23", "R27", "R28", "J12"):
    GROUP[r] = "imu"
for r in ("U8", "R22", "C26", "R29"):
    GROUP[r] = "pack monitor"
for r in ("U9", "J9", "R24", "C24", "C25", "J10", "J11", "R25", "R26"):
    GROUP[r] = "touch and feet"

s = Schematic("unoq_hat", title="the robot UNO Q head hat", rev="B",
              company="this project / the robot")
# symbols KiCad 10 does not have, each with the datasheet its pinout was read from
for name, spec in H.LOCAL_SYMBOLS.items():
    spec = dict(spec)
    s.define(name, spec.pop("pins"), **spec)
for ref, sym, fp, val, lcsc, side, at, rot, fitted, note in H.PARTS:
    nobom = ref.startswith(("TP", "H", "JP"))
    fields = {"LCSC Part #": lcsc} if lcsc else {}
    s.part(ref, sym, value=val, footprint=fp, group=GROUP.get(ref, "misc"),
           dnp=not fitted, in_bom=not nobom, fields=fields, note=note)
s.power("GND", "+3V3", "+1V8", "V5", "VBAT", "VBAT_IN", "VBAT_FET", "VBAT_F")
for net, pins in H.NETS.items():
    s.net(net, *pins)
s.nc(*H.NC)
unused = s.nc_unused()
print("no-connect, unused UNO Q signals (%d):" % len(unused))
print("  " + ", ".join(k for k, _ in unused))
s.text("UNO Q head hat rev B. Board-to-board pinout and the camera block copied from "
       "Arduino's UNO Media Carrier ASX00083 so arduino-linux-config camera0=type1-2lanes "
       "applies. 2S pack -> 5 mOhm shunt -> AON6407 -> PTC -> TPS62933 5 V -> JMISC 5V_SYS. "
       "One 3.3 V I2C bus (CCI0) carries camera 0x10, expander 0x26, ToF 0x29, "
       "INA226 0x40, head IMU 0x6B and, on J12, the trunk IMU 0x6A.")
s.place()
out = sys.argv[sys.argv.index("-o") + 1] if "-o" in sys.argv else os.path.join(HERE, "out")
os.makedirs(out, exist_ok=True)
s.save(os.path.join(out, "unoq_hat.kicad_sch"))
