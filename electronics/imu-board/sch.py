#!/usr/bin/env python3
"""sch.py - the the robot trunk IMU schematic, drawn from trunk_parts.py.

    ce-pcb/bin/sch all electronics/trunk-imu/sch.py -o electronics/trunk-imu/out
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "ce-pcb"))
import trunk_parts as H                                 # noqa: E402
from cepcb.schematic import Schematic                   # noqa: E402

s = Schematic("trunk_imu", title="the robot trunk IMU", rev="A",
              company="this project / the robot")
for name, spec in H.LOCAL_SYMBOLS.items():
    spec = dict(spec)
    s.define(name, spec.pop("pins"), **spec)
for ref, sym, fp, val, lcsc, side, at, rot, fitted, note in H.PARTS:
    nobom = ref.startswith(("TP", "H"))
    fields = {"LCSC Part #": lcsc} if lcsc else {}
    s.part(ref, sym, value=val, footprint=fp, group="imu",
           dnp=not fitted, in_bom=not nobom, fields=fields, note=note)
s.power("GND", "+3V3")
for net, pins in H.NETS.items():
    s.net(net, *pins)
s.nc(*H.NC)
s.text("Body IMU, rev A. The second of the two inertial sensors: "
       "the hat's LSM6DSV16X is the head, this one is the body. SA0 to GND "
       "makes it 0x6A so both sit on the hat's 3.3 V CCI0 bus, which reaches "
       "here on a 4-wire JST-SH cable from the hat's J12.")
s.place()
out = sys.argv[sys.argv.index("-o") + 1] if "-o" in sys.argv else os.path.join(HERE, "out")
os.makedirs(out, exist_ok=True)
s.save(os.path.join(out, "trunk_imu.kicad_sch"))
