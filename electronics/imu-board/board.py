#!/usr/bin/env python3
"""board.py - the the robot trunk IMU, two layers, 18 x 15 mm.

    ce-pcb/bin/pcb electronics/trunk-imu/board.py
    ce-pcb/bin/pcb electronics/trunk-imu/board.py --place-only

The parts and nets are trunk_parts.py; this file adds only copper.

  F.Cu  the four parts, the signals, and a GND fill around them
  B.Cu  solid GND, the reference under the LGA and the glue face

Rules are JLCPCB 2-layer standard (0.09 mm min track and space, 0.15 mm min
drill); 0.127 mm keeps the LGA-14's 0.5 mm pitch pads and the JST-SH's 1.0 mm
pitch pads legal with room to spare.
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
WS = os.path.dirname(os.path.dirname(REPO))
CEPCB = os.path.join(WS, "ce-pcb")
OUT = os.path.join(HERE, "out")
NAME = "trunk_imu"
sys.path.insert(0, CEPCB)
sys.path.insert(0, HERE)

from cepcb import Board                                       # noqa: E402
from cepcb.routeplan import RoutePlan, Class                  # noqa: E402
from cepcb import meter                                       # noqa: E402
from cepcb.silk import place_refs, fab_refs                   # noqa: E402
import trunk_parts as P                                       # noqa: E402


def net_all(b, name, *pins):
    """Every pad with the number, not the first."""
    b.net(name)
    n = b._nets[name]
    for pin in pins:
        ref, _, num = str(pin).partition(".")
        fp = b.component(ref)
        found = 0
        for pad in fp.Pads():
            if pad.GetNumber() != num:
                continue
            was = pad.GetNetname()
            if was and was != name:
                raise SystemExit("%s already on %r, not %r" % (pin, was, name))
            pad.SetNet(n)
            found += 1
        if not found:
            have = ", ".join(sorted({p.GetNumber() for p in fp.Pads()}))
            raise SystemExit("%s has no pad %r. have: %s" % (ref, num, have))


def build(place_only=False):
    os.makedirs(OUT, exist_ok=True)
    t0 = time.time()
    b = Board(NAME, P.W, P.H, layers=2, title="the robot trunk IMU rev A")
    b.thickness(1.6, why="JLCPCB 2-layer default")
    b.rules(clearance=0.127, track=0.127, via=0.45, via_drill=0.25,
            min_via=0.40, min_hole=0.20,
            why="JLCPCB 2-layer standard; 0.127 mm clears the LGA-14's 0.5 mm "
                "pitch pads. 0.45/0.25 is the via everything uses; the floor "
                "is 0.40/0.20 because the two trapped pads in the middle of "
                "the LGA's rows need two vias inside the part and 0.45 ones "
                "cannot clear each other there. JLCPCB's published minimum is "
                "0.25/0.15 (jlcpcb.com/capabilities/pcb-capabilities, read "
                "2026-10-04)")
    b.board_rules("the LGA-14 and JST-SH pads take one spoke",
                  min_resolved_spokes=1)

    for ref, sym, fp, val, lcsc, side, at, rot, fitted, note in P.PARTS:
        b.place(ref, fp, at=at, rot=rot, side=side, value=val, note=note)
        f = b.component(ref)
        if lcsc:
            f.SetField("LCSC Part #", lcsc)
            f.GetField("LCSC Part #").SetVisible(False)
        if not fitted:
            f.SetDNP(True)
            f.SetExcludedFromBOM(True)
            f.SetExcludedFromPosFiles(True)
        if ref.startswith(("TP", "H")):
            f.SetExcludedFromBOM(True)
            f.SetExcludedFromPosFiles(True)
    for net, pins in P.NETS.items():
        net_all(b, net, *pins)

    b.netclass("RAIL", ["+3V3"], 0.127, 0.3, why="under 1 mA: one IMU")
    b.netclass("BUS", ["SDA", "SCL"], 0.127, 0.25,
               why="400 kHz I2C over a 4-wire cable")

    path = os.path.join(OUT, NAME + ".kicad_pcb")
    if place_only:
        b.save(path)
        print("placed", len(P.PARTS), "->", path)
        return

    b.pour("GND", "B.Cu", why="solid ground, the reference under the LGA")
    b.pour("GND", "F.Cu", why="ground fill between the parts")

    plan = RoutePlan(b, grid=0.1, safety=0.03, edge_clearance=0.4)
    try:
        plan.fanout("U1", width=0.15, length=0.3, skip_nets=("GND",),
                    why="0.5 mm pitch pads need a comb before the grid router")
    except Exception as e:                                    # noqa: BLE001
        print("fanout U1 skipped:", e)
    # Pad 6 is the middle of a three-pad row on the LGA-14's border, so it
    # has no outward escape and KiCad left it off the ground plane. The hat
    # has the same part and the same problem; this is the same answer, one
    # via in the empty middle of the footprint.
    inward = plan.fanout_inward("U1", ["U1.2", "U1.3", "U1.6", "U1.7", "U1.13"],
                                size=0.40, drill=0.20,
                                why="the middle pad of a 0.5 mm pitch LGA row "
                                    "cannot escape sideways, and both rows here "
                                    "have one")
    print("  inward via:", {k: (round(v[0], 2), round(v[1], 2))
                            for k, v in inward.items()})
    plan.add(Class("RAIL", ["+3V3"], 0.3, why="supply"))
    plan.add(Class("BUS", ["SDA", "SCL"], 0.25, why="I2C"))
    plan.add(Class("DIGITAL", "*", 0.15, why="every remaining signal"))
    bonded = plan.bond("GND", max_mm=1.5, width=0.4,
                       why="every GND pad gets its own via into the plane")
    stitched = plan.stitch("GND", [(1.0, 1.0, 17.0, 14.0)], pitch=4.0,
                           why="tie the F.Cu fill to B.Cu")
    rows = plan.run(skip=["GND"])
    plan.finish(width=0.127, why="last pass at the rule minimum", skip=["GND"])
    pruned = plan.prune_dangling(why="stubs that end in their own track")
    isl = [plan.stitch_islands("GND", lay,
                               why="an unstitched ground island is not ground")
           for lay in ("F.Cu", "B.Cu")]

    b.tidy_silkscreen(ref_mm=1.0, values_to_fab=True)
    placed, hidden = place_refs(b, size=0.8, thickness=0.12, max_mm=5.0)
    for ref in ("H1", "H2"):
        b.component(ref).Reference().SetVisible(False)
    fab_refs(b, size=0.6, thickness=0.1)
    b.fill_zones()
    b.save(path)

    drc, err = meter.run_drc(path, out_json=os.path.join(OUT, "drc.json"),
                             refill=True, save=True)
    summary = {
        "name": NAME, "layers": 2, "parts": len(P.PARTS), "nets": len(P.NETS),
        "routed_rows": len(rows), "pruned": pruned, "stitch": stitched,
        "bond": [len(bonded[0]), bonded[1]], "islands": isl,
        "open_joins": plan.open_joins(), "silk_refs_placed": placed,
        "silk_refs_hidden": hidden, "drc": drc, "drc_error": err,
        "seconds": round(time.time() - t0, 1),
    }
    json.dump(summary, open(os.path.join(OUT, "build-summary.json"), "w"),
              indent=1, sort_keys=True, default=str)
    print(json.dumps(summary, indent=1, sort_keys=True, default=str))


if __name__ == "__main__":
    build(place_only="--place-only" in sys.argv)
