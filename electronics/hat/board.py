#!/usr/bin/env python3
"""board.py - the the robot UNO Q head hat, rev B, four layers.

    ce-pcb/bin/pcb electronics/unoq-hat/board.py              build, route, DRC, fab
    ce-pcb/bin/pcb electronics/unoq-hat/board.py --place-only  parts only, for a look

The parts and nets are hat_parts.py; this file adds only copper.

WHAT IS ARDUINO'S AND WHAT IS OURS
  The two 2x30 board-to-board headers, the 22-pin camera FFC and every CSI0
  track and via between them are copied from Arduino's UNO Media Carrier
  (ASX00083, Altium files imported to ref/mediacarrier.kicad_pcb). The carrier
  frame shifted by (-114.211, -78.334) IS this board's KiCad frame, so the
  copy is a translation and nothing else: ten CSI0 nets, 0.18 mm, length
  matched by Arduino to 52.65-53.32 mm, one via per net.
  Everything else (CCI level shifter, expander, ToF, servo bus, buck, audio)
  is placed on the BOTTOM so the only parts under the UNO Q are the headers.

THE FOUR LAYERS
  F.Cu   header pads, CSI0 (Arduino's), signals; GND fill
  In1.Cu solid GND, the reference under every CSI0 track
  In2.Cu GND west of x 33 (under CSI0 on B.Cu), V5 plane south-east,
         VBAT plane north-east
  B.Cu   parts, CSI0 (Arduino's), signals; GND fill
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
FAB = os.path.join(OUT, "fab")
NAME = "unoq_hat"
sys.path.insert(0, CEPCB)
sys.path.insert(0, HERE)

import pcbnew                                                 # noqa: E402
from cepcb import Board, register_library                     # noqa: E402
from cepcb.routeplan import RoutePlan, Class                  # noqa: E402
from cepcb import meter                                       # noqa: E402
from cepcb.silk import place_refs, fab_refs                   # noqa: E402
import hat_parts as P                                         # noqa: E402

register_library("unoq_hat", os.path.join(HERE, "footprints", "unoq_hat.pretty"))
MM = 1e6

# UNO outline, from the carrier's Edge.Cuts (right-edge chamfers), y up
OUTLINE = [(0.0, 0.0), (66.04, 0.0), (66.04, 2.54), (68.58, 5.08),
           (68.58, 37.846), (66.04, 40.386), (66.04, 51.816), (64.516, 53.34),
           (0.0, 53.34)]

# pad 1 of each Arduino-positioned part, in this frame (y up), read from the
# carrier: JMEDIA pad 1 (128.0301, 86.5886), JMISC (174.8931, 86.5886),
# FFC (126.65108, 125.2786)
PIN1 = {"J1": (13.8191, 45.0854), "J2": (60.6821, 45.0854), "J3": (12.44008, 6.3954)}
PIN2_DX = {"J1": (-4.27, 0.0), "J2": (-4.27, 0.0), "J3": (0.5, 0.0)}


def place_exact(b, ref, fpid, at, side, value):
    """Place so pad 1 and pad 2 land where Arduino's did; try all four turns."""
    for rot in (0, 90, 180, 270, -90):
        fp = b.place(ref, fpid, at=at, rot=rot, side=side, anchor="origin", value=value)
        p1 = b.pad_xy(ref + ".1")
        dx, dy = PIN1[ref][0] - p1[0], PIN1[ref][1] - p1[1]
        pos = fp.GetPosition()
        fp.SetPosition(pcbnew.VECTOR2I(int(pos.x + dx * MM), int(pos.y - dy * MM)))
        p1, p2 = b.pad_xy(ref + ".1"), b.pad_xy(ref + ".2")
        ex = PIN2_DX[ref]
        if abs(p2[0] - p1[0] - ex[0]) < 0.01 and abs(p2[1] - p1[1] - ex[1]) < 0.01:
            return rot
        b._pcb.Remove(fp)
        del b.refs[ref]
        del b.fpids[ref]
    raise SystemExit("%s: no rotation puts pad 2 at %s from pad 1" % (ref, PIN2_DX[ref]))


def net_all(b, name, *pins):
    """Every pad with the number, not the first (PowerPAK has five pads '5')."""
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


def copy_csi0(b):
    """Arduino's CSI0 copper, translated. Returns (segments, arcs, vias)."""
    d = json.load(open(os.path.join(OUT, "carrier-csi0.json")))
    lay = {"TOP": pcbnew.F_Cu, "BOTTOM": pcbnew.B_Cu}
    ns = na = 0

    def v(p):
        return pcbnew.VECTOR2I(int(round(p[0] * MM)), int(round(p[1] * MM)))
    for s in d["segs"]:
        # Altium's export carries zero-length pieces where an arc was split; KiCad
        # reports each as track_dangling and three as tracks_crossing errors
        # (measured 2026-10-03). They carry no copper, so they are not copied.
        if v(s["a"]) == v(s["b"]):
            continue
        if s["arc"]:
            t = pcbnew.PCB_ARC(b._pcb)
            t.SetStart(v(s["a"]))
            t.SetMid(v(s["mid"]))
            t.SetEnd(v(s["b"]))
            na += 1
        else:
            t = pcbnew.PCB_TRACK(b._pcb)
            t.SetStart(v(s["a"]))
            t.SetEnd(v(s["b"]))
            ns += 1
        t.SetWidth(int(round(s["w"] * MM)))
        t.SetLayer(lay[s["layer"]])
        t.SetNet(b._nets[s["net"]])
        b._pcb.Add(t)
    for x in d["vias"]:
        t = pcbnew.PCB_VIA(b._pcb)
        t.SetPosition(v(x["at"]))
        t.SetDrill(int(round(x["d"] * MM)))
        t.SetWidth(int(round(x["w"] * MM)))
        t.SetNet(b._nets[x["net"]])
        b._pcb.Add(t)
    return ns, na, len(d["vias"])


T0 = time.time()


def stage(name):
    with open(os.path.join(OUT, "stages.log"), "a") as f:
        f.write("%7.1f s  %s\n" % (time.time() - T0, name))


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def pad_half(b, pin):
    """(half width, half height) of a pad's bounding box, mm."""
    ref, num = pin.split(".")
    bb = b.component(ref).FindPadByNumber(num).GetBoundingBox()
    return bb.GetWidth() / 2e6, bb.GetHeight() / 2e6


def pinch_copper(b):
    """Copper for the three pads the router could not reach, laid BEFORE
    fanout() and run() so both route around it. Every point is derived from
    pad positions; nothing is typed.

    Measured 2026-10-03 after the fanout fix, DRC on
    out/unoq_hat.kicad_pcb: U1.2 (GND) and U1.3 (+1V8) unconnected, J3.19 (GND)
    unconnected, and bond() found no legal via for J3.19 even at 3 mm.

    U1 (VSSOP-8, 0.5 mm) sits 0.6 mm east of C1/C2/R1. Pin 3 is +1V8 between
    GND pin 2 and SDA_1V8 pin 4. The gap between C1's east pad edge and U1's
    west pad edge is one 0.15 mm track wide, so:
      +1V8: pin 3 west into that channel, north up it, onto R1.2 (+1V8).
      GND:  pin 2 west to a via just outside the pads, into the In1/In2 plane.
    J3.19 is the FFC's GND between CAM_IO1 and CCI_SCL, every neighbour a
    signal: a via 0.6 mm off the pad's toe (the side the CSI pairs leave by)
    puts it straight into In1 GND."""
    made = []
    hw, _hh = pad_half(b, "U1.3")
    p3, p2 = b.pad_xy("U1.3"), b.pad_xy("U1.2")
    u1_west = p3[0] - hw
    c1_east = b.pad_xy("C1.2")[0] + pad_half(b, "C1.2")[0]
    xch = round((u1_west + c1_east) / 2.0, 3)
    r12 = b.pad_xy("R1.2")
    r_hh = pad_half(b, "R1.2")[1]
    b.track("+1V8", [p3, (xch, p3[1]), (xch, r12[1] - r_hh - 0.25), r12],
            width=0.15, layer="B.Cu")
    made.append("+1V8 U1.3->R1.2 via x %.3f" % xch)
    gv = (round(u1_west - 0.425, 3), p2[1])
    b.track("GND", [p2, gv], width=0.15, layer="B.Cu")
    b.via("GND", gv, drill=0.25, size=0.45)
    made.append("GND U1.2 via %s" % (gv,))
    j = b.pad_xy("J3.19")
    jh = pad_half(b, "J3.19")[1]
    # the toe is the side away from the connector body
    body = b.component("J3").GetPosition()
    body_y = b._y(body.y / 1e6)
    sgn = 1.0 if j[1] > body_y else -1.0
    jv = (j[0], round(j[1] + sgn * (jh + 0.6), 3))
    b.track("GND", [j, jv], width=0.2, layer="B.Cu")
    b.via("GND", jv, drill=0.25, size=0.45)
    made.append("GND J3.19 via %s" % (jv,))
    return made


def build(place_only=False, do_fab=True):
    os.makedirs(OUT, exist_ok=True)
    t0 = time.time()
    b = Board(NAME, outline=OUTLINE, layers=4, title="the robot UNO Q head hat rev B")
    b.thickness(1.6, why="JLCPCB 4-layer default and the carrier's 1.606 mm")
    b.rules(clearance=0.127, track=0.127, via=0.45, via_drill=0.25,
            min_via=0.40, min_hole=0.20,
            why="JLCPCB 4-layer standard: 0.09 mm min track/space, 0.15 mm min "
                "drill; 0.127 mm keeps Arduino's 0.18 mm CSI0 pairs and the "
                "0.5 mm pitch FFC/SOT-583 pads legal. 0.45/0.25 is the via "
                "everything uses; the floor is 0.40/0.20 because the LSM6DSV16X "
                "needs two vias inside an LGA-14 and 0.45 ones cannot clear "
                "each other there. JLCPCB's own multilayer minimum is "
                "0.25/0.15 (jlcpcb.com/capabilities/pcb-capabilities, "
                "'Min. Via hole size/diameter', read 2026-10-04), so 0.40/0.20 "
                "is three steps inside what the fab will build")
    b.board_rules("Arduino's 1.27 mm header and FFC pads take one spoke",
                  min_resolved_spokes=1)

    rots = {}
    for ref, sym, fp, val, lcsc, side, at, rot, fitted, note in P.PARTS:
        if ref in PIN1:
            rots[ref] = place_exact(b, ref, fp, at, side, val)
        else:
            b.place(ref, fp, at=at, rot=rot, side=side, value=val, note=note)
        f = b.component(ref)
        if lcsc:
            f.SetField("LCSC Part #", lcsc)
            # SetField makes a VISIBLE field on the part's silk layer: 56 LCSC codes
            # would print on B.Silkscreen, unmirrored (DRC 2026-10-03). Data only.
            f.GetField("LCSC Part #").SetVisible(False)
        if not fitted:
            f.SetDNP(True)
            f.SetExcludedFromBOM(True)
            f.SetExcludedFromPosFiles(True)
        if ref.startswith(("TP", "H", "JP")):
            f.SetExcludedFromBOM(True)
            f.SetExcludedFromPosFiles(True)
    for net, pins in P.NETS.items():
        net_all(b, net, *pins)

    # net classes
    b.netclass("CSI", [n for n in P.NETS if n.startswith("CSI0_")], 0.127, 0.18,
               why="Arduino's own width, kept with Arduino's copper")
    b.netclass("POWER", ["V5", "VBAT", "VBAT_IN", "VBAT_FET", "VBAT_F", "SW"], 0.2, 0.8,
               why="3 A at 5 V and the servo feed; planes carry the length, "
                   "0.8 mm is the neck between pad and plane")
    b.netclass("RAIL", ["+3V3", "+1V8"], 0.127, 0.3,
               why="under 400 mA: camera 250 mA, ToF 100 mA, logic")
    b.netclass("BUS", ["SERVO_DATA", "DATA_BUF", "SPK_P", "SPK_N"], 0.15, 0.3,
               why="servo bus leaves the board; speaker 0.65 A peak")

    csi = copy_csi0(b) if not place_only else (0, 0, 0)
    path = os.path.join(OUT, NAME + ".kicad_pcb")
    if place_only:
        b.save(path)
        print("placed", len(P.PARTS), "rotations", rots, "->", path)
        return

    # planes
    b.pour("GND", "In1.Cu", why="solid ground under everything; CSI0 reference")
    zg = b.pour("GND", "In2.Cu", why="ground on In2 wherever no power plane is")
    zv = b.pour("V5", "In2.Cu", outline=rect(33.5, 6.0, 66.0, 33.4),
                why="5 V from L1/C11/C12 to JMISC 54/56 and the amplifier")
    # rev B: the west step moved from x 33.5 to x 30.5 so the plane reaches
    # under R22's low-side pad, which is where the pack current now enters it.
    # In2 is GND wherever this is not, and In1 is solid GND, so the 3 mm the
    # plane takes costs no return path.
    zb = b.pour("VBAT", "In2.Cu", outline=[(24.0, 40.0), (30.5, 40.0), (30.5, 34.4),
                                          (66.0, 34.4), (66.0, 40.0), (65.0, 41.0),
                                          (65.0, 51.0), (24.0, 51.0)],
                why="pack after the reverse FET and the sense shunt: servo "
                    "connector and PTC")
    zv.SetAssignedPriority(2)
    zb.SetAssignedPriority(2)
    zg.SetAssignedPriority(0)
    # local power copper, each around its own pads, with via clusters into
    # the In2 planes (one bonded via is ~1 A on JLC's 0.5 oz inner copper)
    local = [
        ("VBAT_IN", "B.Cu", [(43.6, 34.0), (48.6, 34.0), (48.6, 41.8), (44.2, 41.8),
                             (44.2, 46.8), (39.8, 46.8), (39.8, 41.0), (43.6, 41.0)],
         "pack pins 1-2 to Q2 drain", []),
        ("VBAT_F", "B.Cu", [(54.4, 26.0), (60.0, 26.0), (60.0, 37.6), (52.2, 37.6),
                            (52.2, 34.6), (54.4, 34.6)],
         "PTC to TPS62933 VIN and its capacitors", []),
        # rev B: Q2's source no longer goes straight to the plane. It goes to
        # the shunt's high side (R22 pad 2 at x 38.96, source pins at x 41.83,
        # a 2.9 mm hop), and the plane is entered on the LOW side instead, so
        # every amp the pack delivers is measured. No vias here: VBAT_FET is
        # this pour and nothing else.
        # Measured with tools/placecheck.py --board out/unoq_hat.kicad_pcb
        # --pads Q2,R22, not typed. Q2's three source pads are x 41.390-42.660,
        # y 34.290-37.440; R22's high-side pad is x 38.350-39.575,
        # y 34.025-37.375; the GATE is the same column at y 38.100-38.710, so
        # this stops at 37.6 and leaves it 0.5 mm of air.
        ("VBAT_FET", "B.Cu", rect(38.30, 33.90, 42.90, 37.60),
         "Q2 source pins to the sense shunt's high side", []),
        # R22's low-side pad measures x 32.425-33.650, y 34.025-37.375; the
        # nearest foreign copper is U8's pin row at y 38.85, 1.25 mm clear.
        ("VBAT", "B.Cu", rect(30.9, 33.8, 34.4, 37.6),
         "shunt low side into the VBAT plane. Five vias is about 5 A on JLC's "
         "0.5 oz inner copper; bond() adds one more at every other VBAT pad",
         [(31.5, 34.6), (31.5, 35.2), (31.5, 35.8), (31.5, 36.4), (31.5, 37.0)]),
        ("VBAT", "B.Cu", rect(51.2, 40.0, 54.8, 42.6), "PTC input into the VBAT plane",
         [(52.0, 42.0), (53.0, 42.0), (54.0, 42.0)]),
        ("VBAT", "B.Cu", rect(29.3, 39.5, 30.7, 46.6), "servo connector VBAT into the plane",
         [(30.0, 40.2), (30.0, 41.2), (30.0, 42.2)]),
        ("V5", "B.Cu", rect(37.0, 22.0, 43.9, 32.5), "L1 output and C11/C12 into the V5 plane",
         [(41.0, 24.5), (41.0, 25.5), (41.0, 30.0), (41.0, 31.0)]),
        ("V5", "F.Cu", rect(56.5, 10.2, 59.2, 12.7), "JMISC 54/56 5V_SYS into the V5 plane",
         [(58.6, 10.5), (58.6, 11.4), (58.6, 12.3)]),
    ]
    for net, layer, poly, why, vias in local:
        z = b.pour(net, layer, outline=poly, why=why)
        z.SetAssignedPriority(3)
        for at in vias:
            b.via(net, at, drill=0.3, size=0.6)
    b.pour("GND", "B.Cu", why="ground fill on the part side")
    b.pour("GND", "F.Cu", why="ground fill under the UNO Q")

    stage('planes poured')
    hand = pinch_copper(b)
    stage('pinch copper: %s' % hand)
    plan = RoutePlan(b, grid=0.1, safety=0.03, edge_clearance=0.55)
    for ref in ("U3", "U1", "J3", "U7", "U8"):
        try:
            plan.fanout(ref, width=0.15, length=0.3,
                        skip_nets=("GND", "V5", "VBAT") + tuple(n for n in P.NETS if n.startswith("CSI0_")),
                        why="0.5 mm pitch pads need a comb before the grid router")
        except Exception as e:                                  # noqa: BLE001
            print("fanout", ref, "skipped:", e)
    # The LSM6DSV16X is an LGA-14 with pads on all four edges and nothing in
    # the middle. Four of its pads sit in the MIDDLE of a 0.5 mm pitch row,
    # so they cannot escape outward at all: builds on 2026-10-04 left pad 13
    # with "search budget exhausted" on every attempt and pads 3, 6 and 7
    # unconnected in the DRC. They go out through the empty middle instead.
    # 0.40/0.20 rather than the board's 0.45/0.25: two vias at 0.45 need
    # their centres 0.60 mm apart to satisfy KiCad's hole-to-copper rule and
    # the LGA-14's middle gives 0.597. 0.20 mm is JLCPCB's minimum via hole on
    # 4 layers and 0.40 mm leaves their minimum 0.10 mm annular ring.
    inward = plan.fanout_inward("U7", ["U7.13", "U7.2", "U7.3", "U7.6", "U7.7"],
                                size=0.40, drill=0.20,
                                why="pads in the middle of a 0.5 mm pitch LGA row "
                                    "have no outward escape")
    stage('U7 inward fanout: %s' % {k: (round(v[0], 2), round(v[1], 2))
                                    for k, v in inward.items()})
    plan.add(Class("POWER", ["VBAT_IN", "VBAT_FET", "VBAT_F", "SW"], 0.8,
                   why="pack feed, converter input and switch node; pours reinforce"))
    plan.add(Class("RAIL", ["+3V3", "+1V8"], 0.3, why="camera, ToF and logic supply"))
    plan.add(Class("BUS", ["SERVO_DATA", "DATA_BUF", "SPK_P", "SPK_N", "BST"], 0.3,
                   why="servo bus and speaker"))
    plan.add(Class("FIRST", ["CAM_IO1", "TOF_INT", "IMU_INT1", "INA_ALERT"], 0.15,
                   why="signals that lose the race for a pin row: J3.18 and J2.14 on "
                       "2026-10-03, and on 2026-10-04 the IMU and pack-monitor "
                       "interrupts, both of which start on a 0.5 mm pitch pad and "
                       "end on a J2 pin the 36 DIGITAL nets fill first; and the "
                       "interrupts, which start on a 0.5 mm pitch pad"))
    plan.add(Class("EARLY", ["MCU_SCL", "MCU_SDA"], 0.15,
                   why="R5 and R6 reach J2.16 and J2.18 and lost that pin row to "
                       "the DIGITAL crowd twice. Routed after FIRST, not with it: "
                       "putting them in FIRST took the row INA_ALERT needs, which "
                       "is what a shared pin row does when two nets both go early"))
    plan.add(Class("DIGITAL", "*", 0.15, why="every remaining signal"))
    stage('fanout + classes')
    bonded = {}
    for net in ("GND", "V5", "VBAT"):
        bonded[net] = plan.bond(net, max_mm=1.5, width=0.4,
                                why="every pad on a plane net gets its own via into the plane")
    stage('bond done')
    stitched = plan.stitch("GND", [(1.0, 1.0, 12.0, 52.0), (28.0, 1.0, 66.0, 52.0)],
                           pitch=5.0, why="tie the outer GND fills to In1/In2")
    skip = ["GND", "V5", "VBAT"] + \
        [n for n in P.NETS if n.startswith("CSI0_")]
    stage('stitch done')
    rows = plan.run(skip=skip)
    oj = plan.open_joins()
    stage('run done, %d open joins: %s' % (len(oj), sorted({n for n, _a, _b in oj})))
    b.save(path)                       # checkpoint: finish() can take long
    stage('checkpoint saved')
    # repair() ran 25+ min without finishing on 2026-10-03 00:04-00:29 with the Mac
    # swapping (8.1/9.2 GB); HAT_REPAIR=1 turns it back on when there is room.
    if os.environ.get("HAT_REPAIR"):
        plan.repair(radius=3.0, rounds=3)
    stage('repair %s' % ('done' if os.environ.get('HAT_REPAIR') else 'skipped'))
    # 2026-10-03: after the fanout fix, DRC left J3.19 and
    # U1.2 as the only GND pads with no path to the plane; bond() at 1.5 mm had
    # no legal via spot next to the 0.5 mm FFC comb or the SOT-583.
    bonded["GND-wide"] = plan.bond("GND", max_mm=3.0, width=0.15,
                                   refs=["J3", "U1", "U7", "U8"],
                                   why="the FFC comb, the SOT-583 and the two "
                                       "0.5 mm pitch rev B parts need a longer stub")
    bonded["VBAT-wide"] = plan.bond("VBAT", max_mm=3.0, width=0.15, refs=["U8"],
                                    why="the INA226's Vin- and Vbus pins are 0.5 mm "
                                        "apart: no via fits within 1.5 mm of them")
    stage('wide GND bond: %d bonded, refused %s' % (len(bonded["GND-wide"][0]),
                                                    bonded["GND-wide"][1]))
    if not os.environ.get("HAT_NO_FINISH"):
        plan.finish(width=0.15, why="retry at the board minimum", skip=skip)
        stage('finish 0.15 done, %d open' % len(plan.open_joins()))
        plan.finish(width=0.127, why="last pass at the rule minimum", skip=skip)
    stage('repair+finish done, %d open' % len(plan.open_joins()))
    pruned = plan.prune_dangling(why="stubs that end in their own track")
    isl = [plan.stitch_islands("GND", lay, why="an unstitched ground island is not ground")
           for lay in ("F.Cu", "B.Cu")]

    stage('islands done')
    b.tidy_silkscreen(ref_mm=1.0, values_to_fab=True)
    placed, hidden = place_refs(b, size=0.8, thickness=0.12, max_mm=5.0)
    for ref in ("H2", "H3", "H4"):
        b.component(ref).Reference().SetVisible(False)
    fab_refs(b, size=0.6, thickness=0.1)
    b.fill_zones()
    b.save(path)

    stage('saved')
    drc, err = meter.run_drc(path, out_json=os.path.join(OUT, "drc.json"),
                             refill=True, save=True)
    summary = {
        "name": NAME, "layers": 4, "rotations": rots, "parts": len(P.PARTS),
        "nets": len(P.NETS), "csi0_copied": {"segments": csi[0], "arcs": csi[1], "vias": csi[2]},
        "routed_rows": len(rows), "pruned": pruned, "stitch": stitched,
        "bond": {k: [len(v[0]), v[1]] for k, v in bonded.items()},
        "islands": isl, "open_joins": plan.open_joins(),
        "silk_refs_placed": placed, "silk_refs_hidden": hidden,
        "drc": drc, "drc_error": err, "seconds": round(time.time() - t0, 1),
    }
    json.dump(summary, open(os.path.join(OUT, "build-summary.json"), "w"),
              indent=1, sort_keys=True, default=str)
    print(json.dumps(summary, indent=1, sort_keys=True, default=str))


if __name__ == "__main__":
    build(place_only="--place-only" in sys.argv)
