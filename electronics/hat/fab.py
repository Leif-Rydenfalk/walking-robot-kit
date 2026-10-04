#!/usr/bin/env python3
"""fab.py - production files for the UNO Q head hat, from out/unoq_hat.kicad_pcb.

    python3 electronics/unoq-hat/fab.py

Writes out/fab/: gerbers + drill (zip for JLCPCB upload), JLC BOM (from the
schematic, `LCSC Part #`), JLC CPL (Designator, Mid X, Mid Y, Layer, Rotation),
and out/render/: top.png, bottom.png, iso.png, plus out/unoq_hat.step.
"""
import csv
import os
import subprocess
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
FAB = os.path.join(OUT, "fab")
REN = os.path.join(OUT, "render")
PCB = os.path.join(OUT, "unoq_hat.kicad_pcb")
SCH = os.path.join(OUT, "unoq_hat.kicad_sch")
K = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
sys.path.insert(0, HERE)
import hat_parts as P                                         # noqa: E402


def run(*args):
    r = subprocess.run([K] + list(args), capture_output=True, text=True)
    if r.returncode:
        raise SystemExit("kicad-cli %s FAILED: %s %s" % (" ".join(args[:3]), r.stdout[-400:], r.stderr[-400:]))
    return r.stdout


def main():
    os.makedirs(FAB, exist_ok=True)
    os.makedirs(REN, exist_ok=True)
    g = os.path.join(FAB, "gerbers")
    os.makedirs(g, exist_ok=True)
    run("pcb", "export", "gerbers", "--no-protel-ext", "-o", g + "/", PCB)
    run("pcb", "export", "drill", "--format", "excellon", "--excellon-separate-th",
        "--generate-map", "--map-format", "gerberx2", "-o", g + "/", PCB)
    z = os.path.join(FAB, "unoq_hat-gerbers-jlcpcb.zip")
    with zipfile.ZipFile(z, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in sorted(os.listdir(g)):
            zf.write(os.path.join(g, f), f)
    # CPL
    pos = os.path.join(FAB, "pos.csv")
    run("pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both",
        "--exclude-dnp", "-o", pos, PCB)
    fitted = {r[0] for r in P.PARTS if r[8] and r[4]}
    n = 0
    with open(pos) as f, open(os.path.join(FAB, "unoq_hat-cpl-jlcpcb.csv"), "w", newline="") as o:
        w = csv.writer(o)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for row in csv.DictReader(f):
            if row["Ref"] not in fitted:
                continue
            w.writerow([row["Ref"], row["PosX"] + "mm", row["PosY"] + "mm",
                        "Top" if row["Side"].lower().startswith("top") else "Bottom",
                        row["Rot"]])
            n += 1
    # BOM (JLC columns) straight from the design table, grouped by LCSC code
    groups = {}
    for ref, sym, fp, val, lcsc, side, at, rot, fit, note in P.PARTS:
        if not (fit and lcsc):
            continue
        k = (val, fp.split(":")[-1], lcsc)
        groups.setdefault(k, []).append(ref)
    with open(os.path.join(FAB, "unoq_hat-bom-jlcpcb.csv"), "w", newline="") as o:
        w = csv.writer(o)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for (val, fp, lcsc), refs in sorted(groups.items(), key=lambda kv: kv[1][0]):
            w.writerow([val, ",".join(refs), fp, lcsc])
    # renders + STEP
    for side, name in (("top", "top"), ("bottom", "bottom")):
        run("pcb", "render", "--side", side, "--width", "1600", "--height", "1240",
            "--quality", "high", "--background", "opaque", "-o", os.path.join(REN, name + ".png"), PCB)
    run("pcb", "render", "--side", "bottom", "--rotate", "-35,0,30", "--width", "1600",
        "--height", "1240", "--quality", "high", "--background", "opaque", "--perspective",
        "-o", os.path.join(REN, "iso-bottom.png"), PCB)
    run("pcb", "export", "step", "--subst-models", "--force", "-o",
        os.path.join(OUT, "unoq_hat.step"), PCB)
    print("fab: %d CPL rows, %d BOM lines, gerber zip %s" % (n, len(groups), z))


if __name__ == "__main__":
    main()
