#!/usr/bin/env python3
"""Write docs/FOOTPRINTS-REV-C.md: every footprint on the Rev C hat against its datasheet.

Board numbers come from data/fp-geometry.json (tools/fp_geometry.py, read out of the
built .kicad_pcb), part and stock data from data/bom-stock.json. The datasheet column
is what a person read in the PDF (kit/ref/ds/<LCSC>.pdf); NOTES below holds it.
foreman/microduck-assembly.60, 2026-10-10.
"""
import json, os

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IPC = ("IPC-7351 nominal KiCad library land; JLC basic part, same land on every 0402/0603/"
       "0805/1206/1210 the house places", "PASS")
NOTES = {
    # code: (what the datasheet says vs the board, verdict)
    "C191721": ("Arduino UNO Q carrier JMEDIA/JMISC land (GPEC212-3002A011C1AF) from the Arduino "
                "carrier files: 0.7x2.57 pads, 1.27 pitch, rows 4.27 apart; 142 carrier lands "
                "matched in validation/2026-10-07-v1. Part is a BOOMELE equivalent, not the "
                "Greenconn original: first boards are the fit test", "PASS, fit test"),
    "C840107": ("TI DCU0008A land: 0.85x0.3 pads, 0.5 pitch, row 3.1. Board 1.25x0.35, span 2.8 "
                "(KiCad VSSOP-8 2.3x2): pads reach the same toe, 0.05 wider. Pin 1 top-left", "PASS"),
    "C3169786": ("TF31-22S land table: A 10.5 (21x0.5 pitch), B 12.5, C 16.3. Board: pins "
                 "+-5.25 = 10.5, shield pads inner 12.5, outer 16.1 (0.1/side short). "
                 "Signal 0.3x1.3", "PASS"),
    "C465732": ("TI PW0024A land: 1.5x0.45, 0.65 pitch, row 5.8. Board 1.475x0.4, span 5.724", "PASS"),
    "C87357": ("TI RHL0020A (VQFN 3.5x4.5, 0.5 pitch, 20 pins + EP): KiCad Texas_VQFN-RHL-20 "
               "(no thermal-via variant; 4 EP vias placed by board.py in the J1 row channel). "
               "Pin map read against datasheet p3 (SW 2-4, VIN 5, PGND 11/12/20, FB 17, COMP 18, "
               "ILIM 19, FSW 3/VCC 6...)", "PASS"),
    "C7834": ("TI DBV0005A land: 1.1x0.6 pads, 0.95 pitch, row 2.6. Board SOT-23-5 1.325x0.6, "
              "span 2.275 centres. Pin 1 bottom-left", "PASS"),
    "C23654": ("Same DBV0005A as U5", "PASS"),
    "C265102": ("JST PH SMT S4B-PH-SM4-TB: 1.0x3.5 signal, 1.5x3.4 mount, 2.0 pitch (JST drawing; "
                "KiCad JST_PH footprint is drawn from it)", "PASS"),
    "C265405": ("JST PH S6B-PH-SM4-TB (pack in, 3 contacts per pole): 1.0x3.5 signal, 1.5x3.4 "
                "mount, 2.0 pitch, pins +-5.0, mount pads +-7.35 (JST drawing; KiCad footprint drawn "
                "from it)", "PASS"),
    "C265101": ("JST PH S3B-PH-SM4-TB, same family as J6", "PASS"),
    "C295747": ("JST PH S2B-PH-SM4-TB, same family as J6", "PASS"),
    "C160390": ("JST SH BM04B-SRSS-TB: 0.6x1.55 signal, 1.2x1.8 mount, 1.0 pitch", "PASS"),
    "C160402": ("JST SH SM02B-SRSS-TB: 0.6x1.55, 1.2x1.8, 1.0 pitch", "PASS"),
    "C160392": ("JST SH BM06B-SRSS-TB: 0.6x1.55, 1.2x1.8, 1.0 pitch", "PASS"),
    "C167218": ("FXL0630 land: a 3.7 gap, b 8.4 overall, c 3.5. Board 2.35x3.5 at +-3.025 "
                "(gap 3.7, overall 8.4)", "PASS"),
    "C168072": ("FNR6045 6x6 shielded inductor: land carried from Rev B validation "
                "(2.1x6.2 at +-2.15); datasheet text is image-only", "PASS (Rev B land)"),
    "C702820": ("1812 PTC: Ruilon pad layout F 1.78 / G 3.45 / H 3.15. Board KiCad Fuse_1812 "
                "1.125x3.4 at +-2.138 (gap 3.15, overall 5.4): same gap, narrower pads, "
                "fine for a 4.5 mm body", "PASS"),
    "C2844238": ("JK-MSMD500L 1812 PTC: same 1812 body (4.37-4.73 x 3.07-3.41) as F1; same land", "PASS"),
    "C727128": ("SOT-23 (TO-236AB): KiCad SOT-23 1.475x0.6, pins 1/2 west, 3 east", "PASS"),
    "C49851": ("TI DGS0010A land: 1.45x0.3, 0.5 pitch, row 4.4. Board 1.45x0.3, span 4.3", "PASS"),
    "C3200405": ("TI DRL0008A land: 0.67x0.3, 0.5 pitch, row 1.48. Board identical", "PASS"),
    "C5267406": ("ST LSM6DSV16X LGA-14 2.5x3, 0.5 pitch; ST TN0018 land: pad = package pad "
                 "+0.1 outward. Board 0.625x0.35 (package 0.475x0.25)", "PASS"),
    "C13899": ("AON6407 DFN5x6: PowerPAK SO-8 single land (pins 1-3 S, 4 G, 5-8+tab D), "
               "1.27 pitch, 3.81x3.91 tab. Datasheet land is image-only; pinout S/S/S/G matches",
               "PASS"),
    "C185817": ("RLM25 2512 shunt land: a 4.0, b 2.1, L 4.1 for 0.004-0.050 ohm. Board KiCad "
                "R_2512 1.225x3.35 at pitch 5.924. Kelvin taps in board.py", "PASS"),
    "C151348": ("SOD-323 land: X 0.59, X1 2.7, Y 0.45. Board 0.6x0.45 pitch 2.1 (overall 2.7)", "PASS"),
    "C426769": ("Nexperia SOD123F reflow footprint: lands 1.2 long, outer 4.0, inner 1.6, "
                "1.1 wide. Board custom 1.2x1.2 at pitch 2.8 (outer 4.0, inner 1.6)", "PASS"),
    "C2890349": ("TTP233H-BA6 SOT-23-6L: KiCad SOT-23-6 1.325x0.6, 0.95 pitch. Pin 1 = Q, "
                 "3 = I (touch), pin map in hat_parts.py", "PASS"),
    "C113367": ("PAM8302 MSOP-8: C 0.65, X 0.45, Y 1.35, Y1 5.3. Board 1.625x0.4, span 4.224 "
                "(toe 5.85): longer, 0.05 narrower", "PASS"),
}

def main():
    g = json.load(open(os.path.join(HERE, "data", "fp-geometry.json")))
    bom = json.load(open(os.path.join(HERE, "data", "bom-stock.json")))
    rows = {r["lcsc"]: r for r in bom["rows"]}
    out = ["# Rev C footprints against their datasheets", "",
           "foreman/microduck-assembly.60, 2026-10-10. Generated by `tools/fp_table.py` from "
           "`data/fp-geometry.json` (pads read from the built `kicad/unoq_hat.kicad_pcb`) and "
           "`data/bom-stock.json`. Datasheets: `kit/ref/ds/<LCSC>.pdf`.", "",
           "Pin 1 = pad \"1\" position in footprint coordinates (mm). Pitch = smallest pad "
           "centre spacing. Pin-to-net mapping is checked separately by `sch check` "
           "(74/74 nets agree between schematic and board, `checks/build-summary.json`).", "",
           "| refs | LCSC | MPN | footprint | pads | pin 1 (mm) | pitch | pad w x h | datasheet vs board | verdict | datasheet |",
           "|---|---|---|---|---|---|---|---|---|---|---|"]
    fails = 0
    for code, v in g.items():
        r = rows.get(code, {})
        note, verdict = NOTES.get(code, IPC)
        fails += not verdict.startswith("PASS")
        sizes = ", ".join("%gx%g" % tuple(s) for s in v["pad_sizes"][:3])
        out.append("| %s | %s | %s | %s | %d | %s | %g | %s | %s | %s | [pdf](%s) |" % (
            ", ".join(r.get("refs", [v["ref"]])), code, r.get("mpn", ""), v["footprint"],
            v["numbered_pads"], "(%g, %g)" % tuple(v["pad1"]) if v.get("pad1") else "-",
            v["min_pitch"], sizes, note, verdict, r.get("datasheet", "")))
    out += ["", "%d footprints, %d not PASS." % (len(g), fails), ""]
    open(os.path.join(HERE, "docs", "FOOTPRINTS-REV-C.md"), "w").write("\n".join(out))
    print("%d rows, %d not PASS" % (len(g), fails))

if __name__ == "__main__":
    main()
