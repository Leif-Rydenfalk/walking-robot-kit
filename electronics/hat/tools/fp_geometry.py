"""fp_geometry.py - pad geometry of every BOM footprint, read from the built board.

    ce-pcb/bin/pcb tools/fp_geometry.py out/unoq_hat.kicad_pcb data/fp-geometry.json

Run inside KiCad's interpreter. For one part of each LCSC code it records the
footprint name, pad count, pad 1 position relative to the part centre (in the
footprint's own unrotated, unflipped frame), the smallest centre-to-centre
pitch between signal pads and each distinct pad size. docs/FOOTPRINTS.md holds
these numbers against the datasheet's land pattern.
"""
import json
import sys

import pcbnew

MM = 1e6
b = pcbnew.LoadBoard(sys.argv[1])
seen = {}
for fp in b.GetFootprints():
    code = fp.GetFieldText("LCSC Part #") if fp.HasField("LCSC Part #") else ""
    if not code or code in seen:
        continue
    pads = []
    for p in fp.Pads():
        o = p.GetFPRelativePosition()
        s = p.GetSize(pcbnew.F_Cu) if hasattr(p, "GetSize") else p.GetSize()
        try:
            sx, sy = s.x / MM, s.y / MM
        except AttributeError:
            sx, sy = s[0] / MM, s[1] / MM
        pads.append((p.GetNumber(), round(o.x / MM, 3), round(o.y / MM, 3),
                     round(sx, 3), round(sy, 3), p.GetOrientationDegrees() if hasattr(p, "GetOrientationDegrees") else 0))
    num = [q for q in pads if q[0] and q[0] not in ("EP", "MP")]
    pitch = None
    for i in range(len(num)):
        for j in range(i + 1, len(num)):
            d = ((num[i][1] - num[j][1]) ** 2 + (num[i][2] - num[j][2]) ** 2) ** 0.5
            if d > 0.05 and (pitch is None or d < pitch):
                pitch = round(d, 3)
    p1 = [q for q in pads if q[0] == "1"]
    seen[code] = {
        "ref": fp.GetReference(), "footprint": fp.GetFPIDAsString(),
        "pads": len(pads), "numbered_pads": len({q[0] for q in num}),
        "pad1": p1[0][1:3] if p1 else None, "min_pitch": pitch,
        "pad_sizes": sorted({(q[3], q[4]) for q in pads}),
        "pad_list": pads,
    }
json.dump(seen, open(sys.argv[2], "w"), indent=1)
print(len(seen), "codes")
