#!/usr/bin/env python3
"""bom_stock.py - every LCSC code on the hat, checked live on LCSC and JLCPCB.

    python3 tools/bom_stock.py [--boards 5]

Reads hat_parts.PARTS (fitted, in-BOM rows), asks LCSC's product service for
stock and price and JLCPCB's SMT parts list for what the assembly line holds,
and writes data/bom-stock.json with the fetch time on every row. A line is OK
when JLC holds at least the quantity a build of --boards needs.
"""
import json
import os
import sys
import time
from collections import OrderedDict

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hat_parts as P   # noqa: E402
import lcsc             # noqa: E402


def lines():
    out = OrderedDict()
    for ref, sym, fp, val, code, side, at, rot, fitted, note in P.PARTS:
        if not code or not fitted or ref.startswith(("TP", "H", "JP")):
            continue
        row = out.setdefault(code, {"lcsc": code, "value": val, "footprint": fp,
                                    "refs": []})
        row["refs"].append(ref)
    return list(out.values())


def main():
    boards = int(sys.argv[sys.argv.index("--boards") + 1]) if "--boards" in sys.argv else 5
    rows = []
    for ln in lines():
        code = ln["lcsc"]
        try:
            d = lcsc.detail(code)
        except Exception as e:                      # noqa: BLE001
            d = {"verdict": "LCSC ERROR %s" % e}
        try:
            j = [r for r in lcsc.jlc(code) if r.get("lcsc") == code]
        except Exception as e:                      # noqa: BLE001
            j = []
            d["jlc_error"] = str(e)
        need = len(ln["refs"]) * boards
        jst = j[0]["jlc_stock"] if j else None
        row = dict(ln, qty_per_board=len(ln["refs"]), need=need,
                   mpn=d.get("mpn"), brand=d.get("brand"), package=d.get("package"),
                   lcsc_stock=d.get("stock"), lcsc_usd_10=lcsc.usd_at(d, 10 * len(ln["refs"])),
                   datasheet=d.get("datasheet"), jlc_stock=jst,
                   jlc_library=j[0]["library"] if j else None,
                   fetched_utc=d.get("fetched_utc") or time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                   ok=bool(jst is not None and jst >= need))
        rows.append(row)
        print("%-9s %-4s %-28s %-20s lcsc %7s jlc %7s %-8s %s" % (
            code, row["qty_per_board"], (row["mpn"] or "")[:28], (row["package"] or "")[:20],
            row["lcsc_stock"], jst, row["jlc_library"], "OK" if row["ok"] else "SHORT"))
        time.sleep(0.4)
    path = os.path.join(HERE, "data", "bom-stock.json")
    json.dump({"boards": boards, "fetched_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "rows": rows}, open(path, "w"), indent=1)
    short = [r["lcsc"] for r in rows if not r["ok"]]
    print("lines", len(rows), "short", short, "->", path)


if __name__ == "__main__":
    main()
