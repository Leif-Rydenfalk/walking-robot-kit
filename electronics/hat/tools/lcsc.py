#!/usr/bin/env python3
"""lcsc.py - live LCSC part check for the UNO Q head hat (copied from rev-c tools, unchanged logic).

Two verbs, both hitting LCSC's own product service and recording the fetch
date, so no part number on this board is typed from memory.

    python3 tools/lcsc.py detail C109322 C910544 ...
    python3 tools/lcsc.py search "292133-10"
    python3 tools/lcsc.py jlc "TCA9406DCUR"      JLCPCB assembly stock + basic/extended

Writes/updates out/lcsc.json. Every row carries fetched_utc, stock and the
price ladder as LCSC served it.
"""
import json
import os
import ssl
import sys
import time
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, "out")
STORE = os.path.join(OUT, "lcsc.json")
UA = ("Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/140.0.0.0 Safari/537.36")
CTX = ssl.create_default_context()
CTX.check_hostname = False
CTX.verify_mode = ssl.CERT_NONE


def _get(url):
    req = urllib.request.Request(url, headers={"User-Agent": UA,
                                               "Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=30, context=CTX) as r:
        return json.loads(r.read().decode("utf-8", "replace"))


def detail(code):
    url = ("https://wmsc.lcsc.com/ftps/wm/product/detail?productCode=%s"
           % urllib.parse.quote(code))
    d = _get(url).get("result") or {}
    if not d:
        return {"lcsc": code, "verdict": "NOT FOUND",
                "fetched_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                             time.gmtime())}
    ladder = [{"qty": p.get("ladder"), "usd": p.get("usdPrice")}
              for p in (d.get("productPriceList") or [])]
    return {
        "lcsc": d.get("productCode"), "mpn": d.get("productModel"),
        "brand": d.get("brandNameEn"), "desc": d.get("productNameEn"),
        "package": d.get("encapStandard"), "stock": d.get("stockNumber"),
        "min_buy": d.get("minBuyNumber"), "ladder": ladder,
        "datasheet": d.get("pdfUrl"),
        "fetched_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "source": url,
    }


def search(term, limit=8):
    url = ("https://wmsc.lcsc.com/ftps/wm/search/global?keyword=%s"
           % urllib.parse.quote(term))
    res = _get(url).get("result") or {}
    rows = (res.get("productSearchResultVO") or {}).get("productList") or []
    if not rows and res.get("tipProductDetailUrlVO"):
        code = res["tipProductDetailUrlVO"].get("productCode")
        if code:
            return [detail(code)]
    out = []
    for p in rows[:limit]:
        out.append({"lcsc": p.get("productCode"), "mpn": p.get("productModel"),
                    "brand": (p.get("brandNameEn")),
                    "desc": p.get("productNameEn"),
                    "package": p.get("encapStandard"),
                    "stock": p.get("stockNumber")})
    return out


def jlc(term, limit=6):
    """JLCPCB's own assembly parts list: what the SMT line can place today.

    Added 2026-10-02 because LCSC's search endpoint answers 403 to this client
    while JLC's answers; `detail()` on LCSC still works for a known code.
    """
    body = json.dumps({"keyword": term, "currentPage": 1,
                       "pageSize": limit}).encode()
    req = urllib.request.Request(
        "https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/"
        "smtGood/selectSmtComponentList", data=body,
        headers={"User-Agent": UA, "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=30, context=CTX) as r:
        d = json.loads(r.read().decode("utf-8", "replace"))
    rows = ((d.get("data") or {}).get("componentPageInfo") or {}).get(
        "list") or []
    return [{"lcsc": r.get("componentCode"), "mpn": r.get("componentModelEn"),
             "package": r.get("componentSpecificationEn"),
             "jlc_stock": r.get("stockCount"),
             "library": r.get("componentLibraryType"),
             "desc": (r.get("describe") or "")[:90],
             "fetched_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                          time.gmtime())} for r in rows]


def usd_at(row, qty):
    """Unit price for a build of `qty`: the deepest ladder step reached."""
    best = None
    for step in row.get("ladder") or []:
        if step.get("qty") is not None and step["qty"] <= qty:
            best = step["usd"]
    if best is None and row.get("ladder"):
        best = row["ladder"][0]["usd"]
    return best


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    os.makedirs(OUT, exist_ok=True)
    store = {}
    if os.path.exists(STORE):
        store = json.load(open(STORE))
    if argv[1] == "detail":
        for code in argv[2:]:
            row = detail(code)
            store[row.get("lcsc") or code] = row
            print(json.dumps(row, ensure_ascii=False))
    elif argv[1] == "jlc":
        rows = jlc(" ".join(argv[2:]))
        for row in rows:
            print(json.dumps(row, ensure_ascii=False))
        store.setdefault("_jlc", {})[" ".join(argv[2:])] = rows
    elif argv[1] == "search":
        for row in search(" ".join(argv[2:])):
            print(json.dumps(row, ensure_ascii=False))
        return 0
    else:
        print(__doc__)
        return 2
    json.dump(store, open(STORE, "w"), indent=1, sort_keys=True,
              ensure_ascii=False)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
