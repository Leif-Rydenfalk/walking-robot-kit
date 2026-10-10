# Order the UNO Q head hat, Rev C (for Leif, 2026-10-11)

foreman/microduck-assembly.60, written 2026-10-10. Branch `foreman/devkit-pcb` of
walking-robot-kit. Nothing has been ordered or paid.

## Your one step

1. Open https://cart.jlcpcb.com/quote, signed in as you.
2. Upload `electronics/hat/fab/gerbers.zip`, set the options in the table below.
3. Turn on **PCB Assembly**, upload `electronics/hat/fab/bom.csv` and `electronics/hat/fab/cpl.csv`.
4. In JLC's placement preview, check the three parts in the "Look at these" list, then pay.

About 15 minutes. Everything else is done and checked.

## Files (all in `electronics/hat/`)

| file | what | sha256 (first 12) |
|---|---|---|
| `fab/gerbers.zip` | 4-layer gerbers + drills, 31 files | 5cdad18f8c70 |
| `fab/bom.csv` | JLC BOM, 55 lines, every line with an LCSC number | 5f05a8434ca9 |
| `fab/cpl.csv` | JLC placement, 95 parts: 93 bottom, 2 top (J1, J2) | 4b424a0854ed |

Pictures: `img/top.png`, `img/bottom.png`, `img/iso.png`. 3D: `3d/unoq_hat.step`.
Schematic: `schematic.pdf`.

## JLCPCB options

| option | set to | why |
|---|---|---|
| Base material | FR-4 | |
| Layers | 4 | JLC's parser read "4 layer board of 53.34x68.58mm" on Rev B; Rev C has the same outline |
| Dimensions | 68.58 x 53.34 mm | UNO outline |
| PCB qty | 5 | |
| Thickness | 1.6 mm | matches the Arduino carrier |
| Color / silk | green / white | cheapest, fastest |
| Material | FR4 TG155 | JLC forces it with the finer via class |
| Surface finish | **ENIG 1U"** | flat pads for 0.5 mm pitch parts (U10 VQFN, U1, U3, U8, J3 FFC) and the 1.27 mm board-to-board rows |
| Outer / inner copper | 1 oz / 0.5 oz | |
| Via covering | plugged (tented is fine too) | |
| **Min via hole / diameter** | **0.2 mm (0.3/0.35)** | one via inside the IMU's LGA is 0.20/0.40 mm; all others 0.25/0.45 |
| Board outline tolerance | +-0.2 mm | |
| Electrical test | flying probe | |
| Remove order number | yes (if free) | |
| **PCB Assembly** | **yes, Standard, both sides**, qty 5 | J1/J2 on top, everything else on the bottom |
| Tooling holes | added by JLC | |
| Confirm parts placement | yes | lets you see JLC's preview before it builds |

## Price

| item | amount | source |
|---|---|---|
| Bare boards, 5 pcs, these options | about $62 | live JLC quote of Rev B on 2026-10-03 (same size, layers, ENIG, TG155): $62.02. Rev C's 0.2 mm via class is one step finer, so expect a few dollars more |
| DHL DDP to you | about $31 | same quote, 2-4 business days |
| Assembly setup + stencils, two sides | about $50-70 | JLC Standard PCBA fees as published; the page shows the exact number |
| Extended-part fees | up to $123 | 41 lines JLC lists as extended at $3 each; "preferred" ones are free |
| Components, 5 boards | about $70 | $13.67 per board at LCSC 10+ prices (`electronics/hat/docs/BOM-REV-C.md`) |
| **Total, 5 assembled boards to your door** | **about $300-360** | the quote page is the real number; pay only if it is under $400 |

## Look at these in JLC's placement preview

1. **J1, J2** (top side, the two 2x30 1.27 mm board-to-board connectors): the only two top
   parts. They must sit on the two long pad rows.
2. **U10** (TPS61088, bottom, near the J1 end): pin-1 dot of the chip at the corner nearest C33.
3. **U7** (LSM6DSV16X, bottom, small 2.5 x 3 mm square): pin-1 dot matches the board dot.

Bottom-side rotations are the usual mistake. If any part sits rotated by 90 or 180, fix it in
the preview (drag-rotate) before paying.

## The JLCPCB quote from Kris is for Rev B

Kris at JLCPCB (support@jlcpcb.com) sent a quote on 2026-10-05, in your leif@rydenfalk.com
inbox. It was for **Rev B**. Rev C changed the power block (new boost U10, inductor L2, fuse
F2, D40, the 6-pin J6, the EN divider), so that quote does not apply. Order Rev C through the
web quote above, or ask Kris to redo it with these three files. Nobody has emailed JLCPCB.

## What Rev C fixes (the three HOLD findings from the 2026-10-07 audit)

| # | Rev B fault | Rev C fix |
|---|---|---|
| 1 | IMU, expander and pack-monitor interrupts drove 3.3 V into 1.8 V SoC pins (J2.37/39/41) | moved to 3.3 V MCU pins J2.3, J2.7, J2.9 |
| 2 | the hat's 5 V drove JMISC 54/56, which is USB VBUS on the UNO Q | hat no longer touches 54/56; a TPS61088 boost makes 11.86 V into the UNO Q's own VIN (7-24 V) on J1.57/59 |
| 3 | servo bus pull-ups back-fed +3V3 from a powered bus | D40 (BAT54H) blocks it |

Found and fixed on 2026-10-10 while checking Rev C (none of these show in DRC or ERC):

| # | fault | fix |
|---|---|---|
| 4 | U10 EN tied to VCC, an LDO that EN itself turns on: the boost would never start | R35/R36 100k/100k divider from the pack: 2.5 V at 5.0 V, 4.2 V at 8.4 V (EN max 7 V) |
| 5 | a 0.09 mm clearance rule in the JLC rule file let the ground pour into the JMEDIA pads' mask openings (83 bridges) | rule removed; pour back at 0.127 mm |
| 6 | J6 (two contacts per pole, 4 A) below the 4.7 A worst case | J6 is a 6-pin PH, three contacts per pole, 6 A |

## Checks (all on the files above)

| check | result | file |
|---|---|---|
| KiCad DRC, board rules | 0 errors, 0 warnings, 0 unconnected | `electronics/hat/checks/drc-build.json` |
| KiCad DRC with JLC rule file | 0 errors, 0 warnings, 0 unconnected | `checks/drc-jlc-rules.json` |
| KiCad ERC | 0 errors, 0 warnings | `checks/erc.json` |
| schematic vs board | 75 of 75 nets agree | `checks/build-summary.json` |
| ce-fabmcp pcb_drc | In1, In2 PASS at 0.127 mm; F.Cu, B.Cu "FAIL" from its circle approximation of fine-pitch rectangular pads, not real (KiCad's polygon DRC on the same copper is 0) | `checks/pcb-drc-fabmcp.json` |
| footprints vs datasheets | 51 of 51 PASS | `electronics/hat/docs/FOOTPRINTS-REV-C.md` |
| strap pins vs datasheets | 29 rows: 1 fault found and fixed (EN), 28 PASS | `electronics/hat/docs/STRAP-PINS-REV-C.md` |
| power budget | every rail has margin at the worst case; tightest are F2 (20 %) and J6 (21 %) | `electronics/hat/docs/POWER-BUDGET-REV-C.md` |
| BOM stock | 51 lines, 0 short for 5 boards, fetched 2026-10-10 14:53 UTC; tightest line J1/J2 (782 in stock for 10) | `electronics/hat/docs/BOM-REV-C.md` |

## Known limits of the first 5 boards

- J1/J2 are BOOMELE C191721, an equivalent of Arduino's Greenconn GPEC212 header. Pads and
  pitch match Arduino's carrier files exactly; the first boards are the fit test on a real UNO Q.
- Seven basic parts show 0 at the LCSC shop and 200 k to 20 M at JLC's own stock. JLC
  assembles from its own stock, so they are fine.
- Servo power does not go through this board. J5 pin 2 is a VBAT reference only (2 A contact).

## Kit line item: J6-to-XT30 pack lead

Buyers own a 2S pack with an XT30, not a 6-pin PH lead, so the kit ships this pigtail:

| part | LCSC | JLC stock (2026-10-10 14:53 UTC) | per lead |
|---|---|---|---|
| JST PHR-6 housing | C157952 | 58,616 | 1 |
| JST SPH-002T-P0.5S crimp contact | C111515 | 1,244,656 | 6 |
| Amass XT30U-F (female plug; check it mates the pack you test with) | C99102 | 48,043 | 1 |
| wire, 24 AWG silicone, red x3 + black x3, 150 mm | - | - | 6 |

PH crimps take 24-30 AWG only, so 20 AWG cannot go into the housing: the lead is six 24 AWG
wires, pins 1-3 red joined into the XT30 +, pins 4-6 black into the XT30 -. Three 24 AWG in
parallel carry the same as one 19 AWG. A harness shop in Huaqiangbei can make it. Check polarity with a meter before
the first plug-in: J6 pins 1-3 are pack +.
