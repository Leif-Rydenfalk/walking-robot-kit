# Rev C power budget

foreman/microduck-assembly.60, 2026-10-10. Every rail on the UNO Q head hat, its
source, its limit, the worst load we can name, and the margin. Margin = 1 - load/limit.

## 1. The power tree

```
2S pack 5.0-8.4 V ── J6 (JST PH 6-pin, 3 contacts per pole) ── Q2 AON6407 reverse-polarity P-FET
   └─ R22 5 mOhm shunt (INA226 U8 reads V and I) ── VBAT
        ├─ F2 5 A PTC ── U10 TPS61088 boost ── VIN12 11.86 V ── J1.57 + J1.59 ── UNO Q VIN
        │                                                         (UNO Q makes its own 5 V, 3.3 V, 1.8 V)
        ├─ F1 3 A PTC ── U3 TPS62933 buck ── V5 5.0 V ── U4 PAM8302 amplifier only
        └─ J5.2 servo-bus VBAT reference (see section 4)
UNO Q +3V3 (J1.58/60, J2.53/55) and +1V8 (J2.57) ── hat logic, camera, ToF, IMU
```

Rev C change: the hat no longer back-feeds the UNO Q's 5 V (JMISC 54/56 are open). The
UNO Q runs from its own VIN, fed by the boost. V5 now feeds only the amplifier.

## 2. Loads

| load | rail | worst current | source of the number |
|---|---|---|---|
| UNO Q, whole board | VIN12 | 15 W on 5V_SYS (3 A) | ABX00162 datasheet sec. 3.1, 5V_SYS max |
| UNO Q VIN path | VIN12 | 15 W / 0.90 buck + Schottky 0.39 V drop = 18.1 W = 1.52 A at 11.86 V | UNO Q schematic: VIN buck then Schottky; 90 % buck assumed |
| camera module (IMX219 class) | UNO Q +3V3 / +1V8 | 250 mA | `board.py` RAIL netclass note |
| VL53L5CX ToF (J4) | +3V3 | 100 mA | same |
| LSM6DSV16X, TCA9555, INA226, TCA9406, TTP233H, buffers | +3V3 | < 10 mA together | datasheets (each under 1 mA active) |
| PAM8302 at 2.5 W into 4 ohm | V5 | 0.65 A peak | PAM8302 datasheet, 2.5 W at 5 V |
| U10 EN divider R35+R36 | VBAT_B | 42 uA at 8.4 V | Ohm's law |

The hat's 3.3 V and 1.8 V loads come out of the UNO Q's own regulators, so they are
inside the UNO Q's 15 W above and do not add to the hat converters.

## 3. Margin per rail

| rail / part | limit | worst load | margin | notes |
|---|---|---|---|---|
| VIN12 boost output (U10) | 2.03 A at 5.0 V in | 1.52 A | **25 %** | limit from ILIM min 7.1 A (R31 150k), ripple 3.4 A p-p with L -20 %, 90 % efficiency. At 6.0 V pack the limit is 2.43 A (37 %) |
| L2 FXL0630-2R2 | Isat 10.5 A | 5.4 A peak | **49 %** | peak = average input 4.0 A + half ripple 1.4 A (nominal L) |
| L2 inside current limit | ILIM min 7.1 A | 5.4 A peak | **24 %** | the boost limits before L2 saturates |
| F2 JK-MSMD500L (VBAT_B) | 5.0 A hold at 25 C | 4.0 A at 5.0 V pack | **20 %** | at 6.0 V pack 3.34 A (33 %). PTC hold falls about 20 % at 50 C: at a hot, empty pack the margin is near zero, see 5.1 |
| JMEDIA J1.57 + J1.59 (UNO Q VIN) | 1.0 A per contact | 0.76 A each | **24 %** | two contacts share 1.52 A |
| U10 junction | 125 C | 102 C at 40 C ambient | **18 %** | about 1.6 W in the IC (2.0 W total loss at 90 %, 0.24 W of it in L2) at worst load x RthJA 38.8 C/W |
| V5 buck (U3 TPS62933) | 3 A | 0.65 A | **78 %** | amplifier only after Rev C |
| F1 SMD1812P300TF (VBAT_F) | 3.0 A hold | 0.72 A at 5.0 V pack | **76 %** | |
| L1 FNR6045 6.8 uH | 3.3 A | 0.65 A + ripple | **> 70 %** | |
| R22 shunt 2 W | 2 W | 4.7 A: 0.11 W | **95 %** | INA226 range +-81.92 mV = +-16 A |
| Q2 AON6407 | 85 A class, < 6 mOhm | 4.7 A: 0.13 W | **> 90 %** | |
| J6 pack connector S6B-PH-SM4-TB, 3 contacts per pole | 6 A (2 A per JST PH contact) | 4.72 A at 5.0 V with UNO Q at 15 W and amp at full | **21 %** | realistic load 2.76 A (54 %). Was 2 per pole (4 A, -18 %) until build #10 |
| UNO Q +3V3 to hat | UNO Q regulator (inside 15 W) | 360 mA | - | the hat adds no regulator here |

## 4. Servo power

Servo power does not come from this board (`docs/wiring.md`: give the chain its own
5.0-8.4 V supply and share ground). J5 pin 2 carries VBAT only as a reference for a
servo bus that is powered elsewhere. A JST PH contact is rated 2 A; do not run servo
current through J5.

## 5. Corners to know

### 5.1 Hot, empty pack

At 5.0 V in, 50 C board and the UNO Q at its 15 W maximum, F2's hold current (about 4 A
derated) meets the 4.0 A draw. The UNO Q never draws 15 W from 5V_SYS in this robot:
that figure includes the USB-C host port at 3 A. A realistic heavy load (NPU plus camera
streaming, 8 W) needs 2.04 A from the pack at 5.0 V, 49 % under F2's derated hold.

### 5.2 J6 pack connector

Up to build #9 J6 was a 4-pin PH with two contacts per pole: 4 A, below the 4.72 A worst
case (UNO Q at 15 W plus the amplifier at full, 5.0 V pack). A kit buyer plugs in whatever
they have, so build #10 (2026-10-10) made J6 a 6-pin S6B-PH-SM4-TB (C265405) with three
contacts per pole: 6 A, 21 % over the worst case. The kit ships a J6-to-XT30 pigtail so the
buyer plugs in a normal 2S pack (see the order sheet).

## 6. Method

- Boost limit: D = 1 - VIN x eta / VOUT; ripple = VIN x D / (L x fSW); average input
  limit = ILIM min - ripple/2; IOUT = Iin x VIN x eta / VOUT. ILIM min from the datasheet
  spread at 100k (10.6/11.9) applied to 7.93 A typ.
- fSW from datasheet equation 2 with R30 = 300k (516 kHz at 5.0 V in).
- Strap pins and compensation: `STRAP-PINS-REV-C.md`.
