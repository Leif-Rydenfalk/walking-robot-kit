# Rev C strap pins against their datasheets

foreman/microduck-assembly.60, 2026-10-10. One row per configuration pin (enable,
undervoltage, mode, boot, address, frequency, limit, compensation, unused inputs) on
every IC of the Rev C hat. Net names are from `hat_parts.py` NETS; datasheets are
`kit/ref/ds/<LCSC>.pdf` (URLs in `data/bom-stock.json`).

DRC and ERC cannot see these: a pin wired to the wrong rail is still a legal net.
This pass found one real fault (row 1), fixed in build #8.

## Result

| # | IC | pin | net / part | datasheet says | board | verdict |
|---|---|---|---|---|---|---|
| 1 | U10 TPS61088 | 2 EN | BST_EN: R35 100k to VBAT_B, R36 100k to GND | EN high enables, low = shutdown (VENH 1.2 V, VENL 0.4 V); abs max EN 7 V; VCC (pin 1) is the internal LDO output and the LDO sits under shutdown control (block diagram; ISD 1 uA) | Rev C up to build #7 tied EN to VCC: VCC is 0 V in shutdown, so the boost never starts and the UNO Q gets no VIN. Build #8: 1:1 divider, EN = 2.5 V at 5.0 V pack, 4.2 V at 8.4 V; 42 uA | **FIXED** (was FAIL) |
| 2 | U10 | 1 VCC | C33 4.7 uF 0603 to GND | ceramic > 1.0 uF required | 4.7 uF 16 V X5R (about 2.5 uF at 5.2 V bias) | PASS |
| 3 | U10 | 3 FSW | R30 300k to SW2 | RFREQ = 4(1/fSW - tDELAY*VOUT/VIN)/CFREQ, CFREQ 23 pF, tDELAY 89 ns; must always be fitted | 516 kHz at 5.0 V in, 540 kHz at 8.4 V; 200 kHz-2.2 MHz range | PASS |
| 4 | U10 | 8 BOOT | C34 100 nF to SW2 | 0.1 uF BOOT-SW (typical application) | 100 nF 25 V | PASS |
| 5 | U10 | 10 SS | C35 47 nF to GND | 5 uA charge to 1.204 V; "47 nF is usually sufficient" | tSS = 47n x 1.204 / 5u = 11 ms | PASS |
| 6 | U10 | 11, 12 NC | GND | "connect these two pins to ground plane for thermal dissipation" | GND, stubs to EP | PASS |
| 7 | U10 | 13 MODE | not connected | floating = PFM at light load, GND = forced PWM | floating (PFM, light-load efficiency) | PASS |
| 8 | U10 | 17 FB | R32 1M / R33 113k | VREF 1.204 V; VOUT 4.5-12.6 V; FB abs max 3.6 V | 1.204 x (1 + 1000/113) = 11.86 V; UNO Q VIN range 7-24 V | PASS |
| 9 | U10 | 18 COMP | R34 18k + C36 6.8 nF to GND | R5 = 2pi VOUT Rsense fC CO / ((1-D) VREF GEA), C5 = RO CO / (2 R5), C8 = RESR CO / R5 (open if < 10 pF); fC < fSW/10 and < fRHPZ/5. GEA 190 uA/V, Rsense 0.08 ohm | worst corner 5.0 V in, 1.52 A out: D 0.62, RO 7.8 ohm, CO ~30 uF effective (3 x 22 uF at 11.9 V bias), fRHPZ 81 kHz, so fC max 16 kHz. 18k gives fC about 9 kHz; ideal C5 = 6.5 nF, fitted 6.8 nF; C8 = 3 fF, left open | PASS |
| 10 | U10 | 19 ILIM | R31 150k to GND | MODE floating: ILIM = 1 190 000 / RILIM; 100k gives 10.6/11.9/13 A min/typ/max | 7.9 A typ, 7.1 A min (same spread); peak inductor current at 1.52 A out and 5.0 V in is 5.2 A; L2 Isat 10.5 A | PASS |
| 11 | U10 | 9 VIN | VBAT_B via F2 | 2.7-12 V, UVLO 2.7 V rising | 5.0-8.4 V pack | PASS |
| 12 | U3 TPS62933 | 2 EN | not connected | "drive EN high or leave the pin floating to enable"; abs max 5.5 V | floating: on whenever VBAT_F is up | PASS |
| 13 | U3 | 1 RT | not connected | RT floating = 500 kHz | floating | PASS |
| 14 | U3 | 7 SS | C10 10 nF to GND | external soft-start capacitor | 10 nF | PASS |
| 15 | U3 | 3 VIN | VBAT_F via F1 | 3.8-30 V; UVLO rising 3.4-3.8 V | 5.0-8.4 V | PASS |
| 16 | U4 PAM8302 | 1 SD | AMP_SD: R9 100k to GND, J2.1 GPIO | low = shutdown; VSH 1.2 V min, VSL 0.4 V max; start SD 1-100 ms after VDD | default off through R9, MCU drives 3.3 V high when it plays sound | PASS |
| 17 | U1 TCA9406 | 6 OE | R1 10k to +1V8, C1 4.7 uF to GND | OE referenced to VCCA; keep low while supplies come up | VCCA = +1V8 (pin 3); RC 47 ms holds OE low until both rails are up | PASS |
| 18 | U2 TCA9555 | 21 A0, 2 A1, 3 A2 | GND, +3V3, +3V3 | address 0100 A2 A1 A0 | 0x26; bus map in `sch.py`: camera 0x10, ToF 0x29, INA226 0x40, IMU 0x6B, trunk IMU 0x6A. No clash | PASS |
| 19 | U2 | 1 INT | EXP_INT: R2 10k to +3V3, J2.7 | open-drain, needs pull-up | 10k to 3.3 V, read on a 3.3 V MCU pin (Rev C fix) | PASS |
| 20 | U7 LSM6DSV16X | 1 SDO/SA0 | +3V3 | SA0 = address LSB | 0x6B | PASS |
| 21 | U7 | 12 CS | +3V3 | CS high = I2C mode | +3V3 | PASS |
| 22 | U7 | 2 SDx, 3 SCx | GND | "connect to Vdd_IO or GND if the analog hub and Qvar are disabled" | GND | PASS |
| 23 | U7 | 9 INT2, 10 OCS_Aux, 11 SDO_Aux | not connected | INT2 is an output; OCS_Aux and SDO_Aux "leave unconnected" in mode 1 | not connected | PASS |
| 24 | U8 INA226 | 2 A0, 1 A1 | GND, GND | A1 A0 = GND GND gives 0x40 | 0x40 | PASS |
| 25 | U8 | 3 ALERT | INA_ALERT: R29 10k to +3V3, J2.9 | open-drain | 10k to 3.3 V, 3.3 V MCU pin (Rev C fix) | PASS |
| 26 | U9 TTP233H-BA6 | 4 AHLB, 6 TOG | GND, GND | both have internal pull-low; 0 = active-high output, 0 = direct (not toggle) | Q high while touched | PASS |
| 27 | U5 74LVC1G126 / U6 74LVC1G125 | 1 OE | TXEN: R16 10k to GND, JP1 AUTO (Q1) or GPIO | '126 OE active high, '125 OE active low | TXEN high drives the bus (U5), low listens (U6); default low = listen | PASS |
| 28 | Q2 AON6407 | 4 G | Q2_G: R11 100k to GND | VGS max +-25 V; RDS(on) < 6 mOhm at VGS -6 V | VGS = -5.0 to -8.4 V: well inside +-25 V, fully on | PASS |
| 29 | D40 BAT54H | - | SERVO_PULLUP from +3V3 | 30 V, 200 mA | blocks servo-bus pull-up back-feed into the unpowered 3.3 V rail (Rev C fix) | PASS |

28 rows PASS, 1 row was a FAIL in Rev C before build #8 and is fixed.

## How row 1 was found

The TPS61088 pin table (datasheet p3) and section 8.3.1 say EN enables the device and
the block diagram draws the VCC LDO inside the shutdown domain. With EN tied to VCC,
VCC never rises, so EN never goes high: a latch that stays off. The UNO Q would have
had no supply on battery. The board passed DRC and ERC with this fault, because both
pins were on one legal net.
