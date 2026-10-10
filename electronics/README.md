# Electronics

Two boards. The hat is the one that matters; the IMU breakout is a small extra.

## hat

A four-layer hat for the Arduino UNO Q, 68.68 by 53.44 mm, 1.6 mm, ENIG. It
sits under the UNO Q on the two 2x30 board-to-board connectors and carries
everything the robot needs that the UNO Q does not have.

| block | part | notes |
|---|---|---|
| camera | 22-pin 0.5 mm flex connector on CSI0 | takes an IMX219 module. The ten CSI0 tracks are Arduino's own, copied from the UNO Media Carrier and length matched by them |
| power in | 6-pin JST PH pack plug (3 contacts per pole, 6 A), reverse-polarity FET, then two paths | 5.0 to 8.4 V in. Above that the daemon refuses to torque anything |
| UNO Q supply | TPS61088 boost to 11.86 V into the UNO Q's own VIN (7-24 V) | rev C. The hat never touches the UNO Q's 5 V or USB VBUS |
| amplifier supply | PTC, TPS62933 buck to 5 V | feeds only the amplifier |
| servo bus | half-duplex TTL with automatic direction | one 3-pin plug, the chain daisy-chains from there. Serial1, D0 and D1, 1 Mbit/s |
| pack monitor | INA226 across a 5 mOhm shunt in the pack feed | pack voltage and current |
| inertial | LSM6DSV16X on the 3.3 V bus at 0x6B | six axes with on-chip fusion |
| distance | header for a VL53L5CX or VL53L8CX breakout | |
| audio out | PAM8302A | one speaker plug |
| microphone | the carrier's microphone circuit | |
| touch | one capacitive pad | for petting the head |
| foot contacts | two 2-pin plugs | a switch or an FSR to ground per foot |

Everything except the board-to-board headers and the camera connector sits on
the bottom side, so nothing fouls the UNO Q above it.

### Getting one made

Send `hat/fab/gerbers.zip` to a PCB maker. Four layers, 1.6 mm, ENIG. Smallest
via is 0.20 mm drill on a 0.40 mm pad (one, inside the IMU's LGA); the rest are
0.25/0.45. For assembled boards add `fab/bom.csv`
and `fab/cpl.csv`; the part numbers are LCSC.
The order, with options, price and checks: `../docs/PCB-ORDER-LEIF-2026-10-11.md`.
Checks behind it: `hat/checks/` and `hat/docs/` (footprints, strap pins, power
budget, BOM stock).

`hat/schematic.pdf` is the circuit. `hat/kicad/` is the KiCad 10 project.

### Changing it

The board is generated, not drawn by hand. `hat_parts.py` is the whole design:
every part, every net, every position. `sch.py` turns it into the schematic and
`board.py` into the copper. Edit `hat_parts.py` and regenerate; do not edit the
KiCad files and expect the change to survive.

The CSI0 tracks are the exception. They are Arduino's, they are length matched,
and `board.py` locks them so the autorouter cannot touch them.

## imu-board

A two-layer breakout for an inertial sensor on the body plate, for when you
want the sensor away from the head. Same files, same process.

If you do not want to make a board for this, an MPU-6050 module wired straight
to the UNO Q header works. See `../docs/wiring.md`.
