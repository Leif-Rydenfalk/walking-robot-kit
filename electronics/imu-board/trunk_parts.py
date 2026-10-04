"""trunk_parts.py - the the robot trunk IMU board as ONE table: parts and nets.

Read by `sch.py` (the schematic) and `board.py` (the copper). This file IS the
design; neither script adds a connection.

WHY THIS BOARD EXISTS
  The official the robot carries two IMUs, one in the head and one in the body
  (research/01-product-and-specs.md:96). The UNO Q and its hat now live in the
  HEAD, so the hat's LSM6DSV16X is the head IMU. A walking policy wants the
  TRUNK's angular rate and gravity, not the head's, because the head moves on
  two servos relative to the body. This board is that second IMU: the same
  part, the same driver, SA0 to GND so it answers at 0x6A while the head
  answers at 0x6B, on one 4-wire JST-SH cable from the hat's J12.

Frame: mm, origin at the lower-left corner of the outline, +Y up, seen from
the TOP. Everything is on the top side; the bottom is a ground plane and the
glue surface.

Sources:
  ST DS13510 rev 4 LSM6DSV16X, Table 2 (pin description) and section 5.1.1
  (I2C slave address 110101x, SA0 picks the LSb), read 2026-10-04 from the
  LCSC datasheet for C5267406.
"""

R0402 = "Resistor_SMD:R_0402_1005Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
LGA14 = "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y"
SH4V = "Connector_JST:JST_SH_BM04B-SRSS-TB_1x04-1MP_P1.00mm_Vertical"
TP = "TestPoint:TestPoint_Pad_D1.0mm"
HOLE = "MountingHole:MountingHole_2.2mm_M2"

W, H = 18.0, 15.0

# (ref, symbol, footprint, value, LCSC, side, (x, y), rot, fitted, note)
PARTS = [
    ("U1", "local:LSM6DSV16X", LGA14, "LSM6DSV16X", "C5267406",
     "top", (9.0, 6.4), 0, True,
     "trunk IMU, I2C 0x6A (SA0 to GND); same part and driver as the head"),
    ("C1", "Device:C", C0402, "100nF", "C1525", "top", (5.6, 6.4), 0, True, "U1 Vdd_IO"),
    ("C2", "Device:C", C0402, "100nF", "C1525", "top", (12.4, 6.4), 0, True, "U1 Vdd"),
    ("J1", "Connector_Generic_MountingPin:Conn_01x04_MountingPin", SH4V,
     "BM04B-SRSS-TB I2C in", "C160390", "top", (9.0, 11.8), 0, True,
     "1 GND 2 3V3 3 SDA 4 SCL, from the hat's J12"),
    ("TP1", "Connector:TestPoint", TP, "INT1", "", "top", (14.6, 9.0), 0, True,
     "data-ready, not on the 4-wire cable"),
    ("TP2", "Connector:TestPoint", TP, "GND", "", "top", (3.4, 9.0), 0, True, ""),
    ("H1", "Mechanical:MountingHole", HOLE, "M2", "", "top", (2.6, 2.6), 0, True, ""),
    ("H2", "Mechanical:MountingHole", HOLE, "M2", "", "top", (15.4, 2.6), 0, True, ""),
]

NETS = {
    # SA0 (pin 1) to GND makes the address 1101010b = 0x6A (DS13510 sec. 5.1.1).
    # SDx (2) and SCx (3) go to GND because the analog hub and Qvar are unused
    # and Table 2 says those pins must be tied, not floated.
    "GND": ["U1.1", "U1.2", "U1.3", "U1.6", "U1.7", "C1.2", "C2.2",
            "J1.1", "J1.MP", "TP2.1"],
    # CS (12) to Vdd_IO selects I2C (Table 2: 1 = I2C/I3C enabled).
    "+3V3": ["U1.5", "U1.8", "U1.12", "C1.1", "C2.1", "J1.2"],
    "SDA": ["U1.14", "J1.3"],
    "SCL": ["U1.13", "J1.4"],
    "IMU_INT1": ["U1.4", "TP1.1"],
}

# INT2 unused; OCS_Aux and SDO_Aux may be left open when the auxiliary SPI is
# unused (DS13510 Table 2 note 3).
NC = ["U1.9", "U1.10", "U1.11"]

LOCAL_SYMBOLS = {
    "LSM6DSV16X": dict(
        ref_prefix="U",
        cite=("ST DS13510 rev 4 LSM6DSV16X, Table 2 Pin description "
              "(LGA-14L 2.5 x 3.0 x 0.83 mm), read 2026-10-04 from the LCSC "
              "datasheet for C5267406"),
        description="6-axis IMU, I2C/SPI/I3C, on-chip SFLP sensor fusion",
        pins=[
            ("1", "SDO/SA0", "input", "left"),
            ("2", "SDx", "input", "left"),
            ("3", "SCx", "input", "left"),
            ("12", "CS", "input", "left"),
            ("13", "SCL", "input", "left"),
            ("14", "SDA", "bidirectional", "left"),
            ("4", "INT1", "output", "right"),
            ("9", "INT2", "output", "right"),
            ("10", "OCS_Aux", "input", "right"),
            ("11", "SDO_Aux", "output", "right"),
            ("5", "Vdd_IO", "power_in", "right"),
            ("8", "Vdd", "power_in", "right"),
            ("6", "GND", "power_in", "right"),
            ("7", "GND", "power_in", "right"),
        ]),
}


def all_pins():
    seen = {}
    for n, pins in NETS.items():
        for p in pins:
            if p in seen:
                raise SystemExit("%s is on %s and %s" % (p, seen[p], n))
            seen[p] = n
    return seen
