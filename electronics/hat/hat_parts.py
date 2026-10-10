"""hat_parts.py - the UNO Q head hat as ONE table: parts, nets, positions. REV C.

Rev C (2026-10-10, foreman/microduck-assembly.60) fixes the three Rev B faults
found by the 2026-10-07 interface audit (HOLD.md):
  1. IMU_INT1, EXP_INT and INA_ALERT drove 3.3 V into 1.8 V SoC pins
     J2.37/39/41. They now land on MCU pins J2.3 PC7, J2.7 PC9, J2.9 PE4,
     3.3 V domain (datasheet ABX00162 sec. 9, JMISC table).
  2. The TPS62933 drove JMISC 54/56, which Arduino's schematic shows on USB
     VBUS ahead of D2801. The hat no longer touches JMISC 54/56. A TPS61088
     boost makes 11.9 V from the 2S pack and feeds the UNO Q's own 7-24 V VIN
     input (JMEDIA 57/59), where Arduino's buck and Schottky OR it with USB-C.
     The TPS62933 now feeds only the hat's own 5 V (the amplifier).
  3. The servo bus pull-ups (R15, R21) hung on +3V3 and back-fed it from a
     powered bus. D40 (BAT54H) now isolates them.

Read by `sch.py` (the schematic, system python) and `board.py` (the copper,
KiCad's python). This file IS the design; neither script adds a connection.

Frame: mm, origin at the lower-left corner of the UNO outline, +Y up, seen
from the TOP (the side that faces the UNO Q). The KiCad frame of the board
is the same with +Y down, and it equals the UNO Media Carrier's frame shifted
by (-114.211, -78.334), so Arduino's own coordinates carry over unchanged.

Sources (all in ref/, fetched 2026-10-02):
  UNO Q datasheet ABX00162-ABX00173 sec. 9.1 / 9.2 (JMISC / JMEDIA pin maps)
  UNO Media Carrier ASX00083 Altium files, imported to ref/mediacarrier.kicad_pcb
  out/carrier-pins.json  every JMEDIA / JMISC / FFC pad net Arduino drew
  rev-c/robot-hat-revc/board.py  the servo-bus circuit (Feetech auto direction)
"""

R0402 = "Resistor_SMD:R_0402_1005Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
C1206 = "Capacitor_SMD:C_1206_3216Metric"
C1210 = "Capacitor_SMD:C_1210_3225Metric"
JHDR = "unoq_hat:UNOQ_JMEDIA_2x30_P1.27_Male"
TP = "TestPoint:TestPoint_Pad_D1.0mm"
HOLE = "MountingHole:MountingHole_3.2mm_M3"
LGA14 = "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y"
VSSOP10 = "Package_SO:TSSOP-10_3x3mm_P0.5mm"
SOT236 = "Package_TO_SOT_SMD:SOT-23-6"
R2512 = "Resistor_SMD:R_2512_6332Metric"
SH2 = ("Connector_JST:JST_SH_SM02B-SRSS-TB_1x02-1MP_P1.00mm_Horizontal")
SH4V = ("Connector_JST:JST_SH_BM04B-SRSS-TB_1x04-1MP_P1.00mm_Vertical")

# Symbols KiCad 10 does not carry. Each pin row is read off the datasheet
# named in `cite`, never from memory; `sch.py` hands these to Schematic.define().
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
    "TTP233H-BA6": dict(
        ref_prefix="U",
        cite=("TonTouch TTP233H-BA6 datasheet v1.0 2020/12/30, Pin Description "
              "table p.2, read 2026-10-04 from the LCSC datasheet for C2890349"),
        description="1-key capacitive touch pad detector, SOT-23-6",
        pins=[
            ("3", "I", "input", "left"),
            ("4", "AHLB", "input", "left"),
            ("6", "TOG", "input", "left"),
            ("1", "Q", "output", "right"),
            ("5", "VDD", "power_in", "right"),
            ("2", "VSS", "power_in", "right"),
        ]),

    "TPS61088": dict(
        ref_prefix="U",
        cite=("TI TPS61088 datasheet ZHCSDP8A (May 2015), section 6 Pin "
              "Configuration and Functions, RHL VQFN-20, read 2026-10-10 from "
              "the LCSC datasheet for C87357"),
        description="10 A synchronous boost, 2.7-12 V in, 4.5-12.6 V out",
        pins=[
            ("9", "VIN", "power_in", "left"),
            ("2", "EN", "input", "left"),
            ("3", "FSW", "input", "left"),
            ("10", "SS", "passive", "left"),
            ("13", "MODE", "input", "left"),
            ("1", "VCC", "power_out", "left"),
            ("20", "AGND", "power_in", "left"),
            ("21", "PGND", "power_in", "left"),
            ("11", "NC", "passive", "left"),
            ("12", "NC", "passive", "left"),
            ("4", "SW", "passive", "right"),
            ("5", "SW", "passive", "right"),
            ("6", "SW", "passive", "right"),
            ("7", "SW", "passive", "right"),
            ("8", "BOOT", "passive", "right"),
            ("14", "VOUT", "power_out", "right"),
            ("15", "VOUT", "passive", "right"),
            ("16", "VOUT", "passive", "right"),
            ("17", "FB", "input", "right"),
            ("18", "COMP", "passive", "right"),
            ("19", "ILIM", "passive", "right"),
        ]),
}

W, H = 68.58, 53.34

# (ref, symbol, footprint, value, LCSC, side, (x, y), rot, fitted, note)
# side "top" faces the UNO Q: only the two board-to-board headers live there.
# Every other part is on the bottom, so nothing has to fit under the UNO Q.
PARTS = [
    # --- board to board: Arduino's footprint, BOOMELE stock part ------------
    ("J1", "Connector_Generic:Conn_02x30_Odd_Even", JHDR,
     "JMEDIA 2x30 1.27 SMD male (GPEC212-3002A011C1AF equiv.)", "C191721",
     "top", (11.6841, 26.6704), -90, True,
     "Arduino carrier JMEDIA_MALE position, exactly"),
    ("J2", "Connector_Generic:Conn_02x30_Odd_Even", JHDR,
     "JMISC 2x30 1.27 SMD male (GPEC212-3002A011C1AF equiv.)", "C191721",
     "top", (58.5471, 26.6704), -90, True,
     "Arduino carrier JMISC_MALE position, exactly"),
    # --- camera ---------------------------------------------------------------
    ("J3", "Connector_Generic_Shielded:Conn_01x22_Shielded",
     "unoq_hat:FFC_22P_P0.5_TF31-22S", "TF31-22S-0.5SH 22p 0.5 FFC (IMX219)",
     "C3169786", "bottom", (17.69009, 4.7704), 180, True,
     "Arduino carrier J3_2 position; CSI0 tracks copied from the carrier"),
    ("U1", "Interface:TCA9406DC", "Package_SO:VSSOP-8_2.3x2mm_P0.5mm",
     "TCA9406DCUR", "C840107", "bottom", (32.0, 14.0), 0, True,
     "CCI0 1.8 V <-> 3.3 V, same as carrier U2_2"),
    ("R1", "Device:R", R0402, "10k", "C25744", "bottom", (28.6, 15.6), 0, True, "U1 OE pull-up to 1V8"),
    ("C1", "Device:C", C0402, "4.7uF", "C23733", "bottom", (28.6, 14.4), 0, True, "U1 OE delay (Arduino)"),
    ("C2", "Device:C", C0402, "100nF", "C1525", "bottom", (28.6, 12.4), 0, True, "U1 VCCA"),
    ("C3", "Device:C", C0402, "100nF", "C1525", "bottom", (35.4, 14.0), 0, True, "U1 VCCB"),
    ("U2", "Interface_Expansion:TCA9555PWR",
     "Package_SO:TSSOP-24_4.4x7.8mm_P0.65mm", "TCA9555PWR", "C465732",
     "bottom", (40.5, 16.0), 90, True,
     "0x26 on the 3.3 V CCI0 bus: camera enable, ToF LPn"),
    ("C5", "Device:C", C0402, "100nF", "C1525", "bottom", (40.5, 21.3), 0, True, "U2 VCC"),
    ("R2", "Device:R", R0402, "10k", "C25744", "bottom", (34.8, 18.0), 0, True, "U2 INT pull-up"),
    ("C4", "Device:C", C0805, "10uF 16V", "C1713", "bottom", (28.6, 9.6), 0, True, "3V3 bulk at camera + ToF"),
    # --- ToF (VL53L5CX / VL53L8CX breakout, rev-c pinout) -------------------
    ("J4", "Connector_Generic_MountingPin:Conn_01x06_MountingPin",
     "Connector_JST:JST_SH_BM06B-SRSS-TB_1x06-1MP_P1.00mm_Vertical",
     "BM06B-SRSS-TB ToF", "C160392", "bottom", (34.5, 3.4), 0, True,
     "1 GND 2 3V3 3 SDA 4 SCL 5 INT 6 LPn"),
    ("R3", "Device:R", R0402, "0R", "C17168", "bottom", (32.0, 8.6), 0, True, "ToF SDA on CCI0 (3.3 V side)"),
    ("R4", "Device:R", R0402, "0R", "C17168", "bottom", (32.0, 7.4), 0, True, "ToF SCL on CCI0 (3.3 V side)"),
    ("R5", "Device:R", R0402, "0R", "C17168", "bottom", (35.4, 8.6), 0, False, "ToF SDA on MCU I2C4 (DNP)"),
    ("R6", "Device:R", R0402, "0R", "C17168", "bottom", (35.4, 7.4), 0, False, "ToF SCL on MCU I2C4 (DNP)"),
    ("R7", "Device:R", R0402, "10k", "C25744", "bottom", (39.0, 8.6), 0, True, "ToF LPn pull-up"),
    ("R8", "Device:R", R0402, "10k", "C25744", "bottom", (39.0, 7.4), 0, True, "ToF INT pull-up"),
    # --- microphone (Arduino carrier R34 / C12) -------------------------------
    ("J8", "Connector_Generic_MountingPin:Conn_01x02_MountingPin",
     "Connector_JST:JST_SH_SM02B-SRSS-TB_1x02-1MP_P1.00mm_Horizontal",
     "SM02B-SRSS-TB electret mic", "C160402", "bottom", (45.0, 3.8), 180, True,
     "1 MIC+ 2 GND"),
    ("R14", "Device:R", R0402, "2.2k", "C25879", "bottom", (45.0, 8.6), 0, True, "mic bias (carrier R34)"),
    ("C19", "Device:C", C0402, "100nF", "C1525", "bottom", (45.0, 9.8), 0, True, "bias decoupling (carrier C12)"),
    # --- servo bus (rev-c circuit, Feetech auto direction) --------------------
    ("U5", "74xGxx:74LVC1G126", "Package_TO_SOT_SMD:SOT-23-5", "SN74LVC1G126DBVR",
     "C7834", "bottom", (49.5, 15.0), 0, True, "TX driver, OE high"),
    ("U6", "74xGxx:74LVC1G125", "Package_TO_SOT_SMD:SOT-23-5", "SN74LVC1G125DBVR",
     "C23654", "bottom", (49.5, 10.5), 0, True, "RX buffer, OE low"),
    ("Q1", "Transistor_BJT:MMBT3906", "Package_TO_SOT_SMD:SOT-23", "MMBT3906",
     "C727128", "bottom", (55.0, 15.0), 0, True, "auto direction (Feetech HD-1910 p.7)"),
    ("JP1", "Jumper:SolderJumper_3_Bridged12",
     "Jumper:SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm",
     "TXEN source: 1-2 AUTO, 2-3 GPIO PE7", "", "bottom", (55.5, 10.5), 0, True,
     "ships bridged 1-2"),
    ("R15", "Device:R", R0402, "10k", "C25744", "bottom", (46.4, 16.4), 0, True, "bus pull-up"),
    ("R16", "Device:R", R0402, "10k", "C25744", "bottom", (46.4, 13.6), 0, True, "TXEN pull-down"),
    ("R17", "Device:R", R0402, "10k", "C25744", "bottom", (46.4, 11.4), 0, True, "RX pull-up"),
    ("R18", "Device:R", R0402, "10k", "C25744", "bottom", (59.0, 16.4), 0, True, "Q1 base"),
    ("R19", "Device:R", R0402, "20k", "C25765", "bottom", (59.0, 13.6), 0, True, "TXEN_AUTO pull-down"),
    ("R20", "Device:R", R0402, "33R", "C138002", "bottom", (52.0, 19.0), 0, True, "bus series"),
    ("R21", "Device:R", R0402, "1k", "C106235", "bottom", (46.4, 18.6), 0, True, "bus pull-up, FITTED (rev B lesson)"),
    ("D1", "Diode:BZT52Bxx", "Diode_SMD:D_SOD-323", "BZT52C5V1S", "C151348",
     "bottom", (55.5, 19.0), 0, True, "bus clamp"),
    ("D40", "Device:D_Schottky", "unoq_hat:BAT54H_Nexperia_SOD123F", "BAT54H,115",
     "C426769", "bottom", (50.5, 21.0), 0, True,
     "rev C: K=1 feeds R15/R21 only, A=2 on +3V3, so a powered servo bus cannot back-feed the UNO Q rail"),
    ("C20", "Device:C", C0402, "100nF", "C1525", "bottom", (52.4, 15.0), 90, True, "U5 VCC"),
    ("C21", "Device:C", C0402, "100nF", "C1525", "bottom", (52.4, 10.5), 90, True, "U6 VCC"),
    # --- power: 2S pack -> reverse FET -> PTC -> TPS62933 -> 5V_SYS -----------
    # rev C 2026-10-10: 6 pins, 3 per pole. Two PH contacts (4 A) were below the
    # 4.7 A worst case (UNO Q at its 15 W limit + amp at full, 5.0 V pack);
    # three are 6 A. Same JST PH family and crimp, so the kit harness stays PH.
    ("J6", "Connector_Generic_MountingPin:Conn_01x06_MountingPin",
     "Connector_JST:JST_PH_S6B-PH-SM4-TB_1x06-1MP_P2.00mm_Horizontal",
     "S6B-PH-SM4-TB 2S pack in", "C265405", "bottom", (44.5, 47.6), 0, True,
     "1 2 3 VBAT, 4 5 6 GND"),
    ("Q2", "Transistor_FET:Si7141DP", "Package_SO:PowerPAK_SO-8_Single",
     "AON6407 reverse-polarity P-FET", "C13899", "bottom", (44.5, 36.5), 0, True,
     "D from pack, S to load, G to GND"),
    ("R11", "Device:R", R0402, "100k", "C25741", "bottom", (49.5, 35.0), 0, True, "Q2 gate to GND"),
    ("F1", "Device:Fuse", "Fuse:Fuse_1812_4532Metric", "SMD1812P300TF/16 3A PTC",
     "C702820", "bottom", (53.0, 38.5), 90, True, "converter feed only"),
    ("U3", "Regulator_Switching:TPS62933", "Package_TO_SOT_SMD:SOT-583-8",
     "TPS62933DRLR", "C3200405", "bottom", (53.0, 29.0), 180, True,
     "3.8-30 V in, 5 V 3 A out; RT float 500 kHz, EN float on"),
    ("C6", "Device:C", C1206, "10uF 25V", "C77090", "bottom", (57.5, 33.0), 0, True, "VIN"),
    ("C7", "Device:C", C1206, "10uF 25V", "C77090", "bottom", (57.5, 29.0), 90, True, "VIN"),
    ("C8", "Device:C", C0402, "100nF 25V", "C105883", "bottom", (55.6, 29.0), 90, True, "VIN HF"),
    ("C9", "Device:C", C0402, "100nF 25V", "C105883", "bottom", (50.6, 25.6), 0, True, "BST"),
    ("C10", "Device:C", C0402, "10nF", "C60133", "bottom", (50.4, 31.8), 0, True, "soft start"),
    ("R12", "Device:R", R0402, "523k", "C27011", "bottom", (50.4, 30.6), 0, True, "FB top"),
    ("R13", "Device:R", R0402, "100k", "C25741", "bottom", (50.4, 29.4), 0, True, "FB bottom: 0.8*(1+5.23)=4.98 V"),
    ("L1", "Device:L", "unoq_hat:FNR6045_6.0x6.0mm", "FNR6045S6R8MT 6.8uH 3.3A",
     "C168072", "bottom", (45.0, 27.5), 180, True, "4.5 mm tall"),
    ("C11", "Device:C", C1210, "22uF 25V", "C52306", "bottom", (38.6, 29.6), 90, True, "VOUT"),
    ("C12", "Device:C", C1210, "22uF 25V", "C52306", "bottom", (38.6, 24.6), 90, True, "VOUT"),
    # --- servo connector --------------------------------------------------------
    ("J5", "Connector_Generic_MountingPin:Conn_01x03_MountingPin",
     "Connector_JST:JST_PH_S3B-PH-SM4-TB_1x03-1MP_P2.00mm_Horizontal",
     "S3B-PH-SM4-TB servo bus", "C265101", "bottom", (30.0, 47.6), 0, True,
     "1 DATA 2 VBAT 3 GND (rev-c order)"),
    # --- audio out ----------------------------------------------------------------
    ("U4", "Amplifier_Audio:PAM8302AAS", "Package_SO:MSOP-8_3x3mm_P0.65mm",
     "PAM8302AASCR", "C113367", "bottom", (59.5, 22.0), 0, True,
     "2.5 W class D from LINEOUT"),
    ("R9", "Device:R", R0402, "100k", "C25741", "bottom", (54.4, 23.4), 0, True, "AMP SD pull-down: off until PC6 says on"),
    ("C13", "Device:C", C0402, "100nF", "C1525", "bottom", (61.0, 24.6), 0, True, "U4 VDD"),
    ("C16", "Device:C", C0805, "10uF 16V", "C1713", "bottom", (64.0, 21.0), 90, True, "U4 VDD bulk"),
    ("C14", "Device:C", C0402, "100nF", "C1525", "bottom", (54.4, 22.0), 0, True, "IN+ coupling"),
    ("C15", "Device:C", C0402, "100nF", "C1525", "bottom", (54.4, 20.6), 0, True, "IN- coupling"),
    ("J7", "Connector_Generic_MountingPin:Conn_01x02_MountingPin",
     "Connector_JST:JST_PH_S2B-PH-SM4-TB_1x02-1MP_P2.00mm_Horizontal",
     "S2B-PH-SM4-TB speaker", "C295747", "bottom", (58.0, 47.6), 0, True,
     "1 SPK+ 2 SPK-"),
    # --- head IMU (rev B): LSM6DSV16X on the 3.3 V CCI0 bus, 0x6B -----------
    # At (26, 30) with 3 mm to each decoupling cap, 19 mm from L1 and 34 mm
    # from the PAM8302, the two parts here that move current fast enough to
    # couple into a 2.5 x 3 LGA. SA0 to 3V3 makes it 0x6B so the trunk board
    # (SA0 to GND) is 0x6A and both answer on one bus (DS13510 sec. 5.1.1).
    #
    # It went to (18.5, 24) for one build on 2026-10-04 because a PAD scan of
    # x 14-24, y 17-29 came back empty. That rectangle holds the pre-routed
    # CSI0 camera lanes: 306 track segments and 22 vias. The build came back
    # with eight shorts, a hole clearance and a solder mask bridge. An empty
    # pad scan is not an empty region, which is why placecheck now has
    # --copper, and why this spot was checked with it (5 nets, a handful of
    # segments, no pads) before the part was moved back.
    ("U7", "local:LSM6DSV16X", LGA14, "LSM6DSV16X", "C5267406",
     "bottom", (26.0, 30.0), 0, True,
     "head IMU, I2C 0x6B on CCI0; SFLP gives gravity + game rotation vector in hardware"),
    ("C22", "Device:C", C0402, "100nF", "C1525", "bottom", (23.0, 30.0), 0, True, "U7 Vdd_IO"),
    ("C23", "Device:C", C0402, "100nF", "C1525", "bottom", (29.0, 30.0), 0, True, "U7 Vdd"),
    ("R27", "Device:R", R0402, "4.7k", "C25900", "bottom", (23.0, 31.3), 0, True,
     "CCI0 SDA pull-up: the bus now leaves the board to the trunk IMU"),
    ("R28", "Device:R", R0402, "4.7k", "C25900", "bottom", (23.0, 32.4), 0, True, "CCI0 SCL pull-up"),
    ("J12", "Connector_Generic_MountingPin:Conn_01x04_MountingPin", SH4V,
     "BM04B-SRSS-TB trunk IMU I2C", "C160390", "bottom", (6.0, 30.0), 90, True,
     "1 GND 2 3V3 3 SDA 4 SCL to the trunk IMU board (0x6A)"),
    # --- pack monitor (rev B): INA226 across a 5 mOhm shunt in the pack feed --
    # 10 A peak from six servos: 50 mV across the shunt, inside the INA226's
    # +-81.92 mV range, 0.5 W in a 2 W 2512. Vbus reads the pack after the shunt.
    ("R22", "Device:R", R2512, "5mR 2W shunt", "C185817", "bottom", (36.0, 35.7), 0, True,
     "pack current shunt, Q2 source to the VBAT plane; pad 2 east is the high side"),
    ("U8", "Sensor_Energy:INA226", VSSOP10, "INA226AIDGSR", "C49851",
     "bottom", (35.5, 40.0), 0, True,
     "pack volts + amps, I2C 0x40 on CCI0 (A0, A1 to GND)"),
    ("C26", "Device:C", C0402, "100nF", "C1525", "bottom", (39.9, 40.8), 0, True, "U8 VS, beside pins 6 and 7"),
    ("R29", "Device:R", R0402, "10k", "C25744", "bottom", (30.5, 39.0), 0, True,
     "U8 ALERT pull-up (open drain)"),
    # --- capacitive touch (rev B): one pad for petting the head ---------------
    ("U9", "local:TTP233H-BA6", SOT236, "TTP233H-BA6", "C2890349",
     "bottom", (60.0, 4.5), 0, True,
     "AHLB and TOG to GND = active-high direct output into the expander"),
    ("J9", "Connector_Generic_MountingPin:Conn_01x02_MountingPin", SH2,
     "SM02B-SRSS-TB touch pad", "C160402", "bottom", (52.0, 3.6), 180, True,
     "1 pad foil, 2 GND"),
    ("R24", "Device:R", R0402, "1k", "C106235", "bottom", (56.3, 4.5), 0, True,
     "touch sense series, ESD limiting"),
    ("C24", "Device:C", C0402, "10pF C0G (DNP)", "", "bottom", (56.3, 6.0), 0, False,
     "sensitivity trim, 1-50 pF; DNP, fit by hand only if the pad is too sensitive"),
    ("C25", "Device:C", C0402, "100nF", "C1525", "bottom", (60.0, 8.0), 0, True, "U9 VDD"),
    # --- foot contacts (rev B): one 2-pin plug per foot, switch or FSR to GND -
    # On the WEST edge, clear of the camera FFC and of Arduino's CSI0 copper:
    # the only free run of board edge wide enough for two side-entry housings.
    ("J10", "Connector_Generic_MountingPin:Conn_01x02_MountingPin", SH2,
     "SM02B-SRSS-TB left foot contact", "C160402", "bottom", (5.0, 10.0), 90, True,
     "1 FOOT_L, 2 GND; closes to GND"),
    ("J11", "Connector_Generic_MountingPin:Conn_01x02_MountingPin", SH2,
     "SM02B-SRSS-TB right foot contact", "C160402", "bottom", (5.0, 17.0), 90, True,
     "1 FOOT_R, 2 GND; closes to GND"),
    ("R25", "Device:R", R0402, "10k", "C25744", "bottom", (42.8, 21.0), 0, True,
     "FOOT_L pull-up, at the expander it feeds"),
    ("R26", "Device:R", R0402, "10k", "C25744", "bottom", (45.0, 21.0), 0, True,
     "FOOT_R pull-up, at the expander it feeds"),
    # --- rev C: 2S pack -> F2 -> TPS61088 boost 11.9 V -> UNO Q VIN (JMEDIA 57/59) --
    # In the free north-west corner of the bottom side (B.Cu was empty there in
    # rev B, measured 2026-10-10 from kicad/unoq_hat.kicad_pcb). Values from
    # TPS61088 datasheet sec. 9.2.2 and the TPS61088EVM-677 BOM (SLVUAF2):
    #   VOUT = 1.204 * (1 + 1M/113k) = 11.86 V      (EVM table 2, 12 V row)
    #   fsw  ~ 530 kHz with RFREQ 300k at 6-8.4 V in (eq. 2, CFREQ 23 pF, tDELAY 89 ns)
    #   ILIM = 1190000 / 150k = 7.9 A typ, 6.6 A worst (eq. 3, PFM, -1.3 A)
    #   peak inductor current at 5.5 V in, 18 W out, 90 %: 3.64 A DC + 1.28 A = 4.9 A
    #   COMP 18k + 6.8 nF for fc 10 kHz (eq. 18, 19; Co 36 uF effective)
    #   SS 47 nF -> 11 ms ramp (eq. 1)
    # Bottom side: at rot 0 a two-pad part's pad 1 lands WEST and U10's SW row
    # lands west (measured with padcheck on the placed board, 2026-10-10), so
    # U10, L2, F2 and C37-C39 are turned 180 to put SW toward L2, VOUT toward
    # C37-C39 and pad 1 of each into its pour.
    ("U10", "local:TPS61088", "Package_DFN_QFN:Texas_VQFN-RHL-20",
     "TPS61088RHLR", "C87357", "bottom", (12.5, 41.0), 180, True,
     "boost for UNO Q VIN; EN tied to VCC (abs max 7 V), MODE floating = PFM at light load"),
    ("L2", "Device:L", "Inductor_SMD:L_Changjiang_FXL0630", "FXL0630-2R2-M 2.2uH",
     "C167218", "bottom", (19.8, 41.0), 180, True, "Isat above the 7.9 A typ switch limit"),
    # 5 A hold: the UNO Q may draw 3 A at 5 V (datasheet sec. 3.1), which is
    # 3.3 A out of an empty 6.0 V pack after both converters; a 3 A PTC would
    # sit in its trip band there.
    ("F2", "Device:Fuse", "Fuse:Fuse_1812_4532Metric", "JK-MSMD500L-12V 5A PTC",
     "C2844238", "bottom", (26.3, 35.3), 180, True,
     "boost feed; a boost has no output disconnect, so a VIN short is stopped here"),
    ("C30", "Device:C", C1210, "22uF 25V", "C52306", "bottom", (19.4, 47.2), 270, True, "boost VIN bulk"),
    ("C31", "Device:C", C1210, "22uF 25V", "C52306", "bottom", (22.7, 47.2), 90, True, "boost VIN bulk"),
    ("C32", "Device:C", C0402, "100nF 25V", "C105883", "bottom", (15.1, 37.0), 90, True, "U10 VIN HF"),
    ("C33", "Device:C", "Capacitor_SMD:C_0603_1608Metric", "4.7uF 16V", "C19666",
     "bottom", (12.8, 45.1), 0, True, "U10 VCC LDO (>1 uF)"),
    ("C34", "Device:C", C0402, "100nF 25V", "C105883", "bottom", (16.8, 36.0), 0, True, "U10 BOOT-SW"),
    ("C35", "Device:C", C0402, "47nF", "C82219", "bottom", (13.0, 36.6), 0, True, "U10 soft start"),
    ("R30", "Device:R", R0402, "300k", "C25774", "bottom", (15.8, 45.2), 0, True, "U10 FSW-SW, ~530 kHz"),
    ("R31", "Device:R", R0402, "150k", "C25755", "bottom", (10.5, 44.8), 270, True, "U10 ILIM, 7.9 A typ"),
    ("R32", "Device:R", R0402, "1M", "C26083", "bottom", (8.1, 45.1), 0, True, "U10 FB top"),
    ("R33", "Device:R", R0402, "113k", "C25746", "bottom", (8.1, 46.3), 0, True, "U10 FB bottom: 11.86 V"),
    ("R34", "Device:R", R0402, "18k", "C25762", "bottom", (9.55, 46.5), 270, True, "U10 COMP R"),
    # EN: VCC is the output of an LDO that the EN pin itself turns on (TPS61088
    # block diagram: LDO under "Shutdown"; ISD 1 uA), so EN tied to VCC never
    # starts. EN is rated 7 V abs max and the pack reaches 8.4 V: a 1:1 divider
    # gives 2.5 V at 5.0 V (VENH 1.2 V) and 4.2 V at 8.4 V. 42 uA at full pack.
    ("R35", "Device:R", R0402, "100k", "C25741", "bottom", (16.3, 46.55), 0, True, "U10 EN divider top (VBAT_B)"),
    ("R36", "Device:R", R0402, "100k", "C25741", "bottom", (13.4, 46.55), 0, True, "U10 EN divider bottom"),
    ("C36", "Device:C", C0402, "6.8nF", "C1542", "bottom", (9.55, 48.4), 270, True, "U10 COMP C"),
    ("C37", "Device:C", C1210, "22uF 25V", "C52306", "bottom", (7.0, 42.6), 180, True, "boost VOUT"),
    ("C38", "Device:C", C1210, "22uF 25V", "C52306", "bottom", (7.0, 39.3), 180, True, "boost VOUT"),
    ("C39", "Device:C", C1210, "22uF 25V", "C52306", "bottom", (7.0, 36.0), 180, True, "boost VOUT"),
    ("TP7", "Connector:TestPoint", TP, "VIN12", "", "bottom", (10.2, 20.0), 0, True, "UNO Q VIN"),
    # --- test points and holes (not in the BOM) ---------------------------------
    ("TP1", "Connector:TestPoint", TP, "MCLK0", "", "bottom", (20.5, 32.8), 0, True, "SoC camera clock, 1.8 V"),
    ("TP2", "Connector:TestPoint", TP, "V5", "", "bottom", (35.5, 25.0), 0, True, ""),
    ("TP3", "Connector:TestPoint", TP, "VBAT", "", "bottom", (49.5, 40.5), 0, True, ""),
    ("TP4", "Connector:TestPoint", TP, "BUS", "", "bottom", (58.5, 19.0), 0, True, ""),
    ("TP5", "Connector:TestPoint", TP, "GND", "", "bottom", (35.5, 31.0), 0, True, ""),
    ("TP6", "Connector:TestPoint", TP, "EXP_INT", "", "bottom", (36.0, 22.0), 0, True, ""),
    # UNO R3 / UNO Q hole pattern, 3.2 mm (UNO Q datasheet: 4x R1.6)
    # H1 (13.97, 2.54) is NOT drilled: Arduino's camera FFC J3 sits on it (x 9.3-26.0, y 0.7-7.3).
    ("H2", "Mechanical:MountingHole", HOLE, "M3", "", "top", (15.24, 50.80), 0, True, ""),
    ("H3", "Mechanical:MountingHole", HOLE, "M3", "", "top", (66.04, 7.62), 0, True, ""),
    ("H4", "Mechanical:MountingHole", HOLE, "M3", "", "top", (66.04, 35.56), 0, True, ""),
]

GND_J1 = [1, 2, 7, 8, 13, 14, 19, 20, 25, 26, 31, 32, 37, 38, 43, 44, 49, 50, 55, 56]
GND_J2 = [26, 27, 35, 44, 58, 31, 40]      # 31 MIC2_INM, 40 HPH_REF: Arduino ties both to GND


def _p(ref, *nums):
    return ["%s.%s" % (ref, n) for n in nums]


NETS = {
    "GND": (_p("J1", *GND_J1) + _p("J2", *GND_J2)
            + _p("J3", 1, 4, 7, 10, 13, 16, 19, "SH")
            + ["U1.2", "C1.2", "C2.2", "C3.2", "C4.2", "U2.12", "U2.21", "C5.2",
               "J4.1", "J4.MP", "J8.2", "J8.MP", "C19.2",
               "U5.3", "U6.3", "D1.2", "R16.2", "R19.2", "C20.2", "C21.2",
               "J6.4", "J6.5", "J6.6", "J6.MP", "R11.2", "U3.4", "C6.2", "C7.2", "C8.2",
               "C10.2", "R13.2", "C11.2", "C12.2", "J5.3", "J5.MP",
               "U4.7", "R9.2", "C13.2", "C16.2", "J7.MP", "TP5.1",
               # rev B
               "U7.2", "U7.3", "U7.6", "U7.7", "C22.2", "C23.2",
               "J12.1", "J12.MP",
               "U8.1", "U8.2", "U8.7", "C26.2",
               "U9.2", "U9.4", "U9.6", "C25.2", "C24.2",
               "J9.2", "J9.MP", "J10.2", "J10.MP", "J11.2", "J11.MP",
               # rev C boost
               "U10.11", "U10.12", "U10.20", "U10.21", "R36.1", "C30.2", "C31.1", "C32.2",
               "C33.2", "C35.2", "R31.2", "R33.2", "C36.2", "C37.2", "C38.2", "C39.2"]),
    "+3V3": (_p("J1", 58, 60) + _p("J2", 53, 55)
             + ["J3.22", "U1.7", "C3.1", "C4.1", "U2.24", "U2.2", "U2.3", "C5.1",
                "R2.2", "J4.2", "R7.2", "R8.2",
                "U5.5", "U6.5", "Q1.2", "D40.2", "R17.2", "C20.1", "C21.1",
                # rev B
                "U7.1", "U7.5", "U7.8", "U7.12", "C22.1", "C23.1",
                "R27.2", "R28.2", "J12.2",
                "U8.6", "C26.1", "R29.2",
                "U9.5", "C25.1", "R25.2", "R26.2"]),
    "+1V8": ["J2.57", "U1.3", "R1.2", "C2.1"],
    # rev C: JMISC 54/56 are left open. The hat's 5 V feeds only the amplifier.
    "V5": ["L1.2", "C11.1", "C12.1", "R12.1", "U4.6", "C13.1",
           "C16.1", "TP2.1"],
    # camera: CSI0 exactly as the carrier, CCI0 through U1
    "CSI0_D0_N": ["J1.21", "J3.2"], "CSI0_D0_P": ["J1.23", "J3.3"],
    "CSI0_D1_N": ["J1.27", "J3.5"], "CSI0_D1_P": ["J1.29", "J3.6"],
    "CSI0_CK_N": ["J1.33", "J3.8"], "CSI0_CK_P": ["J1.35", "J3.9"],
    "CSI0_D2_N": ["J1.39", "J3.11"], "CSI0_D2_P": ["J1.41", "J3.12"],
    "CSI0_D3_N": ["J1.45", "J3.14"], "CSI0_D3_P": ["J1.47", "J3.15"],
    "CCI_SCL_1V8": ["J1.51", "U1.5"],
    "CCI_SDA_1V8": ["J1.53", "U1.4"],
    "CAM_MCLK0": ["J1.16", "TP1.1"],
    "U1_OE": ["U1.6", "R1.1", "C1.1"],
    "CCI_SCL": ["U1.8", "J3.20", "U2.22", "R4.1", "U7.13", "U8.5", "R28.1", "J12.4"],
    "CCI_SDA": ["U1.1", "J3.21", "U2.23", "R3.1", "U7.14", "U8.4", "R27.1", "J12.3"],
    "CAM_EN": ["U2.4", "J3.17"],
    "CAM_IO1": ["U2.5", "J3.18"],
    "EXP_INT": ["U2.1", "R2.1", "TP6.1", "J2.7"],
    # ToF
    "TOF_SDA": ["J4.3", "R3.2", "R5.2"],
    "TOF_SCL": ["J4.4", "R4.2", "R6.2"],
    "MCU_SDA": ["J2.18", "R5.1"],
    "MCU_SCL": ["J2.16", "R6.1"],
    "TOF_INT": ["J4.5", "J2.14", "R8.1"],
    "TOF_LPN": ["J4.6", "U2.6", "R7.1"],
    # mic
    "MIC_INP": ["J2.29", "J8.1", "R14.2"],
    "MIC_BIAS": ["J2.33", "R14.1", "C19.1"],
    # servo bus
    "SERVO_TX": ["J2.21", "U5.2", "R18.1"],
    "SERVO_RX": ["J2.17", "U6.4", "R17.1"],
    "TXEN_GPIO": ["J2.12", "JP1.3"],
    "TXEN": ["JP1.2", "U5.1", "U6.1", "R16.1"],
    "TXEN_AUTO": ["JP1.1", "Q1.3", "R19.1"],
    "Q1_B": ["Q1.1", "R18.2"],
    "DATA_BUF": ["U5.4", "U6.2", "R20.1", "R15.1", "R21.1"],
    "SERVO_PULLUP": ["D40.1", "R15.2", "R21.2"],
    "SERVO_DATA": ["J5.1", "R20.2", "D1.1", "TP4.1"],
    # power
    "VBAT_IN": ["J6.1", "J6.2", "J6.3", "Q2.5"],
    # rev B: the shunt sits between the FET and everything it feeds, so the
    # INA226 sees the converter and the servos together. Q2 source is at
    # x 41.83 and R22 pad 2 at x 38.96, a 2.9 mm hop (measured 2026-10-04).
    "VBAT_FET": ["Q2.1", "Q2.2", "Q2.3", "R22.2", "U8.10"],
    "VBAT": ["R22.1", "F1.1", "J5.2", "TP3.1", "U8.9", "U8.8", "F2.1"],
    # rev C boost
    "VBAT_B": ["F2.2", "L2.1", "C30.1", "C31.2", "U10.9", "C32.1", "R35.2"],
    "SW2": ["L2.2", "U10.4", "U10.5", "U10.6", "U10.7", "C34.2", "R30.2"],
    "BST_BOOT": ["U10.8", "C34.1"],
    "BST_FSW": ["U10.3", "R30.1"],
    "BST_VCC": ["U10.1", "C33.1"],
    "BST_EN": ["U10.2", "R35.1", "R36.2"],
    "BST_SS": ["U10.10", "C35.1"],
    "BST_FB": ["U10.17", "R32.2", "R33.1"],
    "BST_COMP": ["U10.18", "R34.1"],
    "BST_COMPRC": ["R34.2", "C36.1"],
    "BST_ILIM": ["U10.19", "R31.1"],
    "VIN12": ["U10.14", "U10.15", "U10.16", "C37.1", "C38.1", "C39.1", "R32.1",
              "J1.57", "J1.59", "TP7.1"],
    "Q2_G": ["Q2.4", "R11.1"],
    "VBAT_F": ["F1.2", "U3.3", "C6.1", "C7.1", "C8.1"],
    "SW": ["U3.5", "L1.1", "C9.2"],
    "BST": ["U3.6", "C9.1"],
    "SS": ["U3.7", "C10.1"],
    "FB": ["U3.8", "R12.2", "R13.1"],
    # rev B sensors
    "IMU_INT1": ["U7.4", "J2.3"],
    "INA_ALERT": ["U8.3", "J2.9", "R29.1"],
    "TOUCH": ["U9.1", "U2.9"],
    "TOUCH_SNS": ["U9.3", "R24.2", "C24.1"],
    "TOUCH_PAD": ["J9.1", "R24.1"],
    "FOOT_L": ["J10.1", "U2.7", "R25.1"],
    "FOOT_R": ["J11.1", "U2.8", "R26.1"],
    # audio
    "AMP_SD": ["J2.1", "U4.1", "R9.1"],
    "LINE_P": ["J2.32", "C14.1"],
    "LINE_M": ["J2.34", "C15.1"],
    "AMP_INP": ["C14.2", "U4.3"],
    "AMP_INM": ["C15.2", "U4.4"],
    "SPK_P": ["U4.5", "J7.1"],
    "SPK_N": ["U4.8", "J7.2"],
}

# deliberately unconnected (drawn with a no-connect flag)
NC = (["U3.1", "U3.2", "U4.2"]                       # RT float = 500 kHz, EN float = on (TPS62933 table 7-1)
      # rev B: P03 P04 P05 (pins 7 8 9) are FOOT_L, FOOT_R and TOUCH
      + _p("U2", 10, 11, 13, 14, 15, 16, 17, 18, 19, 20)
      # LSM6DSV16X INT2 unused; OCS_Aux and SDO_Aux may be left open when the
      # auxiliary SPI is unused (DS13510 Table 2 note 3)
      + _p("U7", 9, 10, 11)
      + ["U10.13"])                                     # MODE floating = PFM at light load


def all_pins():
    seen = {}
    for n, pins in NETS.items():
        for p in pins:
            if p in seen:
                raise SystemExit("%s is on %s and %s" % (p, seen[p], n))
            seen[p] = n
    return seen
