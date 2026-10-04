# Servos

Fifteen Feetech STS-protocol bus servos, one chain, 1 Mbit/s. We use the
HD-1910-C001. Any STS servo that speaks the same protocol will answer.

## The IDs

The numbering follows Open Duck Mini: right leg in the tens, left leg in the
twenties, head in the thirties.

| ID | joint | | ID | joint |
|---|---|---|---|---|
| 10 | right hip yaw | | 20 | left hip yaw |
| 11 | right hip roll | | 21 | left hip roll |
| 12 | right hip pitch | | 22 | left hip pitch |
| 13 | right knee | | 23 | left knee |
| 14 | right ankle | | 24 | left ankle |
| 30 | neck pitch | | 33 | head roll |
| 31 | head pitch | | 34 | mouth |
| 32 | head yaw | | | |

The same table as data: `software/servo-bus/servo-ids.json`.

## Setting them

Every servo leaves the factory at ID 1, so plug them in one at a time.

1. Plug in one servo. It shows up on the page as ID 1 within two seconds.
2. Press **Set ID** on its row, type the new ID and the joint name, confirm.
3. Unplug it, plug in the next one.

The page refuses the change when the servo does not answer thirty reads out of
thirty cleanly, which is what two servos on one ID looks like; when the new ID
already answers on the bus; or when the new ID is already recorded for another
joint, plugged in or not. It then offers you a free temporary ID between 101
and 199.

After the write it checks thirty clean reads at the new ID, that the old ID is
gone, and that the ID register reads back. Then it sets maximum torque and the
torque limit to 30 % and turns torque off.

Every change is kept with the time, the old ID, the new ID and the joint, in
`/home/arduino/servo-bus-ids.json` on the board, shown at the bottom of the
page.

## Angle limits

A servo with no recorded band cannot be driven. This is deliberate: the mouth
rests inside about twenty degrees of travel and a hip swings two hundred, and
one global band that suits the hip will tear the jaw off.

To record one:

1. Press **Record** on the servo's card. Torque goes off.
2. Turn that joint by hand, slowly, from one end stop to the other.
3. Press **Save**.

The saved band is the ends you reached minus three degrees each side. It goes
into the servo's own EEPROM and is read back. A recording that jumps more than
half a turn is marked wrapped and the save is refused; so is a band under about
five degrees.

From the command line instead: `software/tools/record_limits.py`, then
`software/tools/limits_table.py` to see them all.

## Power

5.0 to 8.4 V. Read the label on your pack before you plug it in. Two packs in
series read about 13 V and will destroy every servo on the chain in seconds.
The daemon refuses to torque anything above 8.4 V, but a guard is not a reason
to plug in a pack nobody read.

Check the bus any time, without starting anything and without moving anything:

    python3 software/tools/bus_volts.py
