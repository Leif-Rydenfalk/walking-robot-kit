# Wiring

## The servo chain

One 3-pin plug on the hat: signal, supply, ground. The servos daisy-chain from
there, any order. Signal is half duplex on Serial1, which is D0 and D1 on the
UNO Q, at 1 Mbit/s. The hat switches the direction in hardware, so nothing in
the firmware has to drive a transmit-enable pin.

Without the hat, a URT-2 adapter on D0 and D1 does the same job, and everything
in `software/` works unchanged. You lose the camera, the audio, the distance
sensor, the pack monitor and the foot contacts.

Servo power does not come from the board. Give the chain its own supply between
5.0 and 8.4 V and share the ground with the board.

The hat takes the 2S pack on J6, a 6-pin JST PH: pins 1-3 are pack +, 4-6 are
ground. The kit's lead goes from J6 to an XT30. Do not feed servo current
through J5; its pin 2 is a pack-voltage reference with a 2 A contact.

## The inertial sensor

The hat carries an LSM6DSV16X at 0x6B and you do not have to wire anything.

If you are using a bare UNO Q with a loose MPU-6050 module, four wires:

| module | UNO Q | which pin |
|---|---|---|
| SCL | D21 | top row, first pin (PB10) |
| SDA | D20 | top row, second pin (PB11) |
| VCC | 3.3V | bottom row, fourth pin. Never 5V, never VIN |
| GND | GND | bottom row, sixth pin |

Those header pins are I2C2, which the Arduino core calls `Wire`. The Qwiic
socket is a different bus, I2C4, called `Wire1`. You do not have to know which
one you used: the firmware probes all of them and reports which answered.

If the sensor stops answering after a power change and every bus comes back
empty, it is the wiring, not the software. Check VCC is on 3.3V.

## The camera

An IMX219 module on the 22-pin 0.5 mm flex connector, contacts facing the
board. The CSI0 routing is Arduino's own, copied from the UNO Media Carrier, so
the board enumerates exactly as the carrier does.

## Speaker, distance sensor, feet

- Speaker: the 2-pin plug off the PAM8302A. 4 or 8 ohm, up to about 2.5 W.
- Distance: a VL53L5CX or VL53L8CX breakout on the 4-pin header.
- Feet: one 2-pin plug per foot, a switch or an FSR to ground. Pull-ups are on
  the board.

## Power-on order

1. UNO Q on USB-C first. Wait until the page answers on port 8938.
2. Servo supply second.
3. Check the page shows every servo you expect.

Pack first is not dangerous. It only means the servos sit powered with nothing
talking to them.
