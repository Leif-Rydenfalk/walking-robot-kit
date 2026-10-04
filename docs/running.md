# First bring-up

In order. Nothing here needs a terminal except step 4, and that is one line.

## 1. Read the pack

Read the label before you plug it in. 5.0 to 8.4 V is safe. Two packs in series
read about 13 V and will burn every servo on the chain in seconds.

With the board already up, check what the chain is actually seeing:

    python3 software/tools/bus_volts.py

It prints every servo's voltage and one verdict line. It moves nothing.

## 2. Power on

1. UNO Q on USB-C. Wait until `http://<board address>:8938/` answers.
2. Servo supply.
3. Check the page lists every servo. A missing row is a servo with the wrong
   ID or a bad plug, not a servo that is broken.

## 3. Set zero

Hold the robot in the zero pose: legs straight down, feet flat, head level.
Then hold still for two seconds.

    python3 software/tools/zero_now.py

That is the whole calibration. Nothing is written into a servo, nothing moves,
and torque is off the entire time. It reads where every joint is and treats
that pose as zero.

It refuses a pose that is still moving, because a zero read while you are
adjusting a leg is not a zero. While it waits it prints which joint is moving
and by how much, so you know when you are still enough.

## 4. Check the inertial sensor

Tilt the robot and watch the numbers follow. This moves nothing.

    curl http://<board address>:8938/api/imu/scan     # which I2C bus and address answered
    curl http://<board address>:8938/api/imu          # one sample: accelerometer and gyro

Tilt it and read `/api/imu` again. If the numbers do not change, the sensor is
not working, whatever the scan said. A still robot and a frozen chip look
identical in a log, so move it before you believe it.

Do not poll the scan in a loop. With nothing answering it probes several
hundred addresses with a timeout each, on a microcontroller that serves one
request at a time, and it will starve the servo bus while it does.

## 5. Move one joint

Open `http://<board address>:8938/?control`.

Pick one servo. Press **Enable slider**: it holds that servo where it is, at
30 % torque, and reads back that it held. Move the slider a little. Press
**Release** to turn torque off again.

If the slider is disabled, that servo has no recorded angle band yet. Record
one first, see [servos.md](servos.md).

## When the bus stops answering

Hot-plugging a servo while the bus is running can wedge the MCU. The daemon
notices within a second, restarts the MCU app once, which takes about twenty
seconds, and carries on; the page says what happened. If that does not help it
waits rather than restarting again, and tries once more the moment the bus
answers.
