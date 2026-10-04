# Firmware

One Arduino app, `bus-bridge`. It runs on the UNO Q's MCU and does one job:
own the servo chain on Serial1 and answer the Linux side over RouterBridge.

## What it does

- Drives the half-duplex TTL bus on D0 and D1 at 1 Mbit/s, with automatic
  direction. The hat handles the direction switch in hardware, so the sketch
  never has to toggle a transmit-enable pin.
- `bus_batch` is the fast path: one call carries a sync write of every goal
  position plus one telemetry read per servo, so a whole roster costs one
  round trip instead of fifteen.
- Sweeps the bus itself on a poll loop, so the Linux side reads a fresh
  snapshot rather than waiting for each servo in turn.
- Reads an MPU-6050 or compatible sensor over I2C, on whichever bus answers.
  It probes `Wire` (I2C2, the header pins) and `Wire1` (I2C4, the Qwiic
  socket) and reports which one replied. It does not assume.
- Drives the UNO Q's own 13 by 8 LED matrix, so the robot can show its state
  on its face.

## Flashing it

`../software/install.sh` copies this app to the board and starts it; the
sketch compiles and flashes there, which takes about a minute the first time.

By hand:

    adb push app.yaml    /home/arduino/ArduinoApps/servo-bus-mcu/app.yaml
    adb push sketch.ino  /home/arduino/ArduinoApps/servo-bus-mcu/sketch/sketch.ino
    adb push sketch.yaml /home/arduino/ArduinoApps/servo-bus-mcu/sketch/sketch.yaml
    adb push main.py     /home/arduino/ArduinoApps/servo-bus-mcu/python/main.py
    adb shell arduino-app-cli app start user:servo-bus-mcu

`main.py` is the Linux half of the app and does nothing but stay alive so the
app stays started. All the work is in `sketch.ino`.

## If the bus goes quiet

Hot-plugging a servo while the bus is running can wedge the MCU. The daemon on
the Linux side notices within a second and restarts this app, which takes about
twenty seconds, then carries on. You do not have to do anything.
