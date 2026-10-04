#!/bin/sh
# Install the servo bus daemon and its MCU app on an Arduino Uno Q.
#
#   BOARD=arduino@<board address> ./install.sh     over the network (ssh, the board's own login)
#   ./install.sh --adb                          over the USB cable (set ANDROID_SERIAL if you have several)
#
# Run it on your computer, after you have built the binary (see software/README.md, step 2).
# It copies the binary, the MCU app and the service, starts the MCU app, then the daemon.
# It never moves a servo. The daemon starts with torque untouched and only reads the bus
# until you ask for something on the page.
set -eu
cd "$(dirname "$0")"

BIN=servo-bus/target/aarch64-unknown-linux-gnu/release/servo-bus
[ -x "$BIN" ] || { echo "no $BIN: build it first, see README step 2"; exit 1; }

if [ "${1:-}" = "--adb" ]; then
  run() { adb shell "$1"; }
  put() { adb push "$1" "$2" >/dev/null; }
else
  [ -n "${BOARD:-}" ] || { echo "set BOARD=user@host, or pass --adb"; exit 1; }
  run() { ssh "$BOARD" "$1"; }
  put() { scp -q "$1" "$BOARD:$2"; }
fi

H=/home/arduino
APP=$H/ArduinoApps/servo-bus-mcu

echo "1/5 copy the MCU app to $APP"
run "mkdir -p $APP/sketch $APP/python $H/.config/systemd/user"
put ../firmware/bus-bridge/app.yaml      $APP/app.yaml
put ../firmware/bus-bridge/sketch.ino    $APP/sketch/sketch.ino
put ../firmware/bus-bridge/sketch.yaml   $APP/sketch/sketch.yaml
put ../firmware/bus-bridge/main.py       $APP/python/main.py

echo "2/5 copy the daemon and its service"
run "systemctl --user stop servo-bus 2>/dev/null || true"
put "$BIN" $H/servo-bus
run "chmod +x $H/servo-bus"
put servo-bus/servo-bus.service $H/.config/systemd/user/servo-bus.service
# your ID registry stays yours: this seed is copied only when the board has none
run "test -f $H/servo-bus-ids.json" || put servo-bus/servo-ids.json $H/servo-bus-ids.json

echo "3/5 start the MCU app (it compiles and flashes the sketch, about a minute the first time)"
run "arduino-app-cli app start user:servo-bus-mcu"

echo "4/5 start the daemon"
run "systemctl --user daemon-reload && systemctl --user enable --now servo-bus"
sleep 3

echo "5/5 check"
run "curl -s --max-time 5 http://127.0.0.1:8938/api/bus | head -c 300"; echo
echo "open http://<board address>:8938/ on any phone or computer on the same network"
[ "${1:-}" = "--adb" ] && { adb forward tcp:8938 tcp:8938 >/dev/null; echo "or http://127.0.0.1:8938/ over the cable"; }
