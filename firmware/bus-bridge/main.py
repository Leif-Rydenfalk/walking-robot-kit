# The Linux half of the Arduino app. The work happens in the sketch on the MCU;
# this process only has to stay alive so the app stays started.
import time

print("servo bus bridge flashed")
while True:
    time.sleep(3600)
