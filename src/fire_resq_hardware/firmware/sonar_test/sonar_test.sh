#!/bin/bash
# Run ON THE PI (not the PC). One command for the sonar bring-up test:
#   1. stops the robot's driver container if it is running (only one program may use the Uno's serial port),
#   2. flashes the sonar test sketch (~/sonar_test.ino.hex) onto the Uno,
#   3. shows what the Uno prints, live,
#   4. when you press Ctrl+C - or the script ends any other way - reflashes the real robot firmware
#      (~/base_controller.ino.hex) so a test sketch never keeps running after you are done looking at it.
# It does NOT restart the robot's driver container: run ./demo.sh on the PC when you want to drive again.
#
#   ssh pi@pi.local
#   ./sonar_test.sh                  # or: ./sonar_test.sh /dev/ttyUSB1  if the port ever renumbers
set -uo pipefail
PORT="${1:-/dev/ttyUSB0}"
TEST_HEX="$HOME/sonar_test.ino.hex"
ORIGINAL_HEX="$HOME/base_controller.ino.hex"
restored=0

flash() { avrdude -p atmega328p -c arduino -P "$PORT" -b 115200 -U flash:w:"$1":i 2>&1 | tail -4; return "${PIPESTATUS[0]}"; }

restore() {
  [ "$restored" = 1 ] && return
  restored=1
  echo
  if [ -f "$ORIGINAL_HEX" ]; then
    echo "Restoring the real robot firmware..."
    if flash "$ORIGINAL_HEX"; then echo "Done - the robot's normal firmware is back on the Uno."
    else echo "RESTORE FAILED - ask Claude to reflash the real firmware."; fi
  else
    echo "Could not find $ORIGINAL_HEX - nothing restored. Ask Claude to reflash the real firmware."
  fi
}
trap restore EXIT INT TERM

[ -f "$TEST_HEX" ] || { echo "Missing $TEST_HEX"; exit 1; }
if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx fire_resq_robot; then
  echo "Stopping the robot's driver container (it holds the serial port)..."
  docker stop fire_resq_robot >/dev/null 2>&1
fi
echo "Flashing the sonar test sketch..."
flash "$TEST_HEX" || { echo "Flashing the test sketch failed."; exit 1; }
stty -F "$PORT" 115200 raw -echo
echo "Watching $PORT at 115200 baud. Ctrl+C to stop AND restore the real firmware."
cat "$PORT"
