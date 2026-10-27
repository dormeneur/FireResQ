#!/bin/bash
# Run ON THE PI (not the PC): shows whatever test sketch is on the Uno's serial output live, as if you had a
# serial monitor open. Opening the port resets the Uno, so you see it from the very first line.
#
#   ssh pi@pi.local
#   ./watch_serial.sh
#
# Ctrl+C stops watching AND reflashes the real robot firmware (~/base_controller.ino.hex) back onto the Uno, so
# a test sketch (the servo sweep, etc.) never keeps running after you are done looking at it. That restore also
# runs if the script exits any other way. It does NOT restart the robot's Docker container - ask for that
# separately (./demo.sh) once you are ready to drive again.
set -uo pipefail
PORT="${1:-/dev/ttyUSB0}"
ORIGINAL_HEX="$HOME/base_controller.ino.hex"
restored=0

restore() {
  [ "$restored" = 1 ] && return
  restored=1
  echo
  if [ -f "$ORIGINAL_HEX" ]; then
    echo "Restoring the real robot firmware..."
    avrdude -p atmega328p -c arduino -P "$PORT" -b 115200 -U flash:w:"$ORIGINAL_HEX":i
    echo "Done - the robot's normal firmware is back on the Uno."
  else
    echo "Could not find $ORIGINAL_HEX - nothing restored. Ask Claude to reflash the real firmware."
  fi
}
trap restore EXIT INT TERM

stty -F "$PORT" 115200 raw -echo
echo "Watching $PORT at 115200 baud. Ctrl+C to stop AND restore the real firmware."
cat "$PORT"
