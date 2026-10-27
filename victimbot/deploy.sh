#!/bin/bash
# Copy the victim-rescue bot to the Pi (~/victimbot). Run from this folder on the PC.
#   ./deploy.sh [pi@pi.local]
# The program flashes victimbot_fw.hex onto the Uno itself the first time it finds other firmware there.
set -e
cd "$(dirname "$0")"
HOST=${1:-pi@pi.local}
tar cf - victimbot.py rescue3.py teleop_demo.py dashboard.html victimbot_fw.hex README.md firmware/pinfinder/pinfinder.ino \
  | ssh "$HOST" 'mkdir -p ~/victimbot && tar xf - -C ~/victimbot && cd ~/victimbot && python3 victimbot.py --selftest'
echo "Deployed. On the Pi:  cd ~/victimbot && python3 -u rescue3.py      dashboard: http://pi.local:8090"
