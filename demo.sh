#!/bin/bash
# The FireResQ bottle demo, one script. Full guide: docs/DEMO.md
#
#   ./demo.sh             run the demo on the real robot, and open the dashboard
#   ./demo.sh dry         the same, but the motors are NEVER commanded (camera, detection and dashboard only)
#   ./demo.sh rehearse    no robot needed: a pretend robot in a pretend room, same dashboard
#   ./demo.sh motors      ONE-TIME wiring check (wheels OFF the ground): which motor is which wheel. Saves it, redeploys
#   ./demo.sh deploy      copy driver / config changes to the robot and restart its drivers (first time: ~10 min)
#   ./demo.sh stop        stop everything, on the PC and on the robot
#
# The robot's address is remembered in .demo_robot. To change it:   ROBOT=pi@192.168.1.23 ./demo.sh
set -euo pipefail
cd "$(dirname "$0")"
MODE="${1:-run}"

if [ -n "${ROBOT:-}" ]; then echo "$ROBOT" > .demo_robot; fi
ROBOT="$(cat .demo_robot 2>/dev/null || echo pi@pi.local)"   # pi.local follows the Pi when its address changes
mkdir -p "$HOME/.ssh"
# One password prompt, then the connection is reused for 10 minutes.
export FIRE_RESQ_SSH_OPTS="-o ControlMaster=auto -o ControlPath=$HOME/.ssh/fireresq-%C -o ControlPersist=600 -o ConnectTimeout=8"
on_robot() { ssh $FIRE_RESQ_SSH_OPTS "$@"; }
say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
die() { printf '\n\033[1;31m%s\033[0m\n' "$*" >&2; exit 1; }

docker image inspect fire_resq/station >/dev/null 2>&1 \
  || die "The PC's ROS image is missing. Build it once:  docker build -f docker/station.Dockerfile -t fire_resq/station ."

stop_pc() {       # $1 = container; ends a demo that is still running in it (and frees the dashboard's port)
  docker exec "$1" pkill -INT -f 'ros2 launch fire_resq_demo' >/dev/null 2>&1 || true
  sleep 1
  docker exec "$1" pkill -f 'fire_resq_demo' >/dev/null 2>&1 || true
}

open_page() {     # $1 = port; opens the dashboard as soon as it answers
  ( for _ in $(seq 60); do
      if curl -s -m 1 -o /dev/null "http://localhost:$1/state"; then xdg-open "http://localhost:$1/" >/dev/null 2>&1; exit; fi
      sleep 1
    done ) &
}

start_robot() {
  say "Robot: $ROBOT  (starting its camera and motor drivers; you may be asked for the Pi's password)"
  on_robot "$ROBOT" 'docker start fire_resq_robot >/dev/null' \
    || die "Could not start the drivers on $ROBOT.
  - Is the Pi on, and on the same Wi-Fi as this PC?  Try:  ping ${ROBOT#*@}
  - Has its address changed?  ROBOT=pi@<new address> ./demo.sh
  - First time on this Pi?  ./demo.sh deploy"
}

launch() {        # $1 = FIRE_RESQ_SIM value, then the launch arguments
  local sim="$1"; shift
  FIRE_RESQ_SIM="$sim" docker/station.sh bash -c 'colcon build --symlink-install --packages-select fire_resq_demo >/dev/null \
      && source install/setup.bash && exec ros2 launch fire_resq_demo demo.launch.py "$@"' demo "$@"
}

case "$MODE" in
  run|dry)
    start_robot
    docker/station.sh true; stop_pc fire_resq_station
    DRIVE=true; if [ "$MODE" = dry ]; then DRIVE=false; fi
    say "Dashboard: http://localhost:8080   (Ctrl+C here stops the demo and the robot)"
    open_page 8080
    launch 0 drive:=$DRIVE settings_file:=/ws/demo_settings.json || true
    stop_pc fire_resq_station
    ;;
  rehearse)
    FIRE_RESQ_SIM=1 docker/station.sh true; stop_pc fire_resq_sim
    say "Rehearsal (no robot). Dashboard: http://localhost:8081   (Ctrl+C here stops it)"
    open_page 8081
    launch 1 rehearsal:=true port:=8081 || true
    stop_pc fire_resq_sim
    ;;
  motors)
    say "WIRING CHECK. Lift the robot so all four wheels are OFF the ground, batteries in. Each motor spins in turn."
    read -r -p "Wheels off the ground? Press Enter to begin (Ctrl+C to cancel) "
    LOG="$(mktemp)"
    on_robot -t "$ROBOT" 'docker stop fire_resq_robot >/dev/null 2>&1; docker run --rm -it --privileged -v /dev:/dev \
        --ulimit nofile=1024:4096 fire_resq/robot ros2 run fire_resq_hardware motor_check' | tee "$LOG" || true
    LEFT="$(tr -d '\r' < "$LOG" | sed -n 's/^ *left_motors: *\(\[[0-9, -]*\]\).*/\1/p' | tail -1)"
    RIGHT="$(tr -d '\r' < "$LOG" | sed -n 's/^ *right_motors: *\(\[[0-9, -]*\]\).*/\1/p' | tail -1)"
    SIGNS="$(tr -d '\r' < "$LOG" | sed -n 's/^ *motor_signs: *\(\[[0-9, -]*\]\).*/\1/p' | tail -1)"
    rm -f "$LOG"
    [ -n "$LEFT" ] && [ -n "$RIGHT" ] && [ -n "$SIGNS" ] \
      || die "The check did not finish, so nothing was changed. Run ./demo.sh motors again (./demo.sh deploy restarts the drivers)."
    CFG=src/fire_resq_hardware/config/robot.yaml
    sed -i -E "s/^( *left_motors:) *\[[^]]*\]/\1 $LEFT/; s/^( *right_motors:) *\[[^]]*\]/\1 $RIGHT/; s/^( *motor_signs:) *\[[^]]*\]/\1 $SIGNS/" "$CFG"
    say "Saved to $CFG:  left $LEFT  right $RIGHT  signs $SIGNS.  Sending it to the robot ..."
    docker/deploy_robot.sh "$ROBOT"
    say "Done. Check it: ./demo.sh, then hold the arrow keys on the dashboard (wheels still off the ground)."
    ;;
  deploy)
    say "Copying the drivers to $ROBOT and restarting them ..."
    docker/deploy_robot.sh "$ROBOT"
    ;;
  stop)
    stop_pc fire_resq_station; stop_pc fire_resq_sim
    on_robot "$ROBOT" 'docker stop fire_resq_robot >/dev/null' && say "Robot drivers stopped." || true
    ;;
  *)
    sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
    ;;
esac
