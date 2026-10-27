#!/bin/bash
# The PC's ROS 2 environment, with the repository mounted at /ws.
#
#   docker/station.sh                      a shell that talks to the REAL ROBOT (ROS domain 42)
#   docker/station.sh <command...>         run one command there, e.g. docker/station.sh ros2 topic list
#   FIRE_RESQ_SIM=1 docker/station.sh      a SIMULATION / test shell: domain 0, this machine only - it cannot
#                                          reach the robot, so a simulated /cmd_vel never turns a real wheel
#
# One container per mode, reused: later calls `docker exec` into it, so every shell shares one ROS graph.
# First use: docker build -f docker/station.Dockerfile -t fire_resq/station .   (from the repo root)
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
if [ "${FIRE_RESQ_SIM:-0}" = "1" ]; then
  NAME=fire_resq_sim;     ENVS=(-e ROS_DOMAIN_ID=0 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
else
  NAME=fire_resq_station; ENVS=(-e ROS_DOMAIN_ID=42)
fi
if [ -z "$(docker ps -q -f name="^${NAME}$")" ]; then
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  GUI=()
  if [ -n "${DISPLAY:-}" ]; then
    GUI=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
    if [ -n "${XAUTHORITY:-}" ]; then GUI+=(-e XAUTHORITY=/tmp/.Xauthority -v "$XAUTHORITY":/tmp/.Xauthority:ro); fi
  fi
  docker run -d --name "$NAME" --net=host --ipc=host --user "$(id -u):$(id -g)" -e HOME=/tmp/home \
      "${ENVS[@]}" "${GUI[@]}" -v "$REPO":/ws -w /ws fire_resq/station sleep infinity >/dev/null
fi
TTY=(-i); if [ -t 0 ]; then TTY=(-it); fi
if [ $# -eq 0 ]; then set -- bash; fi
exec docker exec "${TTY[@]}" -w /ws "$NAME" /entrypoint.sh "$@"
