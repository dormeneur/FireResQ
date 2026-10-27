#!/bin/bash
# Copy the robot-side packages to the Pi, build the robot image there, and (re)start the drivers.
#
#   docker/deploy_robot.sh pi@10.250.139.26            build + start
#   docker/deploy_robot.sh pi@10.250.139.26 --no-start build only
#
# Needs on the Pi: docker (pi in the docker group). Extra ssh options: export FIRE_RESQ_SSH_OPTS="...".
set -euo pipefail
ROBOT="${1:?usage: deploy_robot.sh user@host [--no-start]}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
SSH=(ssh ${FIRE_RESQ_SSH_OPTS:-} "$ROBOT")

tar -C "$REPO" --exclude='__pycache__' --exclude='firmware/backup' -cf - \
    docker src/fire_resq_description src/fire_resq_hardware \
  | "${SSH[@]}" 'rm -rf ~/fireresq/ws && mkdir -p ~/fireresq/ws && tar -C ~/fireresq/ws -xf -'
"${SSH[@]}" 'cd ~/fireresq/ws && DOCKER_BUILDKIT=0 docker build -q -f docker/robot.Dockerfile -t fire_resq/robot . '
if [ "${2:-}" = "--no-start" ]; then exit 0; fi
# --privileged + /dev: the camera and the Arduino may re-enumerate (ttyUSB0 -> ttyUSB1) when re-plugged.
# --ulimit nofile: NOT optional on the Pi. Docker gives containers an open-file limit of ~2^30 there, and the tracing
#   library every ROS 2 process loads (liblttng-ust) sizes a table by it: 128 MB per process, measured
#   (a node: 198 MB without this flag, 71 MB with it; the Pi has 905 MB).
"${SSH[@]}" 'docker rm -f fire_resq_robot >/dev/null 2>&1; docker run -d --name fire_resq_robot --net=host \
    --privileged -v /dev:/dev --ulimit nofile=1024:4096 fire_resq/robot >/dev/null \
    && sleep 12 && docker logs fire_resq_robot 2>&1 | tail -15'
