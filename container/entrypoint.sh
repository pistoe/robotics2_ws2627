#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/panda/mujoco_runtime/setup.bash
source /opt/panda/interfaces/setup.bash
if [[ -f /opt/panda/hardware/setup.bash ]]; then
    source /opt/panda/hardware/setup.bash
fi
source /opt/panda/ws/install/setup.bash
export PYTHONPATH="/opt/panda/venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/panda-logs}"
mkdir -p "$ROS_LOG_DIR"
if [[ -f /workspaces/student/install/setup.bash ]]; then
    source /workspaces/student/install/setup.bash
fi
exec "$@"
