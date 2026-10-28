#!/usr/bin/env bash
# Read-only diagnostics; never starts a control publisher.
set -e
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"
source /opt/ros/jazzy/setup.bash
source campusgame_ros/ros2_auto_framework/install/local_setup.bash
source install/local_setup.bash
exec python3 src/stage1_comm_test/tools/capture_competition_status.py "$@"
