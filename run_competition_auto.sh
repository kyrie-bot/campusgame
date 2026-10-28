#!/usr/bin/env bash
set -e
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"
source /opt/ros/jazzy/setup.bash
source campusgame_ros/ros2_auto_framework/install/local_setup.bash
source install/local_setup.bash
for argument in "$@"; do
    if [[ "$argument" == "--show-args" || "$argument" == "-s" ]]; then
        exec ros2 launch stage1_comm_test competition_auto.launch.py "$@"
    fi
done
python3 src/stage1_comm_test/tools/competition_preflight.py "$@"
exec ros2 launch stage1_comm_test competition_auto.launch.py "$@"
