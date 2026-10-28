#!/usr/bin/env bash
# Bounded-parallel competition-only source build. No game/control nodes are started.
set -Ee -o pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! /usr/bin/python3 -c 'import sys; sys.exit(0 if sys.argv[1].isascii() else 1)' "$script_dir"; then
    printf '请将源码重新解压到 Desktop 下的英文新目录后编译。当前路径：%s\n' "$script_dir" >&2
    exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    printf '缺少 ROS 2 Jazzy：/opt/ros/jazzy/setup.bash\n' >&2
    exit 2
fi
# Keep packages ordered; allow at most two compiler jobs inside each package.
export CAMPUS_BUILD_JOBS="${CAMPUS_BUILD_JOBS:-2}"
if [[ "$CAMPUS_BUILD_JOBS" != 1 && "$CAMPUS_BUILD_JOBS" != 2 ]]; then
    printf 'CAMPUS_BUILD_JOBS只允许1或2。\n' >&2
    exit 2
fi
build_mode=all
case "${1:-}" in
    --check-only) build_mode=check; shift ;;
    --bridge-only) build_mode=bridge; shift ;;
esac
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
mkdir -p "$script_dir/.logs"
log="$script_dir/.logs/safe_build_$(date +%Y%m%d_%H%M%S)_$$.log"
exec > >(tee "$log") 2>&1
trap 'rc=$?; printf "\n编译停止，退出码 %s。日志：%s\n不要连续反复编译；请保留最后的错误信息。\n" "$rc" "$log" >&2; exit "$rc"' ERR
memory_plan() {
    /usr/bin/python3 - <<'MEMORY'
from pathlib import Path
import os,sys
mem={line.split(':',1)[0]:int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines() if line.startswith(('MemAvailable:','MemFree:'))}
available=mem.get('MemAvailable',mem.get('MemFree',0))
# Respect visible cgroup-v2 memory limits when present.
for base in (Path('/sys/fs/cgroup'),Path('/sys/fs/cgroup')/Path('/proc/self/cgroup').read_text().strip().split('::')[-1].lstrip('/')):
    try:
        limit=(base/'memory.max').read_text().strip()
        if limit!='max':available=min(available,max(0,int(limit)-int((base/'memory.current').read_text()))//1024)
    except (OSError,ValueError):pass
if available<2*1024*1024:
    print('可用内存不足2 GiB；本次不启动编译。请先关闭游戏和非必要应用。',file=sys.stderr);sys.exit(2)
jobs=int(os.environ.get('CAMPUS_BUILD_JOBS','2'))
# Prior clean build reached about 2.2 GiB RSS in one compiler. Preserve room
# for compiler virtual mappings and the desktop; fall back instead of doubling
# the old per-process budget on machines with insufficient available RAM.
if jobs==2 and available<10*1024*1024:jobs=1
cap=min(4*1024*1024,available*60//(100*jobs))
print(jobs,cap)
MEMORY
}
build_stage() {
    local plan jobs cap
    plan="$(memory_plan)"
    read -r jobs cap <<< "$plan"
    printf '\n包按依赖顺序，包内%s任务并行；该阶段每个进程虚拟内存上限：%s MiB\n' "$jobs" "$((cap / 1024))"
    if [[ "$jobs" != "$CAMPUS_BUILD_JOBS" ]]; then
        printf '可用内存不足10 GiB，已自动降为1个任务。\n'
    fi
    # Exceeding this process limit fails allocation instead of allowing one
    # compiler to consume all available memory. This is not a thermal/power fix.
    (
        ulimit -v "$cap"
        export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
        export MAKEFLAGS="-j$jobs"
        exec nice -n 10 colcon "$@"
    )
}
initial_plan="$(memory_plan)"
read -r initial_jobs initial_cap <<< "$initial_plan"
printf '正式比赛编译：包按依赖顺序，包内最多%s任务并行，-O2且不生成调试信息。\n日志：%s\n' "$CAMPUS_BUILD_JOBS" "$log"
if [[ "$build_mode" == check ]]; then
    printf '资源预检通过；当前实际任务数%s，每进程上限%s MiB。未执行编译。\n' "$initial_jobs" "$((initial_cap/1024))"
    exit 0
fi
source /opt/ros/jazzy/setup.bash
bridge_root="$script_dir/campusgame_ros/ros2_auto_framework"
cd "$bridge_root"
bridge_cmake_args=()
if [[ "$build_mode" == bridge ]]; then bridge_cmake_args=("$@"); fi
build_stage --log-base .logs build --executor sequential --event-handlers console_direct+ --build-base .build_safe --install-base install \
    --base-paths "$bridge_root/src" --packages-select ros_tcp_endpoint tdt_interface --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
    -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG -g0' '-DCMAKE_C_FLAGS_RELEASE=-O2 -DNDEBUG -g0' "${bridge_cmake_args[@]}" -DBUILD_TESTING=OFF
if [[ "$build_mode" == bridge ]]; then
    printf '\n通信桥与接口编译完成。\n'
    exit 0
fi
source "$bridge_root/install/local_setup.bash"
tdt_cmake_dir="$bridge_root/install/tdt_interface/share/tdt_interface/cmake"
if [[ ! -f "$tdt_cmake_dir/tdt_interfaceConfig.cmake" ]]; then
    printf '通信接口未完整构建：%s\n' "$tdt_cmake_dir" >&2
    exit 1
fi
cd "$script_dir"
build_stage --log-base .logs build --executor sequential --event-handlers console_direct+ --build-base .build_safe --install-base install \
    --base-paths "$script_dir/src" --packages-select campus_aim_interfaces stage1_comm_test --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
    -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG -g0' '-DCMAKE_C_FLAGS_RELEASE=-O2 -DNDEBUG -g0' \
    -Dtdt_interface_DIR="$tdt_cmake_dir" "$@" -DBUILD_TESTING=OFF
for node in armor_detector tracker aim_solver combat_gimbal buff_controller static_grid_publisher dijkstra_planner path_follower; do
    strip --strip-debug "$script_dir/install/stage1_comm_test/lib/stage1_comm_test/$node"
done
printf '\n编译完成。运行入口仍为 bash run_competition_auto.sh。\n日志：%s\n' "$log"
