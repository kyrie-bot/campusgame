# 正式比赛并行编译包：最多2任务，不编译测试

解压到Desktop下的英文新目录。在能看到build_autoaim.sh的文件夹里打开终端，执行：

```bash
bash build_autoaim.sh
```

build_autoaim_safe.sh是相同入口。依赖包按顺序构建，每个包内部最多2个编译任务并行；可用内存低于10 GiB时自动降为1任务，不足2 GiB时拒绝启动。内存预算仍为可用内存的60%，再按任务数均分给每个进程，每进程最多4 GiB地址空间。保留-O2优化和-g0，不生成调试信息。输出实时编译进度。

需要主动退回1任务时：

```bash
CAMPUS_BUILD_JOBS=1 bash build_autoaim.sh
```

通信工作区单独运行build.sh也复用相同资源策略，仅构建通信桥与游戏消息，不会顺带构建主程序。

只允许四个必需包：ros_tcp_endpoint（Unity桥）、tdt_interface（游戏消息）、campus_aim_interfaces（算法消息）、stage1_comm_test（正式比赛主程序）。stage1_comm_test是沿用旧包名，内含8个比赛节点，不是测试包。三个CMake包强制BUILD_TESTING=OFF；没有test/tests目录、测试依赖或colcon test/ctest执行。ROS生成消息、类型支持等是通信所需代码。

编译成功后，终端一执行bash start_bridge.sh；主机若已有赛方通信桥，不要重复占用同一端口。另开终端执行：

```bash
bash run_competition_auto.sh player_id:=1 buff_display:=true mask_display:=true
```

红方player_id:=2，关闭显示用false。之后正常比赛不用重复编译。

环境仍需Ubuntu/ROS2 Jazzy、与cv_bridge匹配的OpenCV等依赖。本机验证使用OpenCV4.6.0；不内置系统依赖。源码包不含.build/.build_safe/install缓存，编译缓存可在同一目录保留用于后续增量构建。算法、瞄准/开火参数、地图、模型和消息均沿用上一版。

并行会提高内存/CPU负载，资源预检和进程上限不能解决过热、供电或驱动故障，也不能保证主机不会重启。具体故障仍需主机系统日志判断。
