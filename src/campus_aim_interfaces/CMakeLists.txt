cmake_minimum_required(VERSION 3.16)
project(campus_aim_interfaces)
# Disable project test programs for competition deployment.
set(BUILD_TESTING OFF CACHE BOOL "Competition-only build" FORCE)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tdt_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg" "msg/ArmorFrame.msg" "msg/Target.msg" "msg/TrackFrame.msg" "msg/AimSolution.msg" "msg/BuffSolution.msg" "msg/BuffStatus.msg"
  "msg/NavigationPlan.msg"
  DEPENDENCIES tdt_interface geometry_msgs std_msgs nav_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
