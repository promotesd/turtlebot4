cmake_minimum_required(VERSION 3.8)
project(turtlebot4_rl_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EpisodeState.msg"
  "srv/ResetEpisode.srv"
  "srv/SetGoal.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
