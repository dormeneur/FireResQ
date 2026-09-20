cmake_minimum_required(VERSION 3.8)
project(fire_resq_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/VictimState.msg"
  "msg/WorldState.msg"
  "msg/ScoreBreakdown.msg"
  "msg/RescueTarget.msg"
  "msg/MagnetState.msg"
  "srv/SelectTarget.srv"
  "srv/UpdateVictimStatus.srv"
  "srv/SetMagnet.srv"
  "action/ExecuteRescue.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces vision_msgs
)

ament_package()
