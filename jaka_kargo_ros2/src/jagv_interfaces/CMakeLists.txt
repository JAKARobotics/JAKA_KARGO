cmake_minimum_required(VERSION 3.8)
project(jagv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DisplaySensorData.msg"
  "msg/ObstacleState.msg"
  "msg/ObstacleStateArray.msg"
  "msg/LocState.msg"
  "msg/MotionState.msg"
  "msg/AgvState.msg"
  "msg/HardwareState.msg"
  "msg/BatteryState.msg"
  "msg/SlamDiagInfo.msg"
  "srv/WheelOdomReset.srv"
  "srv/BaseRelocal.srv"
  "srv/MapControl.srv"
  "srv/MapDisplayControl.srv"
  "srv/LocControl.srv"
  "srv/GetLocState.srv"
  "srv/LocDisplayControl.srv"
  "srv/AutoMove.srv"
  "srv/MotionStateControl.srv"
  "srv/OnlineUpdate.srv"
  "srv/HardwareSelfCheck.srv"
  "srv/SaveChargerPos.srv"
  "srv/CalChargerPos.srv"
  DEPENDENCIES geometry_msgs
)

install(DIRECTORY
  msg srv
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()