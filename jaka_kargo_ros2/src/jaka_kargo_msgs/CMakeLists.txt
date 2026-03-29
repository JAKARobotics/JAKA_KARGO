cmake_minimum_required(VERSION 3.8)
project(jaka_kargo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStates.msg"
  "msg/DIInfo.msg"
  "msg/MultiMoveUnit.msg"
  
  "srv/Move.srv"
  "srv/ServoMoveEnable.srv"
  "srv/ServoMove.srv"
  "srv/GetDefaultBase.srv"
  "srv/SetToolOffset.srv"
  "srv/GetToolOffset.srv"
  "srv/SetToolPayload.srv"
  "srv/GetToolPayload.srv"
  "srv/DragMode.srv"
  "srv/SetCollisionLevel.srv"
  "srv/GetCollisionLevel.srv"
  "srv/GetFK.srv"
  "srv/GetIK.srv"
  "srv/ClearError.srv"
  "srv/GetSdkVersion.srv"
  "srv/SetDebugMode.srv"
  "srv/GetDHParams.srv"
  "srv/ExtEnable.srv"
  "srv/JogExt.srv"
  "srv/MultiMoveExt.srv"
  "srv/GetExtStatus.srv"
   DEPENDENCIES std_srvs
)

ament_package()
