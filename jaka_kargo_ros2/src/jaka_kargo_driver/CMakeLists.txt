cmake_minimum_required(VERSION 3.8)
project(jaka_kargo_driver)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jaka_kargo_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)


# Declare ROS 2 message and service dependencies
find_package(rosidl_default_generators REQUIRED)

# Declare executable targets
add_executable(jaka_kargo_driver src/jaka_kargo_driver.cpp)

# Link libraries
ament_target_dependencies(jaka_kargo_driver
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  jaka_kargo_msgs
  action_msgs
  control_msgs
)

# For including directories in ROS 2
target_include_directories(jaka_kargo_driver
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

# Specify where to find external libraries
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Add the directory to the search path
link_directories(${LIBRARY_DIR})

# Find the library and link it properly
target_link_libraries(jaka_kargo_driver ${LIBRARY_DIR}/libjakaAPI_2_3_0_DUAL_9.so)

# Install executables
install(TARGETS
  jaka_kargo_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()