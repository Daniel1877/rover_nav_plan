cmake_minimum_required(VERSION 3.5)
project(rover_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/JointCommand.msg"
  "msg/JointCommandArray.msg"
  "msg/TemperatureArray.msg"
  "msg/WrenchStampedArray.msg")

set(srv_files
  "srv/ChangeLocomotionMode.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs builtin_interfaces)

ament_package()
