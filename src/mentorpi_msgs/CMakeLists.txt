cmake_minimum_required(VERSION 3.8)
project(mentorpi_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlState.msg"
  "msg/FollowPersonStatus.msg"
  "msg/PersonHypothesis.msg"
  "msg/PersonArray.msg"
  "msg/NearestPerson.msg"
  "msg/MotionRestriction.msg"
  "msg/ChassisStatus.msg"
  "msg/ControlStatus.msg"
  "msg/NamedCommand.msg"
  "msg/ObstacleAvoidanceStatus.msg"
  "srv/SetControlMode.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)

ament_package()
