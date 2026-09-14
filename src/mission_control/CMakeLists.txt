cmake_minimum_required(VERSION 3.8)
project(mission_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mentorpi_msgs REQUIRED)

add_executable(mission_control src/mission_control.cpp)
ament_target_dependencies(mission_control rclcpp)
# Humble cpp typesupport of mentorpi_msgs:
if(TARGET mentorpi_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target mentorpi_msgs "rosidl_typesupport_cpp")
else()
  set(cpp_typesupport_target ${mentorpi_msgs_TARGETS__rosidl_typesupport_cpp})
endif()
target_link_libraries(mission_control "${cpp_typesupport_target}")
target_include_directories(mission_control PRIVATE include)
install(TARGETS mission_control DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_follow_behavior test/test_follow_behavior.cpp)
  target_include_directories(test_follow_behavior PRIVATE include)
  add_test(NAME test_follow_behavior COMMAND test_follow_behavior)
endif()

ament_package()
