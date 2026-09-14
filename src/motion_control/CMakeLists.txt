cmake_minimum_required(VERSION 3.8)
project(motion_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mentorpi_msgs REQUIRED)

add_executable(motion_control src/motion_control.cpp)
ament_target_dependencies(motion_control rclcpp geometry_msgs)
# Humble cpp typesupport of mentorpi_msgs:
if(TARGET mentorpi_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target mentorpi_msgs "rosidl_typesupport_cpp")
else()
  set(cpp_typesupport_target ${mentorpi_msgs_TARGETS__rosidl_typesupport_cpp})
endif()
target_link_libraries(motion_control "${cpp_typesupport_target}")
target_include_directories(motion_control PRIVATE include)
install(TARGETS motion_control DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_follow_control test/test_follow_control.cpp)
  target_include_directories(test_follow_control PRIVATE include)
  add_test(NAME test_follow_control COMMAND test_follow_control)
endif()

ament_package()
