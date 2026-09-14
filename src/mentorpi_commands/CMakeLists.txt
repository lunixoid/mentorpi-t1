cmake_minimum_required(VERSION 3.8)
project(mentorpi_commands)

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

add_executable(command_dispatcher src/command_dispatcher.cpp)
ament_target_dependencies(command_dispatcher rclcpp)
# Humble cpp typesupport of mentorpi_msgs:
if(TARGET mentorpi_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target mentorpi_msgs "rosidl_typesupport_cpp")
else()
  set(cpp_typesupport_target ${mentorpi_msgs_TARGETS__rosidl_typesupport_cpp})
endif()
target_link_libraries(command_dispatcher "${cpp_typesupport_target}")
target_include_directories(command_dispatcher PRIVATE include)
install(TARGETS command_dispatcher DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_command_table test/test_command_table.cpp)
  target_include_directories(test_command_table PRIVATE include)
  add_test(NAME test_command_table COMMAND test_command_table)
endif()

ament_package()
