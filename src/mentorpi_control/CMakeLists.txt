cmake_minimum_required(VERSION 3.8)
project(mentorpi_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mentorpi_msgs REQUIRED)

# Humble cpp typesupport of mentorpi_msgs (not deprecated rosidl_target_interfaces).
# Resolved once so extra executables can link the same target.
if(TARGET mentorpi_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target mentorpi_msgs "rosidl_typesupport_cpp")
else()
  set(cpp_typesupport_target ${mentorpi_msgs_TARGETS__rosidl_typesupport_cpp})
endif()

add_executable(control_state src/control_state.cpp)
ament_target_dependencies(control_state rclcpp std_msgs)
target_link_libraries(control_state "${cpp_typesupport_target}")
target_include_directories(control_state PRIVATE include)

add_executable(pad_teleop src/pad_teleop.cpp)
ament_target_dependencies(pad_teleop rclcpp std_msgs sensor_msgs geometry_msgs)
target_link_libraries(pad_teleop "${cpp_typesupport_target}")
target_include_directories(pad_teleop PRIVATE include)

add_executable(linux_joy src/linux_joy.cpp)
ament_target_dependencies(linux_joy rclcpp sensor_msgs)
target_include_directories(linux_joy PRIVATE include)
find_package(Threads REQUIRED)
target_link_libraries(linux_joy Threads::Threads)

add_executable(control_mux src/control_mux.cpp)
ament_target_dependencies(control_mux rclcpp geometry_msgs)
target_link_libraries(control_mux "${cpp_typesupport_target}")
target_include_directories(control_mux PRIVATE include)

install(TARGETS control_state pad_teleop linux_joy control_mux
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_joy_control test/test_joy_control.cpp)
  target_include_directories(test_joy_control PRIVATE include)
  add_test(NAME test_joy_control COMMAND test_joy_control)

  add_executable(test_control_mode test/test_control_mode.cpp)
  target_include_directories(test_control_mode PRIVATE include)
  add_test(NAME test_control_mode COMMAND test_control_mode)

  add_executable(test_control_mux test/test_control_mux.cpp)
  target_include_directories(test_control_mux PRIVATE include)
  add_test(NAME test_control_mux COMMAND test_control_mux)
endif()

ament_package()
