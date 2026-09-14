cmake_minimum_required(VERSION 3.8)
project(mentorpi_platform)

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

add_executable(platform_adapter src/platform_adapter.cpp)
ament_target_dependencies(platform_adapter rclcpp geometry_msgs)
target_include_directories(platform_adapter PRIVATE include)
# Humble cpp typesupport of mentorpi_msgs (not deprecated rosidl_target_interfaces).
# rosidl_get_typesupport_target() only works in the generating package; a
# downstream node links the exported C++ typesupport target instead.
if(TARGET mentorpi_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target mentorpi_msgs "rosidl_typesupport_cpp")
  target_link_libraries(platform_adapter "${cpp_typesupport_target}")
else()
  target_link_libraries(platform_adapter ${mentorpi_msgs_TARGETS__rosidl_typesupport_cpp})
endif()

install(TARGETS platform_adapter
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_adapter_gate test/test_adapter_gate.cpp)
  target_include_directories(test_adapter_gate PRIVATE include)
  add_test(NAME test_adapter_gate COMMAND test_adapter_gate)
endif()

ament_package()
