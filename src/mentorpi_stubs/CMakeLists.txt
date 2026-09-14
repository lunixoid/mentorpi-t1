cmake_minimum_required(VERSION 3.8)
project(mentorpi_stubs)

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

add_executable(stub_graph src/stub_graph.cpp)
ament_target_dependencies(stub_graph rclcpp)
# Humble cpp typesupport of mentorpi_msgs (not deprecated rosidl_target_interfaces).
# rosidl_get_typesupport_target() only works in the generating package; a
# downstream node links the exported C++ typesupport target instead.
if(TARGET mentorpi_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target mentorpi_msgs "rosidl_typesupport_cpp")
  target_link_libraries(stub_graph "${cpp_typesupport_target}")
else()
  target_link_libraries(stub_graph ${mentorpi_msgs_TARGETS__rosidl_typesupport_cpp})
endif()

install(TARGETS stub_graph
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
