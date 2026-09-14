cmake_minimum_required(VERSION 3.8)
project(mentorpi_perception)

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
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mentorpi_msgs REQUIRED)

add_executable(person_perception src/person_perception.cpp)
ament_target_dependencies(
  person_perception
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(person_perception PRIVATE include)
# Humble cpp typesupport of mentorpi_msgs (not deprecated rosidl_target_interfaces).
# rosidl_get_typesupport_target() only works in the generating package; a
# downstream node links the exported C++ typesupport target instead.
if(TARGET mentorpi_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target mentorpi_msgs "rosidl_typesupport_cpp")
  target_link_libraries(person_perception "${cpp_typesupport_target}")
else()
  target_link_libraries(person_perception ${mentorpi_msgs_TARGETS__rosidl_typesupport_cpp})
endif()

install(TARGETS person_perception
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_person_geometry test/test_person_geometry.cpp)
  target_include_directories(test_person_geometry PRIVATE include)
  add_test(NAME test_person_geometry COMMAND test_person_geometry)

  add_executable(test_person_target_lock test/test_person_target_lock.cpp)
  target_include_directories(test_person_target_lock PRIVATE include)
  add_test(NAME test_person_target_lock COMMAND test_person_target_lock)
endif()

ament_package()
