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
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mentorpi_msgs REQUIRED)

# Humble cpp typesupport of mentorpi_msgs:
if(TARGET mentorpi_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target mentorpi_msgs "rosidl_typesupport_cpp")
else()
  set(cpp_typesupport_target ${mentorpi_msgs_TARGETS__rosidl_typesupport_cpp})
endif()

add_executable(motion_control src/motion_control.cpp)
ament_target_dependencies(motion_control rclcpp geometry_msgs sensor_msgs tf2 tf2_ros)
target_link_libraries(motion_control "${cpp_typesupport_target}")
target_include_directories(motion_control PRIVATE include)

# SD036 D6: F13/F14 guard, the only publisher of /control/motion_restriction.
add_executable(obstacle_guard src/obstacle_guard.cpp)
ament_target_dependencies(obstacle_guard rclcpp geometry_msgs sensor_msgs tf2 tf2_ros)
target_link_libraries(obstacle_guard "${cpp_typesupport_target}")
target_include_directories(obstacle_guard PRIVATE include)

install(TARGETS motion_control obstacle_guard DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_follow_control test/test_follow_control.cpp)
  target_include_directories(test_follow_control PRIVATE include)
  add_test(NAME test_follow_control COMMAND test_follow_control)

  # SD036 T1: obstacle points and virtual bumper geometry, no ROS.
  add_executable(test_obstacle_points test/test_obstacle_points.cpp)
  target_include_directories(test_obstacle_points PRIVATE include)
  add_test(NAME test_obstacle_points COMMAND test_obstacle_points)

  add_executable(test_virtual_bumper test/test_virtual_bumper.cpp)
  target_include_directories(test_virtual_bumper PRIVATE include)
  add_test(NAME test_virtual_bumper COMMAND test_virtual_bumper)

  # SD036 T2: avoidance planner, no ROS.
  add_executable(test_avoidance_planner test/test_avoidance_planner.cpp)
  target_include_directories(test_avoidance_planner PRIVATE include test)
  add_test(NAME test_avoidance_planner COMMAND test_avoidance_planner)

  # SD036 T3: guard decision (F13/F14), no ROS.
  add_executable(test_guard_decision test/test_guard_decision.cpp)
  target_include_directories(test_guard_decision PRIVATE include test)
  add_test(NAME test_guard_decision COMMAND test_guard_decision)

  # SD036 T4: 2D closed-loop simulation, no ROS.
  add_executable(test_avoidance_sim test/test_avoidance_sim.cpp)
  target_include_directories(test_avoidance_sim PRIVATE include test)
  add_test(NAME test_avoidance_sim COMMAND test_avoidance_sim)
endif()

ament_package()
