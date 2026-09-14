cmake_minimum_required(VERSION 3.8)
project(mentorpi_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/t1-stop
  scripts/t1-uart-halt
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
