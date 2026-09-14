cmake_minimum_required(VERSION 3.8)
project(mentorpi_person_detect)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# Distro OpenCV (libopencv-dev), not Deptrum /usr/local 4.10 (SD013 D9).
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
set(_mentorpi_opencv_includes "${OpenCV_INCLUDE_DIRS}")
if(_mentorpi_opencv_includes MATCHES "/usr/local")
  message(FATAL_ERROR
    "mentorpi_person_detect must link distro OpenCV, not Deptrum ${_mentorpi_opencv_includes}")
endif()

# NCNN from overlay-builder (/opt/ncnn). No apt or network at colcon time (SD026 AC1/AC3).
if(DEFINED ENV{NCNN_ROOT})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{NCNN_ROOT}")
elseif(NOT ncnn_DIR AND EXISTS "/opt/ncnn/lib/cmake/ncnn")
  list(APPEND CMAKE_PREFIX_PATH "/opt/ncnn")
endif()
find_package(ncnn REQUIRED)

if(TARGET ncnn::ncnn)
  set(_mentorpi_ncnn_target ncnn::ncnn)
elseif(TARGET ncnn)
  set(_mentorpi_ncnn_target ncnn)
else()
  message(FATAL_ERROR "find_package(ncnn) succeeded but no ncnn target")
endif()

# Exported ncnn INTERFACE is .../include/ncnn (so "net.h" works). Sources use
# <ncnn/net.h>, which needs the parent include dir.
# ncnnConfig lives in <prefix>/lib/cmake/ncnn; headers in <prefix>/include/ncnn.
get_filename_component(_mentorpi_ncnn_include "${ncnn_DIR}/../../../include" ABSOLUTE)
if(NOT EXISTS "${_mentorpi_ncnn_include}/ncnn/net.h")
  set(_mentorpi_ncnn_include "/opt/ncnn/include")
endif()
if(NOT EXISTS "${_mentorpi_ncnn_include}/ncnn/net.h")
  message(FATAL_ERROR "ncnn/net.h not under ${_mentorpi_ncnn_include} (ncnn_DIR=${ncnn_DIR})")
endif()

message(STATUS "mentorpi_person_detect: ncnn ${ncnn_VERSION} from ${ncnn_DIR}")
message(STATUS "mentorpi_person_detect: ncnn includes ${_mentorpi_ncnn_include}")

add_library(mentorpi_person_detect_core
  src/byte_track.cpp
  src/yolo11n_detector.cpp
)
target_include_directories(mentorpi_person_detect_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${_mentorpi_ncnn_include}
)
target_link_libraries(mentorpi_person_detect_core
  PUBLIC
    ${_mentorpi_ncnn_target}
  PRIVATE
    ${OpenCV_LIBS}
)

add_executable(person_detect_pi src/person_detect_pi.cpp)
ament_target_dependencies(
  person_detect_pi
  ament_index_cpp
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
  cv_bridge
)
target_link_libraries(person_detect_pi mentorpi_person_detect_core)

install(TARGETS person_detect_pi mentorpi_person_detect_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_byte_track test/test_byte_track.cpp src/byte_track.cpp)
  target_include_directories(test_byte_track PRIVATE include)
  add_test(NAME test_byte_track COMMAND test_byte_track)
  add_executable(test_yolo11n_decode test/test_yolo11n_decode.cpp)
  target_include_directories(test_yolo11n_decode PRIVATE include)
  add_test(NAME test_yolo11n_decode COMMAND test_yolo11n_decode)
endif()

ament_package()
