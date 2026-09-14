cmake_minimum_required(VERSION 3.8)
project(t1ctl VERSION 1.9.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(T1CTL_ROS_PROBE_PY_DELIM "T1CTL_PY_END")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/ros_probe.py" T1CTL_ROS_PROBE_PY_CONTENT)
string(FIND "${T1CTL_ROS_PROBE_PY_CONTENT}" "${T1CTL_ROS_PROBE_PY_DELIM}" _delim_pos)
if(NOT _delim_pos EQUAL -1)
  message(FATAL_ERROR "ros_probe.py must not contain embed delimiter ${T1CTL_ROS_PROBE_PY_DELIM}")
endif()
set(T1CTL_ROS_MODE_PY_DELIM "T1CTL_MODE_END")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/ros_mode.py" T1CTL_ROS_MODE_PY_CONTENT)
string(FIND "${T1CTL_ROS_MODE_PY_CONTENT}" "${T1CTL_ROS_MODE_PY_DELIM}" _mode_delim_pos)
if(NOT _mode_delim_pos EQUAL -1)
  message(FATAL_ERROR "ros_mode.py must not contain embed delimiter ${T1CTL_ROS_MODE_PY_DELIM}")
endif()
set(T1CTL_ROS_DEBUG_PY_DELIM "T1CTL_DEBUG_END")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/ros_debug.py" T1CTL_ROS_DEBUG_PY_CONTENT)
string(FIND "${T1CTL_ROS_DEBUG_PY_CONTENT}" "${T1CTL_ROS_DEBUG_PY_DELIM}" _debug_delim_pos)
if(NOT _debug_delim_pos EQUAL -1)
  message(FATAL_ERROR "ros_debug.py must not contain embed delimiter ${T1CTL_ROS_DEBUG_PY_DELIM}")
endif()
set(T1CTL_ROS_DETECT_PY_DELIM "T1CTL_DETECT_END")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/ros_detect.py" T1CTL_ROS_DETECT_PY_CONTENT)
string(FIND "${T1CTL_ROS_DETECT_PY_CONTENT}" "${T1CTL_ROS_DETECT_PY_DELIM}" _detect_delim_pos)
if(NOT _detect_delim_pos EQUAL -1)
  message(FATAL_ERROR "ros_detect.py must not contain embed delimiter ${T1CTL_ROS_DETECT_PY_DELIM}")
endif()
set(T1CTL_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${T1CTL_GENERATED_DIR}")
file(WRITE "${T1CTL_GENERATED_DIR}/ros_probe_embed.hpp"
"#pragma once

namespace units {

inline constexpr const char kRosProbePy[] = R\"${T1CTL_ROS_PROBE_PY_DELIM}(
${T1CTL_ROS_PROBE_PY_CONTENT})${T1CTL_ROS_PROBE_PY_DELIM}\";

}  // namespace units
")
file(WRITE "${T1CTL_GENERATED_DIR}/ros_mode_embed.hpp"
"#pragma once

namespace units {

inline constexpr const char kRosModePy[] = R\"${T1CTL_ROS_MODE_PY_DELIM}(
${T1CTL_ROS_MODE_PY_CONTENT})${T1CTL_ROS_MODE_PY_DELIM}\";

}  // namespace units
")
file(WRITE "${T1CTL_GENERATED_DIR}/ros_debug_embed.hpp"
"#pragma once

namespace units {

inline constexpr const char kRosDebugPy[] = R\"${T1CTL_ROS_DEBUG_PY_DELIM}(
${T1CTL_ROS_DEBUG_PY_CONTENT})${T1CTL_ROS_DEBUG_PY_DELIM}\";

}  // namespace units
")
file(WRITE "${T1CTL_GENERATED_DIR}/ros_detect_embed.hpp"
"#pragma once

namespace units {

inline constexpr const char kRosDetectPy[] = R\"${T1CTL_ROS_DETECT_PY_DELIM}(
${T1CTL_ROS_DETECT_PY_CONTENT})${T1CTL_ROS_DETECT_PY_DELIM}\";

}  // namespace units
")

include(FetchContent)
# CLI11 1.5.x is the C++ Clap analog named in the design stack (1.4 also exists).
# Skip its CMake so we do not install CLI11 or build its tests.
FetchContent_Declare(
  clap
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v1.5.4
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR "cmake-do-not-add"
)
FetchContent_MakeAvailable(clap)

add_executable(t1ctl
  src/main.cpp
  src/ui.cpp
  src/units.cpp
  src/viewer.cpp
  src/calib.cpp
)
target_include_directories(t1ctl PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${T1CTL_GENERATED_DIR}"
)
target_include_directories(t1ctl SYSTEM PRIVATE "${clap_SOURCE_DIR}/include")
target_compile_definitions(t1ctl PRIVATE T1CTL_VERSION="${PROJECT_VERSION}")
target_compile_options(t1ctl PRIVATE -Wall -Wextra -Wpedantic)
target_compile_features(t1ctl PRIVATE cxx_std_17)

enable_testing()
add_executable(t1ctl_test
  tests/test_status.cpp
  src/ui.cpp
  src/units.cpp
  src/viewer.cpp
  src/calib.cpp
)
target_include_directories(t1ctl_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${T1CTL_GENERATED_DIR}"
)
target_compile_definitions(t1ctl_test PRIVATE T1CTL_VERSION="${PROJECT_VERSION}")
target_compile_options(t1ctl_test PRIVATE -Wall -Wextra -Wpedantic)
target_compile_features(t1ctl_test PRIVATE cxx_std_17)
add_test(NAME t1ctl_test COMMAND t1ctl_test)

install(TARGETS t1ctl DESTINATION bin)
