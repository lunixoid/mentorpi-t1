cmake_minimum_required(VERSION 3.8)
project(mentorpi_voice)

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
find_package(std_msgs REQUIRED)
find_package(mentorpi_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED IMPORTED_TARGET alsa)

if(NOT DEFINED ENV{VOSK_ROOT} OR "$ENV{VOSK_ROOT}" STREQUAL "")
  message(FATAL_ERROR "VOSK_ROOT is not set (overlay-builder ENV, SD031 I12.2)")
endif()
set(VOSK_ROOT "$ENV{VOSK_ROOT}")
if(NOT EXISTS "${VOSK_ROOT}/libvosk.so" OR NOT EXISTS "${VOSK_ROOT}/vosk_api.h")
  message(FATAL_ERROR "Vosk not found in VOSK_ROOT=${VOSK_ROOT}")
endif()

if(NOT DEFINED ENV{VOSK_MODEL_DIR} OR "$ENV{VOSK_MODEL_DIR}" STREQUAL "")
  message(FATAL_ERROR "VOSK_MODEL_DIR is not set (overlay-builder ENV, SD031 I12.2)")
endif()
set(VOSK_MODEL_DIR "$ENV{VOSK_MODEL_DIR}")
if(NOT IS_DIRECTORY "${VOSK_MODEL_DIR}")
  message(FATAL_ERROR "Vosk model not found in VOSK_MODEL_DIR=${VOSK_MODEL_DIR}")
endif()

add_executable(voice_command src/voice_command.cpp)
ament_target_dependencies(voice_command ament_index_cpp mentorpi_msgs rclcpp std_msgs)
if(TARGET mentorpi_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target mentorpi_msgs "rosidl_typesupport_cpp")
else()
  set(cpp_typesupport_target ${mentorpi_msgs_TARGETS__rosidl_typesupport_cpp})
endif()
target_include_directories(voice_command PRIVATE include "${VOSK_ROOT}")
target_link_libraries(voice_command
  "${cpp_typesupport_target}"
  PkgConfig::ALSA
  "${VOSK_ROOT}/libvosk.so"
  Threads::Threads
)
set_target_properties(voice_command PROPERTIES
  BUILD_RPATH "${VOSK_ROOT}"
  INSTALL_RPATH "$ORIGIN/.."
)
install(TARGETS voice_command DESTINATION lib/${PROJECT_NAME})
install(FILES "${VOSK_ROOT}/libvosk.so" DESTINATION lib)
install(DIRECTORY "${VOSK_MODEL_DIR}/"
  DESTINATION share/${PROJECT_NAME}/models/vosk-model-small-ru-0.22
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()

  add_executable(test_phrase_match test/test_phrase_match.cpp)
  target_include_directories(test_phrase_match PRIVATE include)
  add_test(NAME test_phrase_match COMMAND test_phrase_match)

  add_executable(test_announce_policy test/test_announce_policy.cpp)
  target_include_directories(test_announce_policy PRIVATE include)
  add_test(NAME test_announce_policy COMMAND test_announce_policy)

  add_executable(test_wav_pcm test/test_wav_pcm.cpp)
  target_include_directories(test_wav_pcm PRIVATE include)
  add_test(NAME test_wav_pcm COMMAND test_wav_pcm)

  add_executable(test_vosk_phrases test/test_vosk_phrases.cpp)
  target_include_directories(test_vosk_phrases PRIVATE include "${VOSK_ROOT}")
  target_link_libraries(test_vosk_phrases "${VOSK_ROOT}/libvosk.so" Threads::Threads)
  target_compile_definitions(test_vosk_phrases PRIVATE
    "MENTORPI_VOICE_TEST_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data\""
    "MENTORPI_VOICE_MODEL_DIR=\"${VOSK_MODEL_DIR}\""
  )
  set_target_properties(test_vosk_phrases PROPERTIES BUILD_RPATH "${VOSK_ROOT}")
  add_test(NAME test_vosk_phrases COMMAND test_vosk_phrases)
  set_tests_properties(test_vosk_phrases PROPERTIES TIMEOUT 300)
endif()

ament_package()
