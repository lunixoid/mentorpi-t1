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

# SD033 I14.2: static RNNoise and SpeexDSP from the overlay-builder image.
if(NOT DEFINED ENV{RNNOISE_ROOT} OR "$ENV{RNNOISE_ROOT}" STREQUAL "")
  message(FATAL_ERROR "RNNOISE_ROOT is not set (overlay-builder ENV, SD033 I14.2)")
endif()
set(RNNOISE_ROOT "$ENV{RNNOISE_ROOT}")
if(NOT EXISTS "${RNNOISE_ROOT}/lib/librnnoise.a" OR NOT EXISTS "${RNNOISE_ROOT}/include/rnnoise.h")
  message(FATAL_ERROR "RNNoise not found in RNNOISE_ROOT=${RNNOISE_ROOT}")
endif()
if(NOT DEFINED ENV{SPEEXDSP_ROOT} OR "$ENV{SPEEXDSP_ROOT}" STREQUAL "")
  message(FATAL_ERROR "SPEEXDSP_ROOT is not set (overlay-builder ENV, SD033 I14.2)")
endif()
set(SPEEXDSP_ROOT "$ENV{SPEEXDSP_ROOT}")
if(NOT EXISTS "${SPEEXDSP_ROOT}/lib/libspeexdsp.a"
   OR NOT EXISTS "${SPEEXDSP_ROOT}/include/speex/speex_preprocess.h")
  message(FATAL_ERROR "SpeexDSP not found in SPEEXDSP_ROOT=${SPEEXDSP_ROOT}")
endif()
add_library(mentorpi_voice_denoise INTERFACE)
target_include_directories(mentorpi_voice_denoise INTERFACE
  "${RNNOISE_ROOT}/include"
  "${SPEEXDSP_ROOT}/include"
)
target_link_libraries(mentorpi_voice_denoise INTERFACE
  "${RNNOISE_ROOT}/lib/librnnoise.a"
  "${SPEEXDSP_ROOT}/lib/libspeexdsp.a"
  m
)

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
  mentorpi_voice_denoise
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

# SD033 I14.3: offline bench, no ROS and no ALSA.
add_executable(voice_bench src/voice_bench.cpp)
target_include_directories(voice_bench PRIVATE include "${VOSK_ROOT}")
target_link_libraries(voice_bench
  mentorpi_voice_denoise
  "${VOSK_ROOT}/libvosk.so"
  Threads::Threads
)
set_target_properties(voice_bench PROPERTIES
  BUILD_RPATH "${VOSK_ROOT}"
  INSTALL_RPATH "$ORIGIN/.."
)
install(TARGETS voice_bench DESTINATION lib/${PROJECT_NAME})

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

  add_executable(test_audio_dsp test/test_audio_dsp.cpp)
  target_include_directories(test_audio_dsp PRIVATE include)
  add_test(NAME test_audio_dsp COMMAND test_audio_dsp)

  add_executable(test_wav_writer test/test_wav_writer.cpp)
  target_include_directories(test_wav_writer PRIVATE include)
  add_test(NAME test_wav_writer COMMAND test_wav_writer)

  add_executable(test_denoiser test/test_denoiser.cpp)
  target_include_directories(test_denoiser PRIVATE include)
  target_link_libraries(test_denoiser mentorpi_voice_denoise)
  add_test(NAME test_denoiser COMMAND test_denoiser)

  add_executable(test_fft test/test_fft.cpp)
  target_include_directories(test_fft PRIVATE include)
  add_test(NAME test_fft COMMAND test_fft)

  add_executable(test_dual_channel test/test_dual_channel.cpp)
  target_include_directories(test_dual_channel PRIVATE include)
  add_test(NAME test_dual_channel COMMAND test_dual_channel)

  add_executable(test_utterance test/test_utterance.cpp)
  target_include_directories(test_utterance PRIVATE include)
  add_test(NAME test_utterance COMMAND test_utterance)

  add_executable(test_command_pipeline test/test_command_pipeline.cpp)
  target_include_directories(test_command_pipeline PRIVATE include "${VOSK_ROOT}")
  target_link_libraries(test_command_pipeline
    mentorpi_voice_denoise
    "${VOSK_ROOT}/libvosk.so"
    Threads::Threads
  )
  set_target_properties(test_command_pipeline PROPERTIES BUILD_RPATH "${VOSK_ROOT}")
  add_test(NAME test_command_pipeline COMMAND test_command_pipeline)

  add_executable(test_bench_score test/test_bench_score.cpp)
  target_include_directories(test_bench_score PRIVATE include)
  add_test(NAME test_bench_score COMMAND test_bench_score)

  # SD033 D8.1: fixtures come from $MENTORPI_VOICE_TEST_DATA_DIR at run time; exit 77 = skipped.
  add_executable(test_vosk_phrases test/test_vosk_phrases.cpp)
  target_include_directories(test_vosk_phrases PRIVATE include "${VOSK_ROOT}")
  target_link_libraries(test_vosk_phrases
    mentorpi_voice_denoise
    "${VOSK_ROOT}/libvosk.so"
    Threads::Threads
  )
  target_compile_definitions(test_vosk_phrases PRIVATE
    "MENTORPI_VOICE_MODEL_DIR=\"${VOSK_MODEL_DIR}\""
  )
  set_target_properties(test_vosk_phrases PROPERTIES BUILD_RPATH "${VOSK_ROOT}")
  add_test(NAME test_vosk_phrases COMMAND test_vosk_phrases)
  set_tests_properties(test_vosk_phrases PROPERTIES TIMEOUT 300 SKIP_RETURN_CODE 77)
endif()

ament_package()
