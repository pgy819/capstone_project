cmake_minimum_required(VERSION 3.16)
project(rcws_rpi CXX)

set(CMAKE_CXX_STANDARD 20)   # libcamera >= 0.5 headers use std::span
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_compile_options(-mcpu=cortex-a76)   # Raspberry Pi 5
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d imgcodecs)
find_package(Threads REQUIRED)
find_package(PkgConfig)

set(RCWS_WITH_LIBCAMERA AUTO CACHE STRING "Build the libcamera frame source (AUTO|ON|OFF)")
set(RCWS_WITH_HAILO AUTO CACHE STRING "Build the HailoRT detector (AUTO|ON|OFF)")

add_library(rcws_core STATIC
  src/common/common.cpp
  src/common/config.cpp
  src/estimation/camera_model.cpp
  src/estimation/imm.cpp
  src/estimation/estimator.cpp
  src/perception/zncc_tracker.cpp
  src/perception/roi.cpp
  src/perception/perception.cpp
  src/detect/detector.cpp
  src/record/binlog.cpp
  src/record/clip.cpp
  src/sim/synthetic.cpp
)
target_include_directories(rcws_core PUBLIC src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rcws_core PUBLIC ${OpenCV_LIBS} Threads::Threads)

# ---- libcamera source (Pi 5)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(LIBCAMERA libcamera)
endif()
if(LIBCAMERA_FOUND AND NOT RCWS_WITH_LIBCAMERA STREQUAL "OFF")
  target_sources(rcws_core PRIVATE src/camera/libcamera_source.cpp)
  target_include_directories(rcws_core PUBLIC ${LIBCAMERA_INCLUDE_DIRS})
  target_link_libraries(rcws_core PUBLIC ${LIBCAMERA_LINK_LIBRARIES})
  target_compile_definitions(rcws_core PUBLIC RCWS_HAVE_LIBCAMERA=1)
  message(STATUS "rcws: libcamera source ON (${LIBCAMERA_VERSION})")
else()
  target_sources(rcws_core PRIVATE src/camera/libcamera_stub.cpp)
  message(STATUS "rcws: libcamera source OFF")
endif()

# ---- HailoRT detector (Hailo-8)
if(NOT RCWS_WITH_HAILO STREQUAL "OFF")
  find_package(HailoRT QUIET)
endif()
if(HailoRT_FOUND)
  target_sources(rcws_core PRIVATE src/detect/hailo_detector.cpp)
  target_link_libraries(rcws_core PUBLIC HailoRT::libhailort)
  target_compile_definitions(rcws_core PUBLIC RCWS_HAVE_HAILO=1)
  message(STATUS "rcws: HailoRT detector ON")
else()
  message(STATUS "rcws: HailoRT detector OFF")
endif()

add_executable(rcws_vision src/app/main.cpp src/app/preview_server.cpp)
target_link_libraries(rcws_vision PRIVATE rcws_core)
find_package(OpenCV QUIET COMPONENTS highgui)
if(OpenCV_highgui_FOUND OR TARGET opencv_highgui)
  target_link_libraries(rcws_vision PRIVATE opencv_highgui)
  target_compile_definitions(rcws_vision PRIVATE RCWS_HAVE_HIGHGUI=1)
  message(STATUS "rcws: --window popup ON (OpenCV highgui)")
else()
  message(STATUS "rcws: --window popup OFF (no OpenCV highgui)")
endif()

# ---- tests (no external framework)
enable_testing()
add_executable(rcws_tests
  tests/test_main.cpp
  tests/test_geometry.cpp
  tests/test_imm.cpp
  tests/test_zncc.cpp
  tests/test_pipeline.cpp
  tests/test_preproc.cpp
)
target_link_libraries(rcws_tests PRIVATE rcws_core)
add_test(NAME rcws_tests COMMAND rcws_tests)
