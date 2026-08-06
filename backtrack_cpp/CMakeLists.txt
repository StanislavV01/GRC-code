cmake_minimum_required(VERSION 3.16)
project(backtrack_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Threads REQUIRED)

# Core library: the on-vehicle real-time logic (estimator + controller + model).
add_library(backtrack
  src/kinematics.cpp
  src/sensors.cpp
  src/estimator.cpp
  src/controller.cpp
  src/route.cpp
  src/simulation.cpp
  src/logio.cpp
  src/log.cpp
)
target_include_directories(backtrack PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(backtrack PUBLIC Threads::Threads)

# HAL: wire-protocol codecs are portable (unit-tested on any host); the device
# drivers (I2C IMU, SocketCAN VESC, RS485 UART) are Linux-only.
add_library(backtrack_hal src/hal/frame_codec.cpp)
target_include_directories(backtrack_hal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_sources(backtrack_hal PRIVATE
    src/hal/mpu9250_i2c.cpp
    src/hal/vesc_socketcan.cpp
    src/hal/rs485_link.cpp
  )
  target_link_libraries(backtrack_hal PRIVATE backtrack)  # log.hpp
endif()

# Demo executable.
add_executable(run_demo apps/run_demo.cpp)
target_link_libraries(run_demo PRIVATE backtrack)

# On-vehicle daemon (CM4): needs the Linux device drivers.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_executable(run_vehicle apps/run_vehicle.cpp)
  target_link_libraries(run_vehicle PRIVATE backtrack backtrack_hal)

  # Passive DroneCAN bus recon for the first live hookup.
  add_executable(can_sniff apps/can_sniff.cpp)
endif()

# Bench operator console (plain termios -- builds on Linux and macOS).
add_executable(op_console apps/op_console.cpp)
target_link_libraries(op_console PRIVATE backtrack_hal)

# Tests (zero-dependency harness).
enable_testing()
add_executable(unit_tests
  tests/test_main.cpp
  tests/test_kinematics.cpp
  tests/test_sensors.cpp
  tests/test_estimator.cpp
  tests/test_backtrack.cpp
  tests/test_hal_codec.cpp
)
target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(unit_tests PRIVATE backtrack backtrack_hal)
add_test(NAME unit_tests COMMAND unit_tests)
