#!/usr/bin/env bash
# No-CMake fallback build. Prefer CMake for the real build; this exists so the
# demo and tests compile on a bare toolchain (matches the offline constraint).
set -euo pipefail

cd "$(dirname "$0")"
CXX=${CXX:-c++}
FLAGS="-std=c++17 -O2 -Wall -Wextra -pthread -Iinclude"
mkdir -p build data

LIB_SRCS="src/kinematics.cpp src/sensors.cpp src/estimator.cpp \
src/controller.cpp src/route.cpp src/simulation.cpp src/logio.cpp src/log.cpp"
HAL_CODEC_SRCS="src/hal/frame_codec.cpp"
HAL_LINUX_SRCS="src/hal/mpu9250_i2c.cpp src/hal/vesc_socketcan.cpp src/hal/rs485_link.cpp"

echo "[build] run_demo"
# shellcheck disable=SC2086
$CXX $FLAGS apps/run_demo.cpp $LIB_SRCS -o build/run_demo

echo "[build] unit_tests"
# shellcheck disable=SC2086
$CXX $FLAGS -Itests \
  tests/test_main.cpp tests/test_kinematics.cpp tests/test_sensors.cpp \
  tests/test_estimator.cpp tests/test_backtrack.cpp tests/test_hal_codec.cpp \
  $LIB_SRCS $HAL_CODEC_SRCS \
  -o build/unit_tests

echo "[build] op_console"
# shellcheck disable=SC2086
$CXX $FLAGS apps/op_console.cpp $HAL_CODEC_SRCS -o build/op_console

# The on-vehicle daemon needs Linux device APIs (i2c-dev, SocketCAN, termios).
if [[ "$(uname -s)" == "Linux" ]]; then
  echo "[build] run_vehicle"
  # shellcheck disable=SC2086
  $CXX $FLAGS apps/run_vehicle.cpp $LIB_SRCS $HAL_CODEC_SRCS $HAL_LINUX_SRCS \
    -o build/run_vehicle
  echo "[build] can_sniff"
  $CXX $FLAGS apps/can_sniff.cpp -o build/can_sniff
  echo "[done] build/run_demo  build/unit_tests  build/op_console  build/run_vehicle  build/can_sniff"
else
  echo "[skip] run_vehicle (Linux-only device drivers; build it on the Pi)"
  echo "[done] build/run_demo  build/unit_tests  build/op_console"
fi
