#!/bin/bash
set -e

if [ -z "$PICO_DIR" ]; then
  echo "Error: PICO_DIR is not set"
  exit 1
fi

SRC_DIR="$PICO_DIR/XRoboToolkit-PC-Service"
PYBIND_DIR="$PICO_DIR/XRoboToolkit-PC-Service-Pybind"

mkdir -p "$PYBIND_DIR/lib"
mkdir -p "$PYBIND_DIR/include"

cp "$SRC_DIR/RoboticsService/PXREARobotSDK/PXREARobotSDK.h" \
   "$PYBIND_DIR/include/"

cp -r "$SRC_DIR/RoboticsService/PXREARobotSDK/nlohmann" \
   "$PYBIND_DIR/include/nlohmann/"

cp "$SRC_DIR/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so" \
   "$PYBIND_DIR/lib/"

pip3 install -e .
