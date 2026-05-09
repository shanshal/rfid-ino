#!/usr/bin/env bash
set -euo pipefail

SKETCH_DIR="scanner"
PORT="/dev/ttyUSB0"
FQBN="esp32:esp32:esp32"

echo "Compiling and uploading $SKETCH_DIR to $PORT ($FQBN)..."
arduino-cli compile \
  --fqbn "$FQBN" \
  --board-options UploadSpeed=460800,FlashMode=qio,FlashFreq=80 \
  -p "$PORT" \
  --upload \
  "$SKETCH_DIR"

echo "Opening serial monitor on $PORT..."
echo "Press Ctrl+C to exit monitor."
arduino-cli monitor -p "$PORT" -c baudrate=115200
