#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Building view_cameras utility..."
g++ -O3 "$SCRIPT_DIR/view_cameras.cpp" -o "$SCRIPT_DIR/view_cameras" \
  -I"$WORKSPACE_ROOT/src/volleyball_vision/3rdparty/mindvision/include" \
  -I"$WORKSPACE_ROOT/src/volleyball_vision/include" \
  -L"$WORKSPACE_ROOT/src/volleyball_vision/3rdparty/mindvision/lib" \
  -L"$WORKSPACE_ROOT/install/volleyball_vision/lib" \
  -lvolleyball_vision_core \
  $(pkg-config --cflags --libs opencv4) \
  -lMVSDK -lpthread \
  -Wl,-rpath="$WORKSPACE_ROOT/src/volleyball_vision/3rdparty/mindvision/lib:$WORKSPACE_ROOT/install/volleyball_vision/lib"

echo "Build complete: $SCRIPT_DIR/view_cameras"
