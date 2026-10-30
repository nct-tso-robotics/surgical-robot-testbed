#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
workspace="$PWD"
environment="${PIXI_ENVIRONMENT_NAME:?Run through Pixi}"
cmake -S src/robots_controllers/rcm_solvers -B "build/$environment/core" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror=return-type" \
  -DCMAKE_INSTALL_PREFIX="$workspace/install/$environment/core" \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DTESTBED_WITH_FRANKA="${TESTBED_WITH_FRANKA:-0}" \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"
cmake --build "build/$environment/core" --parallel "${TESTBED_BUILD_JOBS:-2}"
cmake --install "build/$environment/core"
