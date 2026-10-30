#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
workspace="$PWD"
skip_packages=()
if [ "${TESTBED_WITH_ISAAC:-0}" != 1 ]; then
  skip_packages+=(testbed_isaac_ur5e_system)
fi
hardware_options=()
if [ "${TESTBED_WITH_LAMBDA:-0}" != 1 ]; then
  skip_packages+=(lambda)
else
  hardware_options+=("-DFORCE_DIMENSION_SDK_ROOT=${FORCE_DIMENSION_SDK_ROOT:?Set the Lambda SDK directory}")
fi
selection_options=()
if [ "${#skip_packages[@]}" -gt 0 ]; then
  selection_options=(--packages-skip "${skip_packages[@]}")
fi
colcon --log-base logs/ros2 build --base-paths src/testbed_msgs src/robots_controllers/controllers src/input_devices \
  --build-base build/ros2/packages --install-base install/ros2 --merge-install \
  --executor sequential "${selection_options[@]}" \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror=return-type" \
  -Drcm_solvers_DIR="$workspace/install/ros2/core/lib/cmake/rcm_solvers" \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" "${hardware_options[@]}"
