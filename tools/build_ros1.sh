#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace"

if ! patch --dry-run --silent --reverse --fuzz=0 -p1 < tools/vendor-cxx17.patch >/dev/null 2>&1; then
    patch --batch --forward --fuzz=0 -p1 < tools/vendor-cxx17.patch
fi

cmake -S src/robots_controllers/rcm_solvers -B build/solvers \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_EXTENSIONS=OFF \
    -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror=return-type" \
    -DCMAKE_INSTALL_PREFIX="$workspace/install/solvers" \
    -DBUILD_PYTHON_BINDINGS=OFF
cmake --build build/solvers --parallel 2
cmake --install build/solvers

catkin config --profile ros1 --workspace "$workspace" \
    --source-space src --build-space build/ros1 --devel-space devel/ros1 \
    --install-space install/ros1 --log-space logs/ros1 \
    --install --merge-install --extend "$CONDA_PREFIX" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_EXTENSIONS=OFF \
    -DCMAKE_CXX_FLAGS="-Wall -Wextra" \
    -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python" \
    -DPython_EXECUTABLE="$CONDA_PREFIX/bin/python" \
    -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
    -Drcm_solvers_DIR="$workspace/install/solvers/lib/cmake/rcm_solvers" \
    -DCATKIN_ENABLE_TESTING=OFF -DBUILD_TESTING=OFF

catkin build --profile ros1 --workspace "$workspace" --no-status \
    -j 2 -p 1 rcm_controller ur_robot_driver ur_description \
    cartesian_trajectory_controller twist_controller \
    industrial_robot_status_controller robot_pose_setters \
    lambda pedal_board ps4_controller language_publisher
