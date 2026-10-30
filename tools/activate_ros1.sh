export ROS_ROOT="" ROS_PACKAGE_PATH="" PYTHONPATH=""
export AMENT_PREFIX_PATH="" COLCON_PREFIX_PATH=""
unset AMENT_CURRENT_PREFIX _CATKIN_SETUP_DIR
export CMAKE_PREFIX_PATH="$CONDA_PREFIX"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib"
. "$CONDA_PREFIX/setup.sh"

if [ -f "$PIXI_PROJECT_ROOT/install/ros1/setup.sh" ]; then
    . "$PIXI_PROJECT_ROOT/install/ros1/setup.sh" --extend
fi
export LD_LIBRARY_PATH="$PIXI_PROJECT_ROOT/install/ros1/lib:$PIXI_PROJECT_ROOT/install/solvers/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$PIXI_PROJECT_ROOT/src/tso_robotics_sockets/src:$PIXI_PROJECT_ROOT/src/versatil_constants/src${PYTHONPATH:+:$PYTHONPATH}"
