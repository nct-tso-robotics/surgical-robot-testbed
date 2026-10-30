export ROS_ROOT="" ROS_PACKAGE_PATH="" PYTHONPATH=""
export AMENT_PREFIX_PATH="" COLCON_PREFIX_PATH=""
unset AMENT_CURRENT_PREFIX _CATKIN_SETUP_DIR
export CMAKE_PREFIX_PATH="$CONDA_PREFIX"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib"
. "$CONDA_PREFIX/setup.sh"
if [ -f "$PIXI_PROJECT_ROOT/install/ros2/local_setup.sh" ]; then
    . "$PIXI_PROJECT_ROOT/install/ros2/local_setup.sh"
fi
. "$PIXI_PROJECT_ROOT/tools/activate_core.sh"
