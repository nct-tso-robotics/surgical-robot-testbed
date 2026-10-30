if [ "$PIXI_ENVIRONMENT_NAME" != ros2 ]; then
    export ROS_ROOT="" ROS_PACKAGE_PATH="" PYTHONPATH=""
    export AMENT_PREFIX_PATH="" COLCON_PREFIX_PATH=""
    unset ROS_VERSION ROS_DISTRO AMENT_CURRENT_PREFIX _CATKIN_SETUP_DIR
    export CMAKE_PREFIX_PATH="$CONDA_PREFIX"
    export LD_LIBRARY_PATH="$CONDA_PREFIX/lib"
fi
testbed_core_prefix="$PIXI_PROJECT_ROOT/install/$PIXI_ENVIRONMENT_NAME/core"
export CMAKE_PREFIX_PATH="$testbed_core_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$testbed_core_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$testbed_core_prefix/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
unset testbed_core_prefix
