# Robot control framework

RCM motion control with a ROS-independent C++ core and ROS 2 Jazzy adapters.

`rcm_solvers/` contains kinematics, trajectory profiling and Cartesian/joint
velocity solvers. It uses Eigen and Orocos KDL, with optional `pyrcm_solvers`
Python bindings. It does not import or link ROS.

`controllers/` contains the ROS control plugins, UR5e interface mapping,
broadcaster and launch files. Sensorium consumes the published robot state;
it is not a dependency of the motion solver or controller loop.

## Build

From the enclosing robot-testbed workspace:

```bash
pixi run --locked -e core build-core
pixi run --locked -e ros2 build
pixi shell -e ros2
```

The core also builds independently with ordinary CMake:

```bash
cmake -S rcm_solvers -B build -DCMAKE_INSTALL_PREFIX="$PWD/install" \
  -DBUILD_PYTHON_BINDINGS=ON
cmake --build build
cmake --install build
```

Consumers use `find_package(rcm_solvers CONFIG REQUIRED)` and link
`rcm_solvers::rcm_solvers`. Python constructors take a `RobotType`, own their
kinematics, and expose the common `VelocitySolver` API.

## Run

The supported mock robot integrates joint velocity commands without hardware
or the UR driver:

```bash
ros2 launch testbed_launchers mock.launch.py namespace:=robot1
```

The default topics are `/robot1/robot_state` and `/robot1/set_tip_pose`.
A small one-shot command, also accepted without a source timestamp:

```bash
ros2 topic pub --once /robot1/set_tip_pose testbed_msgs/msg/TargetTipPose \
  '{target_delta_position: {x: 0.002}, target_delta_roll: 0.0, movement_time: 0.3}'
```

For a physical UR5e, run from the workspace root and supply its address and
extracted calibration.

```bash
ros2 launch testbed_launchers ur5e_joint_rcm_controller.launch.py \
  robot_ip:=ROBOT_IP kinematics_params_file:=calibration/robot.yaml \
  namespace:=robot1 ee_frame_rcm_point:='[0.0, 0.0, 0.1]' \
  ee_frame_tip_point:='[0.0, 0.0, 0.3]'
```

The physical launch additionally requires `ur_robot_driver` and `ur_description`.

## Command and state contracts

`TargetTipPose` contains a displacement in meters expressed in `base_frame`, a
shaft roll increment in radians and a positive planning horizon in seconds.
The latest accepted command replaces the remaining trajectory, including zero
components. Repeated commands form a receding horizon; increments are not queued
or guaranteed to accumulate when a newer command arrives before completion.
Profilers may reduce displacement to respect configured limits.

Publishers should use a unique `source_id` per process/session, increment
`sequence` from 1, and stamp every command. Duplicate or decreasing sequences
from a source are discarded. Sequence 0 supports legacy senders: nonzero stamps
must increase; a zero stamp is treated as a new receipt. Up to 64 sources are
tracked per controller instance. Commands are ignored while inactive.

`header.frame_id`, when supplied, must match `frame_prefix + base_frame`
(`robot1/base_link` in the default launch).
The default transport age limit is 0.25 seconds. Remaining motion is cancelled
after 1 second without a fresh accepted command, even if a longer horizon was
requested. Deactivation cancels motion and writes zero joint commands. Invalid
state samples stop the controller. Controller and broadcaster retain their
reference pivot across deactivate/reactivate; reconfigure both to establish a
new reference. Bounds, timeout, linear limits and separate
angular limits are configurable ROS parameters.

Both controller and broadcaster derive their KDL chain from the same
controller-manager `robot_description`, using `base_frame` and `tip_frame`
(default `base_link` and `tool0`). Consequently the physical robot calibration
is shared with state publication, including URDF joint position/velocity and
soft safety limits. Tip and pivot offsets are expressed in
`tip_frame`. Launch files set `frame_prefix` to the robot namespace followed by
`/`, consistently in robot_state_publisher and both control plugins. For example,
`namespace:=robot2` publishes `robot2/base_link` through `robot2/tool0` on TF and
uses `robot2/base_link` in state and command headers. URDF link and joint names
remain unchanged for kinematics and hardware mapping. The low-level plugin
parameter defaults to an empty prefix; custom launch files must set it to match
robot_state_publisher. UR's separate `tf_prefix` must remain empty because it
also renames hardware joints.

Both mock and UR descriptions have an identity `world` → `base_link` fixed
joint, with both frames prefixed per robot. To connect two robots, publish the
interbase calibration from `robot1/base_link` to `robot2/world`; the latter
coincides with `robot2/base_link`. This keeps one parent for each TF frame.

Physical and mock bringup also publish a static `tool0` → `tool_tip` transform
(using the configured `tip_frame` as parent on physical robots). Both frame names
receive the same namespace prefix. Its translation is exactly `ee_frame_tip_point`
and its rotation is identity, so the TF tip position matches the position in
`RobotState`. Mock bringup accepts the same `ee_frame_tip_point` and
`ee_frame_rcm_point` arguments as physical bringup.

Robot state uses the control-cycle timestamp and the prefixed base frame. Effort and wrench
interfaces are optional; unavailable measurements are NaN rather than fabricated
zeros. Enable `read_effort` and `read_wrench` only when hardware supplies them.

## Hardware capabilities

UR5e has a ROS 2 joint-velocity adapter and the supported mock configuration.
Franka Panda kinematics are enabled separately with
`-DTESTBED_WITH_FRANKA=ON` (default OFF), or the workspace `core-franka` Pixi
environment. This includes the core model, factory and Python binding without
vendor SDK or ROS dependencies. `pyrcm_solvers.with_franka` reports availability.
