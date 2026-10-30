# Robot testbed

Jazzy workspace for the delta-based RCM controller, independent state broadcaster,
input devices, policy interface, endoscope capture and Sensorium recording.
The controller and message repositories use `refactor/update` as their main-branch
baseline. The input repository starts from Jazzy and adopts the same delta interface.
The previous Noetic setup is preserved on the repositories' Noetic branches.

## Install and build

Requires Pixi and access to the repositories listed in `sources.repos`.
Import the pinned sources before installing the environments, because the Python
packages are installed from those checkouts:

```bash
pixi exec --spec vcstool --spec pyyaml bash tools/fetch_sources.sh
pixi install --locked -e ros2
pixi run --locked -e ros2 build
```

`sources.repos` pins application source commits; `pixi.lock` pins toolchain and
runtime dependencies. The importer leaves existing checkouts in place and reports
revision differences or local edits. It never resets a working tree.

The `core` environment has no ROS dependencies. It builds the shared C++ solver
and Python bindings and installs the NumPy-based input core:

```bash
pixi install --locked -e core
pixi run --locked -e core build-core
```

Build and install directories are separate for each environment. The ROS2
activation sources only its own environment and workspace overlay. Do not source
system ROS or the Noetic workspace into it.

## Run

Use a separate terminal for each long-running process. The default robot namespace
is `robot1`; state is `/robot1/robot_state` and commands are `/robot1/set_tip_pose`.

```bash
# Supported simulator: no robot connection or physical movement.
pixi run --locked -e ros2 mock

# Physical UR robot: supply its address and its own extracted calibration.
pixi run --locked -e ros2 robot robot_ip:=<address> kinematics_params_file:=<file.yaml>

# Camera runs in its own capture environment.
pixi run --locked -e ros2 camera-build
pixi run --locked -e ros2 camera

# Record robot state and camera streams with Sensorium's browser interface.
pixi run --locked -e ros2 record-ui

# Optional policy RPC server; requires live state and synchronized stereo input.
pixi run --locked -e ros2 model
```

Configure input-device paths and scaling through the launch arguments documented
in `src/input_devices/README.md`. Camera mount transforms are published by
`testbed_transforms`, with an explicit calibration artifact. Sensorium provides
the reusable calibration/processing mathematics and records the published state.

Commands are motion deltas in metres/radians with movement duration, source ID,
sequence and source timestamp. A newer accepted command preempts the current
motion; commands are not an additive queue. Inputs do not accumulate targets.
The controller validates command age, frame and limits. Controller and broadcaster
derive their kinematics from the same robot description.

The optional WebSocket gripper state reports the last delivered command, not a
measured physical opening. The Jazzy recording profiles retain that distinction.
Camera timestamps represent host capture-callback arrival, not hardware exposure.

## Optional hardware

The default build does not require the Lambda SDK or Isaac integration.
Set `TESTBED_WITH_ISAAC=1` to include the optional Isaac system plugin.
To build Lambda, provide its Force Dimension SDK and enable the package:

```bash
TESTBED_WITH_LAMBDA=1 FORCE_DIMENSION_SDK_ROOT=<sdk-directory> pixi run --locked -e ros2 build
```

The UR hardware backend is supported by the Jazzy implementation. Optional Franka
kinematics and Python bindings can be built with `pixi run -e core-franka build-core`.
This does not implement a Jazzy Franka hardware adapter; the existing hardware
backend remains available in the preserved Noetic implementation. The default
core/UR build excludes Franka. Blackmagic capture still requires its host driver/runtime;
Pixi manages userspace dependencies, not kernel drivers.
