# Surgical Robot Testbed

ROS 2 Jazzy workspace for the surgical robotics platform described in
[An Open-Source Robotics Research Platform for Autonomous Laparoscopic Surgery](https://arxiv.org/abs/2603.08490).
It includes delta-based RCM control, robot state publishing, teleoperation,
stereo endoscope capture, Sensorium recording and a policy inference interface.

## Install and build

Requires Pixi on 64-bit x86 Linux. The ROS controllers, input devices and
message definitions are included under `src/`. Fetch the remaining public
dependencies before installing the environments.

```bash
git clone https://github.com/nct-tso-robotics/surgical-robot-testbed.git
cd surgical-robot-testbed
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

Build and install directories are separate for each environment. Use Pixi to
activate the Jazzy environment and its workspace overlay.

## Run

Use a separate terminal for each long-running process. The default robot namespace
is `robot1`; state is `/robot1/robot_state` and commands are `/robot1/set_tip_pose`.

```bash
# Supported simulator: no robot connection or physical movement.
pixi run --locked -e ros2 mock

# Physical UR robot: supply its address and its own extracted calibration.
pixi run --locked -e ros2 robot robot_ip:=ROBOT_IP kinematics_params_file:=calibration/robot.yaml

# Camera runs in its own capture environment.
pixi run --locked -e ros2 camera-build
pixi run --locked -e ros2 camera

# Record robot state and camera streams with Sensorium's browser interface.
pixi run --locked -e ros2 record-ui

# Optional policy RPC server; requires live state and synchronized stereo input.
pixi run --locked -e ros2 model
```

Configure input-device paths and scaling through the launch arguments documented
in `src/input_devices/README.md`. Robot2 carries the endoscope: its kinematic TF
chain and configured tool-tip offset determine the camera position, while
`testbed_transforms` supplies the fixed camera-axis rotation from the mount
calibration. The camera reference origin is the modeled endoscope tip. Supply
the calibrated relationship between robot bases in TF. Consumers select their robot
base and camera frames and resolve TF at robot acquisition timestamps.
Sensorium provides the reusable calibration/processing mathematics and records
the selected per-robot transform streams. Endoscope guidance uses robot2.

Commands are motion deltas in metres/radians with movement duration, source ID,
sequence and source timestamp. A newer accepted command preempts the current
motion; commands are not an additive queue. Inputs do not accumulate targets.
The controller validates command age, frame and limits. Controller and broadcaster
derive their kinematics from the same robot description.

The optional `gripper_server` WebSocket bridge reports the latest requested gripper
command, including while its browser is disconnected. Recording that state does
not provide delivery acknowledgment or measured physical opening.
Camera timestamps represent host capture-callback arrival, not hardware exposure.
The recording UI loads the files in `configs/recording/` and `configs/dataset/`.
Their includes are relative to the configuration files. The policy interface
uses Sensorium's packaged stereo calibration by default. The camera mount
package ships an [example mount calibration](src/input_devices/testbed_transforms/config/endoscope_mount.example.yaml).
For a physical UR, use a kinematics calibration extracted from that robot.

## Optional hardware

The default build does not require the Lambda SDK.
To build Lambda, provide its Force Dimension SDK and enable the package:

```bash
TESTBED_WITH_LAMBDA=1 FORCE_DIMENSION_SDK_ROOT=<sdk-directory> pixi run --locked -e ros2 build
```

The Jazzy hardware backend supports UR robots. Optional Franka support covers
kinematics and Python bindings and builds with `pixi run -e core-franka build-core`.
Blackmagic capture requires the Desktop Video driver and runtime on the capture host.

## Maintainers

- Lorenzo Mazza <lorenzo.mazza@nct-dresden.de>
- Ariel Rodriguez <ariel.rodriguezjimenez@nct-dresden.de>

## License

The workspace is distributed under the [MIT License](LICENSE). See the
[source and license notices](NOTICE.md) for the bundled packages.
