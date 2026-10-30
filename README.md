# Robot testbed

ROS Noetic and C++17 on Linux x86-64, using Pixi and RoboStack.

```bash
pixi install --locked -e ros1
pixi run --locked -e ros1 build
pixi shell -e ros1
```

The build installs the RCM solvers, UR controllers and driver, messages, and
input-device and pose-setter nodes under `install/`. Dependencies stay in `.pixi/envs/ros1`.
The shell loads both automatically; no system ROS installation is needed.

`sources.repos` pins the UR driver (2.1.5), client library (1.4.0), and supporting
packages. The build fetches them into `src/vendor` and applies their C++17 patch.
`pixi.lock` pins the compiler, ROS, and Python dependencies.

Franka is disabled with `TESTBED_WITH_FRANKA = "0"` in `pixi.toml`. To build its
adapter too, provide `franka_hw` and libfranka, change that value to `"1"`, and
rebuild.

To launch a configured UR robot from the Pixi shell, set its address explicitly
and use its matching calibration:

```bash
roslaunch rcm_controller ur5e_twist_controller.launch \
  robot_ip:="<robot-ip>" kinematics_config:="<calibration.yaml>"
```

The model pose setter uses Sensorium for image processing and calibration.
Pixi installs it from the sibling `../tso_sensorium` checkout.

See [COMMANDS.md](src/robots_controllers/COMMANDS.md) for the local run sequence
and memory measurement.
