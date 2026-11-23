# Testbed input and policy adapters

`testbed_input_core` is a standalone Python package. Its gamepad/pedal decoders,
immutable snapshots, and motion mapping need no ROS. Linux acquisition lives in
`linux.py` (optional `evdev` dependency); policy preparation uses Sensorium's
calibration/rectification and the existing robotics socket protocol.

Install the core with `pip install -e .` or use the testbed workspace's Pixi
`core`/`ros2` environments. Build the ROS 2 Jazzy packages with the workspace
build task.

- `ps4_controller`, `pedal_board`, `foot_switch`: device acquisition adapters.
- `input_translation`: PS4/Lambda teleoperation, policy RPC, language publishing,
  and optional WebSocket gripper delivery.
- `lambda`: optional Force Dimension SDK driver, including force feedback.
- `testbed_transforms`: calibrated camera mount TF and recording transforms.

The command contract is `TargetTipPose.target_delta_position` in robot-base
metres and `target_delta_roll` in radians. Each command has a timestamp, source
identifier and increasing sequence. Teleoperation scales each input into one
delta; it does not accumulate an absolute target. The controller replaces the
remaining trajectory when a new command arrives, so `movement_time` describes
the planning horizon, not a promise to sum every input event's displacement.
Policy actions use physical units; absolute targets are converted to deltas
against measured state exactly once. Camera-frame absolute targets are compared
with measured state in that same frame before conversion into the robot base.

Default topics are `/robot1/robot_state`, `/robot1/set_tip_pose`,
`/robot1/ps4_controller/state`, `/robot1/pedal_board/state`, and
`/robot1/foot_switch/state`. Device topics are namespace-relative, so separate
robot namespaces do not share pad inputs. Input adapters reject stale, replayed,
future-dated or nonfinite state; teleoperation retains the acquisition timestamp.
`RobotState.header.frame_id` must equal `robot_base_frame`, which defaults to
`<namespace>/base_link` (for example `robot1/base_link`). Lambda requires the
configured `top_grey_pedal` clutch by default.

Camera-coordinate consumers resolve TF from `camera_frame` (default
`endoscope/camera_optical_frame`) into `robot_base_frame` at the robot state's
acquisition timestamp. A bounded history lets the consumer select the newest
fresh state for which that exact-time transform has arrived. State and rotation
always come from the same timestamp; callbacks never wait for TF or substitute a
latest-time transform. A missing path, extrapolation or stale state prevents the
camera-coordinate command and reports the requested frames. Base-coordinate
commands do not need a camera transform.

The endoscope is carried by robot2. Its TF chain runs through
`robot2/base_link`, `robot2/tool0`, the modeled `robot2/tool_tip`, and the
calibrated camera orientation. A calibrated link between the robot bases lets
robot1 use that moving camera too; no identity link between bases is assumed. Policy position
observations/actions remain displacements from the controller's activation pose,
expressed in the selected frame's axes; these vectors are rotated, rather than
translated as points relative to the optical origin. Override the robot and
camera frames independently. For the endoscope guidance policy, select robot2:

```bash
ros2 launch input_translation model_pose_setter.launch.py \
    namespace:=robot2 robot_base_frame:=robot2/base_link \
    camera_frame:=endoscope/camera_optical_frame
```

After activating the workspace ROS 2 environment:

```bash
ros2 launch ps4_controller ps4.launch.py
ros2 launch pedal_board pedal_board.launch.py
ros2 launch foot_switch foot_switch.launch.py
ros2 launch input_translation model_pose_setter.launch.py
ros2 run input_translation language_publisher
```

The policy RPC defaults to `127.0.0.1:5550`, uses `/endoscope/left/image_raw`
and `/endoscope/right/image_raw`, and preserves the
`get_observation`/`send_action` payloads.
Combined interlaced input remains available with `use_interlaced_image:=true`;
`combined_width`/`combined_height` resize each eye to 960×540 by default, matching
the capture publisher. Rectification rejects images whose dimensions differ
from the calibration maps.
Rectification, crops, image reliability and the depth server are configurable.
Stereo samples must be fresh and their timestamps must
match within `maximum_stereo_skew`. Depth/point-cloud requests require the
external stereo service. Camera transforms map camera vectors into robot-base
coordinates; quaternions use xyzw order.

The default gripper transport uses the existing Koala command service. Set
`gripper_transport:=topic` and run `ros2 run input_translation gripper_server`
for the ROS 2 WebSocket server.
The ROS-free transport sends direct commands (`1` opens, `0` closes), retains
the latest request and replays it when the browser reconnects. The initial
requested state is CLOSE. `/koala_grasper/state` publishes the latest requested
command at 10 Hz, including while disconnected; it is neither a browser delivery
acknowledgment nor measured physical feedback.

The matching browser script is included as
`translation_node/web/koala_client.js` and installed under
`share/input_translation/web/koala_client.js`. Refresh the Koala HMI page
before pasting it once into the browser console, so an older client does not
remain connected. The original script retains its 100 ms button press, 800 ms
delay before an opposite command, and 100 ms settling pause.

Build Lambda only on hosts with the Force Dimension SDK. Pass
`-DFORCE_DIMENSION_SDK_ROOT=/path/to/sdk` (or explicit include/library CMake
variables for a nonstandard layout). The default workspace build skips it.
Force commands time out to zero after `force_timeout` seconds; shutdown joins
the device thread and closes the SDK device. Launch with
`ros2 launch lambda lambda.launch.py` after building the optional package.

Select each Lambda explicitly when controlling multiple robots:

```bash
ros2 launch lambda lambda.launch.py namespace:=robot1 hand:=right
ros2 launch lambda lambda.launch.py namespace:=robot2 hand:=left
```

`hand` matches the SDK-reported handedness; it does not mirror the input axes.
Optional `device_index` (SDK enumeration index) and `device_serial` narrow the
selection; `-1` leaves either unspecified. All supplied selectors must agree.
The driver requires exactly one available matching device and reports the selected
index, serial and hand before initializing it. State, force and robot-command
topics remain relative to each namespace. Every SDK device operation uses that
instance's selected handle.

Read a shared pedal board once, then pass its absolute `pedal_board_topic` to
both launches. Their `clutch_pedal` parameters can name the same pedal or different
pedals. Enable gripper output only for the robot carrying the gripper, or give
each gripper its own command topic.
