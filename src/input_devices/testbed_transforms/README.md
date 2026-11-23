# Camera frames and recording

Robot state publishers supply each robot's kinematic TF tree, including
`robot2/base_link → robot2/tool0 → robot2/tool_tip`. The endoscope is carried
by robot2. `camera_mount` adds the fixed calibrated rotation from its modeled
`tool_tip` to `endoscope/camera_optical_frame`. Capture uses that same `frame_id`.

The camera reference origin is defined at the modeled endoscope tip. It is not a
measured optical center. The existing calibration provides rotation only; this
node leaves the tip-to-camera translation zero by definition. Camera-relative
motion and dataset transforms use rotation, not optical-center translation.
No separate stereo origins or baseline are inferred from the shared capture header.

From the workspace root, load the shipped example for the illustrated mount.
Supply a calibration for your own endoscope mount when its geometry differs.

```bash
ros2 run testbed_transforms camera_mount --ros-args -r __ns:=/robot2 \
  -p calibration_path:=src/input_devices/testbed_transforms/config/endoscope_mount.example.yaml
```

`parent_frame` defaults to `<namespace>/tool_tip` (namespace defaults to robot2);
`camera_frame` defaults to `endoscope/camera_optical_frame`. The rotation is the
existing calibration's inverse axis mapping followed by its optical x tilt,
without changes to the shared calibration math. The modeled tip shares tool0's
orientation. Calibration must describe this tool frame and camera reference.
Run only one mount publisher per camera.

To resolve the camera from robot1, supply the calibrated interbase relationship
using `tf2_ros static_transform_publisher`. The UR and mock descriptions already
publish identity `robot2/world → robot2/base_link`, so attach the calibration as
`robot1/base_link → robot2/world` to retain one parent per TF frame. Its translation
is expressed in robot1's base frame, in metres. Aligned bases use an explicit
identity rotation; differently aligned bases also need calibrated rotation.
No identity relationship is assumed between robots.

For Sensorium, start a recording bridge for each robot being recorded:

```bash
ros2 run testbed_transforms camera_transform --ros-args -r __ns:=/robot2
```

The bridge defaults to namespace robot1, reads relative `robot_state`, and writes
relative `camera_transform`, using `target_frame=<namespace>/base_link` and
`camera_frame=endoscope/camera_optical_frame`. Both frames and both topic
parameters (`robot_state_topic`, `transform_topic`) are configurable. RobotState's
frame must match `target_frame`. The quaternion maps camera vectors into that
robot's base; its inverse maps base vectors into camera coordinates. Translation
is resolved from the complete TF chain to the modeled camera reference.

Lookups use the exact source acquisition timestamp, with no latest-TF fallback.
A nonblocking queue tolerates TF arriving after RobotState. `rate_hz` defaults to
30, `max_age_seconds` to 0.25, and `pending_limit` to 32. Expired, future, repeated,
out-of-order, and mismatched-frame source stamps are discarded. Missing TF is
retried only while its sample remains fresh; queue size stays bounded. Published
samples keep the acquisition stamp, including entirely static frame paths.

The Pixi environment supplies Sensorium's ROS-independent mount rotation math.
Only these adapters import TF/ROS.
