# Testbed interfaces

The shared ROS 2 Jazzy interfaces build with ament and rosidl.

`TargetTipPose` carries an incremental displacement in the robot base frame,
an incremental tool roll in radians, and a movement duration in seconds. Stamp
the command at acquisition, set `header.frame_id` to the destination robot base,
and identify the producer with `source_id` and a strictly increasing `sequence`.
Sequence zero is reserved for legacy producers using timestamp ordering.
The controller validates freshness and replaces the active target with an
accepted command; the message does not request queueing.

`RobotState.relative_pivot_rpy` uses the ROS2 lowercase field name. Numeric
recording should select scalar fields instead of stringifying ROS messages.

`KoalaCommand` in `msg` is the stamped boolean websocket gripper command/state.
The separate `KoalaCommand` service and EtherCAT messages retain the motor-control
interface. Import the appropriate `msg` or `srv` namespace explicitly.
