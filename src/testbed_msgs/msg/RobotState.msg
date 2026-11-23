std_msgs/Header header
sensor_msgs/JointState joints_state
geometry_msgs/Pose robot_pose
geometry_msgs/Twist end_effector_twist

# Robot space means in world/base frame coordinates.
geometry_msgs/Point robot_space_tip_position
geometry_msgs/Twist robot_space_tip_twist
geometry_msgs/Point robot_space_initial_tip_position
geometry_msgs/Point robot_space_rcm_position

# Relative means w.r.t. the initial pose of the robot (when the controller is started).
geometry_msgs/Point relative_tip_position
geometry_msgs/Point relative_pivot_rpy
float64 relative_pivot_translation

geometry_msgs/Wrench robot_space_wrench