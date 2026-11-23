std_msgs/Header header
# Displacement in header.frame_id (robot base); roll is in radians.
geometry_msgs/Point target_delta_position
float64 target_delta_roll
float64 movement_time
# Monotonic sequence within a source. Zero enables legacy timestamp ordering.
uint64 sequence
string source_id
