# Compact pose and velocity message equivalent to nav_msgs/Odometry
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

std_msgs/Header header
string child_frame_id
argus_msgs/CompactPoseWithCovariance pose
argus_msgs/CompactTwistWithCovariance twist
