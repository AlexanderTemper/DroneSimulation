Header header
# Message should be expressed in the frame_id specified in the header.
geometry_msgs/Vector3 gyro      # [Grad*10/s]
geometry_msgs/Vector3 pid
geometry_msgs/Vector3 att
int32[] RateError
int32[] PTerm
int32[] ITerm
int32[] DTerm
int32[] AngleRateTmp
