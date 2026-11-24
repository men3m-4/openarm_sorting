# Pose estimate of an object located through a fiducial marker and a configured
# marker -> object transform, expressed in a fixed reference frame (e.g. the robot base).

string name                            # configured object name, e.g. "portafilter"
int32 marker_id
geometry_msgs/PoseStamped pose         # object frame in pose.header.frame_id
float64 confidence                     # 0..1 (latest detection confidence)
bool stable                            # true if the last samples agree within tolerance
int32 num_samples                      # samples in the stability window
float64 position_spread                # max distance of a sample from the filtered position [m]
float64 angular_spread                 # max angle of a sample from the filtered orientation [rad]
builtin_interfaces/Time last_seen
