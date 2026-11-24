# One fiducial (e.g. ArUco) marker detected in one camera image.
#
# pose: marker frame in the frame of pose.header (normally the camera OPTICAL frame:
#       x right, y down, z forward). Marker frame (OpenCV convention): origin at the centre
#       of the black square, x to the right of the printed marker, y up, z out of the face.

int32 marker_id
string dictionary                      # e.g. DICT_4X4_50
float64 marker_size                    # edge of the black square [m] used for the estimate

geometry_msgs/PoseStamped pose

# ---- quality indicators
float64 confidence                     # 0..1, combines the indicators below (see aruco_pose.py)
float64 reprojection_error             # RMS corner reprojection error of the chosen solution [px]
float64 ambiguity_ratio                # reprojection error of best / second planar solution
                                       # (IPPE); close to 1 = flip ambiguity, low = unambiguous
float64 side_length_px                 # mean edge length in the image [px]
float64 view_angle                     # angle between marker normal and viewing ray [rad]
bool depth_refined                     # true if a registered depth image refined the pose
float32[8] corners                     # image corners u0 v0 .. u3 v3 [px], OpenCV order
