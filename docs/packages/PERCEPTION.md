# Perception: `openarm_sorting_vision`

Finds the red and the blue 4 cm cube in the simulated RGB-D camera and publishes their 3D
pose in the `world` frame. **Classical OpenCV colour segmentation + depth**, no neural
network, no YOLO. The cup is **not** detected; its pose comes from
`openarm_sorting_manipulation/config/cup_sorting_scene.yaml`.

Diagram: [../architecture/diagrams/perception_pipeline.md](../architecture/diagrams/perception_pipeline.md)

## Node `cube_detector`

| | |
|---|---|
| Source | `openarm_sorting_vision/cube_detector.py` (+ `color_segmentation.py`, `camera_geometry.py`) |
| Start | `ros2 launch openarm_sorting_bringup perception.launch.py` (part of `cup_sorting.launch.py`) |
| Parameters | `config/cube_detector.yaml` |
| Subscribes | `/openarm_zed/image` (rgb8), `/openarm_zed/depth_image` (32FC1, m), `/openarm_zed/camera_info`, TF |
| Publishes | `/cup_sorting/{red,blue}_cube/pose` (PoseStamped, world, **stable only**), `/cup_sorting/{red,blue}_cube/raw` (PointStamped, every frame), `/cup_sorting/debug_image`, `/cup_sorting/detection_markers` |

### Camera

Gazebo `rgbd_camera`, 640x480, HFOV 90 deg -> fx = fy = 320, cx = 320, cy = 240, no
distortion, 15 Hz. Depth is registered to RGB and is **z-depth** (distance along the optical
axis), not the ray length; this was verified numerically (a top-row pixel measured 1.461 m
against 1.463 m predicted for z-depth, ray length would be 1.815 m).

### Pipeline (per synchronized RGB + depth pair, `ApproximateTimeSynchronizer`, slop 0.03 s)

1. **HSV segmentation** (`color_mask`): red H in [0, 8] or [172, 179], blue H in [100, 128],
   both S >= 120 and V >= 50, then 3x3 morphological open and close. The table (S ~ 50), cup
   (S < 30), floor and robot (S ~ 0) never pass.
2. **Blob filter** (`find_blobs`): connected components with 60-20000 px and a fill ratio
   >= 0.55 against their minimum-area rectangle; the largest one is used.
3. **Depth**: erode the blob by 2 px (drops edge pixels mixing cube and background depth),
   keep finite depths in 0.15-2.0 m, require >= 25 valid pixels.
4. **Back-projection** (`camera_geometry.back_project`) into the optical frame:
   `x = (u - cx) Z / fx`, `y = (v - cy) Z / fy`, `z = Z`.
5. **TF**: `world <- openarm_zed_sim_camera_optical_frame` (from robot_state_publisher).
6. **Cube centre**: top-face points = within 6 mm (`top_band`) of the 95th-percentile height.
   Centre = centre of their minimum-area rectangle, z = top - cube_size / 2, yaw from the
   rectangle folded to (-45, 45] deg (cube symmetry). Rejected if the top face is larger than
   cube_size + 12 mm or smaller than 45 % of cube_size.
7. **Stability**: publish only when the last 6 estimates (within 1.5 s) all lie within 4 mm of
   their median; the published pose is the median centre and the circular median of the yaw.
   Missing or rejected frames age the history out, so a stale pose is never republished.

### Accuracy

| Measurement | Result |
|---|---|
| Original project, `detection_validator` vs Gazebo ground truth | red 1.3 mm, blue 1.7 mm (mean = max) |
| New workspace, first stable red pose vs spawn pose (0.30, 0.04, 0.32) | (0.2988, 0.0398, 0.3204) = 1.3 mm |

### Parameters (`config/cube_detector.yaml`)

`target_frame`, `optical_frame`, `image_topic`, `depth_topic`, `camera_info_topic`,
`output_namespace`, `colors`, `cube_size`, `hsv.<colour>.{hue_ranges, s_min, v_min}`,
`min_area_px`, `max_area_px`, `min_fill_ratio`, `erode_px`, `min_depth`, `max_depth`,
`min_valid_depth_px`, `top_band`, `size_tolerance`, `stability_window`,
`stability_window_sec`, `stability_tolerance`. All defaults equal the validated values; the
topic names and HSV thresholds were hard-coded in the original and are now parameters with
the same defaults.

## Test tools (Gazebo ground truth, never used by the robot logic)

`ground_truth.py` reads the named model poses straight from Gazebo Transport
(`/world/openarm_cup_sorting/dynamic_pose/info`), because ros_gz_bridge's Pose_V -> TFMessage
conversion drops the entity names.

```bash
# detected vs true cube poses for 10 s (PASS if every error < 10 mm)
ros2 run openarm_sorting_vision detection_validator
ros2 run openarm_sorting_vision detection_validator --ros-args -p duration:=30.0 -p max_error:=0.005
```

## Watching it

```bash
ros2 topic echo /cup_sorting/red_cube/pose
ros2 launch openarm_sorting_bringup image_view.launch.py image:=/cup_sorting/debug_image   # contours + "STABLE" labels
ros2 launch openarm_sorting_bringup rviz.launch.py rviz_config:=camera_view.rviz
```
