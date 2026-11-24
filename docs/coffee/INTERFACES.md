# Coffee task – ROS 2 interfaces, frames and parameters

All names below are the defaults of the OpenArm/Gazebo configuration
(`src/openarm_sorting_bringup/config/coffee/*.yaml`). Every topic / frame / service name used
by the generic nodes is a parameter (or a private `~/` name that follows the node name), so
another robot or camera is connected by configuration or remapping only.

## Nodes and topics

### `aruco_detector` (visual_manipulation_perception)
| | Name (default config) | Type |
|---|---|---|
| sub | `image_topic` = `/openarm_zed/image` | `sensor_msgs/msg/Image` (rgb8 / bgr8 / mono8) |
| sub | `camera_info_topic` = `/openarm_zed/camera_info` | `sensor_msgs/msg/CameraInfo` (K, D) |
| sub (optional) | `depth_topic` = `/openarm_zed/depth_image` (`""` = RGB only) | `sensor_msgs/msg/Image` 32FC1 [m] or 16UC1 [mm], registered to the image |
| pub | `/aruco_detector/detections` | `visual_manipulation_interfaces/msg/FiducialDetectionArray` |
| pub | `/aruco_detector/debug_image` | `sensor_msgs/msg/Image` |
| TF | `<camera_frame> -> aruco_<id>` | |

Parameters: `aruco_dictionary`, `marker_ids` + `marker_sizes` (black-square edge, m),
`default_marker_size`, `camera_frame` (optical frame; `""` = image header frame),
`corner_refinement` (`apriltag` default | `subpix` | `contour` | `none`), `use_depth`,
`depth.{shrink,min_points,max_plane_rms}`, `min_side_px`, `max_reprojection_error`,
`min_confidence`, `publish_tf`, `tf_prefix`, `publish_debug_image`.

`FiducialDetection` fields: `marker_id`, `dictionary`, `marker_size`, `pose` (PoseStamped in
the camera optical frame), `confidence` (0..1), `reprojection_error` [px], `ambiguity_ratio`
(IPPE flip ambiguity), `side_length_px`, `view_angle` [rad], `depth_refined`, `corners[8]`.

### `fiducial_object_tracker` (visual_manipulation_perception)
| | Name | Type |
|---|---|---|
| sub | `detections_topic` = `/aruco_detector/detections` | `FiducialDetectionArray` |
| pub | `/fiducial_object_tracker/objects` | `visual_manipulation_interfaces/msg/TrackedObjectArray` (every frame; `stable` flag) |
| pub | `/fiducial_object_tracker/<name>/pose` | `geometry_msgs/msg/PoseStamped` (only while stable) |
| srv | `/fiducial_object_tracker/reset` | `std_srvs/srv/Trigger` (forget samples; used for "fresh" measurements after a motion) |
| TF | `<reference_frame> -> <name>` (`world -> portafilter`, `world -> machine_mount`) | only while stable |

Parameters: `reference_frame`, `tracked_objects`, `objects.<name>.marker_id`,
`objects.<name>.marker_to_object` [x y z roll pitch yaw], `filter.{window,max_age,
position_tolerance,angle_tolerance}`, `min_confidence`, `require_depth_refined`,
`publish_tf`, `tf_timeout`.

### `portafilter_task` (visual_manipulation_tasks)
| | Name | Type |
|---|---|---|
| srv | `/portafilter_task/run` | `std_srvs/srv/Trigger` – install, verify, remove, put back |
| srv | `/portafilter_task/install` | install + verify, the portafilter stays locked in the machine |
| srv | `/portafilter_task/remove` | locked portafilter → unlock, remove, place at `release.place_pose` |
| srv | `/portafilter_task/detect` | perception only (poses + bayonet state) |
| srv | `/portafilter_task/plan_check` | perception + grasp + IK of every key pose, **no motion** |
| srv | `/portafilter_task/go_ready`, `/ready`, `/cancel` | |
| pub | `/portafilter_task/status` | `visual_manipulation_interfaces/msg/TaskStatus` (transient local) |
| sub | `perception.objects_topic` | `TrackedObjectArray` |
| sub | `/joint_states`, `/robot_description` (limits) | |
| client | `/compute_ik`, `/compute_cartesian_path`, `/apply_planning_scene`, `/get_planning_scene`, `/move_action`, `/execute_trajectory` (prefix `arm.moveit_namespace`) | MoveIt 2 |
| client | `gripper.action` = `/left_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |
| client | `/controller_manager/list_controllers` | readiness only (`required_controllers`, may be empty) |
| TF out | `world -> task/{mount,portafilter,locked_goal,pre_insert,staging,grasp_tcp,pregrasp_tcp,place_goal}` | debug goals for RViz |

Parameters (see the commented `task.yaml`): `planning_frame`; `arm.{group,tip_link,joints,
hand_links,flange_to_tcp,moveit_namespace}`; `motion.{velocity_scaling,
insertion_velocity_scaling,holding_velocity_scaling,tracking_tolerance,settle_timeout,
ik_attempts,cartesian_max_step,ready_tcp_poses}`; `gripper.{action,joint,open_position,
release_position,close_position,grasp_min_position,grasp_max_position,max_effort,settle_time,
contact_tolerance}`; `perception.{objects_topic,reset_service,timeout,in_hand_tolerance}`;
`grasp.candidates`, `grasp.<name>.object_to_tcp`, `grasp.{approach,retreat}.{direction,
distance,frame}`; `install.{staging_offset,rotation_step_deg}`; `servo.{enabled,gain,
max_iterations,position_tolerance,axial_tolerance,angle_tolerance_deg,tilt_tolerance_deg,
accept_factor,coarse_factor,insert_axial_tolerance,correct_tilt,max_translation}`;
`verify.{radial_tolerance,axial_tolerance,angle_tolerance_deg,release_test,place_tolerance}`;
`release.{clearance,place_pose}`; plus the object geometry from `coffee_geometry.yaml`
(`objects.<name>.shape_names / shapes.*`, `bayonet.*`, `workcell.*`).

### `task_prompt` (visual_manipulation_tasks)
Waits for `ready_service`, asks `question` on the terminal, calls `run_service` only after `y`.

### Gazebo layer (visual_manipulation_gazebo, simulation only)
* `grasp_assist` – subscribes `/joint_states`, TF; Gazebo topics `/grasp_assist/{attach,detach}`
  (DetachableJoint), `/world/<world>/dynamic_pose/info`; publishes `~/state`.
* `check_portafilter` – CLI: ground-truth pose of the portafilter vs. the locked pose / dock,
  optional comparison with the perception estimates; `--expect locked|dock` sets the exit code.

## TF frames

```text
world ─┬─ openarm_body_link0 ─ ... ─ openarm_zed_sim_camera_link ─ openarm_zed_sim_camera_optical_frame ─ aruco_7, aruco_21
       │                    └─ ... ─ openarm_left_ee_base_link (tip link; TCP = 16 cm along -z)
       ├─ portafilter         (tracker, measured)        frame P: basket axis, rim plane, +x handle
       ├─ machine_mount       (tracker, measured)        frame M: group-head axis, bottom plane, +z insertion
       └─ task/*              (task node, goals)
```

Required from the robot side: TF from `planning_frame` to the camera optical frame (fixed or
moving camera) and to the tip link, i.e. a hand-eye calibration on a real robot.

## Custom messages (visual_manipulation_interfaces)

| Message | Why not a standard type |
|---|---|
| `FiducialDetection(Array)` | id + pose + quality indicators (confidence, reprojection error, ambiguity, size, view angle, depth flag) in one atomic message |
| `TrackedObject(Array)` | object pose with stability / spread information for gating motions |
| `TaskStatus` | state, previous state, attempt, detail for monitoring |
