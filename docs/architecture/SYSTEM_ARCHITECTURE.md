# System architecture

The OpenArm cup-sorting system is a Gazebo Harmonic simulation in which the **left arm** of a
bimanual OpenArm v2.0 robot finds a red and a blue 4 cm cube with an RGB-D camera, picks each
one with MoveIt 2 and drops both into **one coffee cup**: red first, then blue. Every step is
verified (camera, joint tracking, finger angle) rather than assumed.

Diagram: [diagrams/system_architecture.md](diagrams/system_architecture.md)

## 1. Components

| Layer | Component | Implementation | Package |
|---|---|---|---|
| Simulation | Gazebo Harmonic world `openarm_cup_sorting` (2 ms step, **bullet-featherstone** physics) | `worlds/openarm_cup_sorting_world.sdf` + 4 models | `openarm_sorting_bringup` |
| Robot model | OpenArm v2.0 bimanual, pinch grippers, ZED Mini mount | upstream `openarm_description` + Gazebo wrapper Xacro + Gazebo preset | `openarm_sorting_description` |
| Robot state | `/robot_description`, `/tf` | `robot_state_publisher` | (system) |
| Control | `/controller_manager` inside Gazebo, 5 controllers | `gz_ros2_control` + `config/gazebo_controllers.yaml` | `openarm_sorting_description` |
| Camera | simulated RGB-D sensor `openarm_zed_rgbd` (640x480, HFOV 90 deg, 15 Hz) bridged to ROS | Xacro `<sensor>` + `ros_gz_bridge` | `openarm_sorting_description`, `openarm_sorting_bringup` |
| Perception | red/blue cube 3D poses | `cube_detector` (OpenCV HSV + depth + TF) | `openarm_sorting_vision` |
| Motion planning | IK, OMPL, Cartesian paths, execution, planning scene | `move_group` with upstream `openarm_bimanual_moveit_config` (v2.0) | upstream + `openarm_sorting_bringup` overrides |
| Task | pick-and-place sequence, verification, recovery | `cup_sorting_coordinator` | `openarm_sorting_manipulation` |
| Operator | readiness wait + y/N confirmation | `task_prompt` | `openarm_sorting_manipulation` |
| Visualisation | MoveIt MotionPlanning, camera, detections | RViz 2 (`rviz/cup_sorting.rviz`) | `openarm_sorting_bringup` |
| Startup | event-driven launch hierarchy | 9 launch files + `wait_for_controllers` gate | `openarm_sorting_bringup` |

Only **one** controller manager exists: the `gz_ros2_control` plugin inside Gazebo. No
`ros2_control_node` is started and the real-hardware `openarm_hardware` plugin is not used
(it is built because it is part of the upstream repository).

## 2. Frames and coordinates

* `world` is the OpenArm root frame and the MoveIt planning frame: +x forward, +y to the
  robot's left (where the LEFT arm is mounted), +z up. `openarm_body_link0` sits at its origin.
* Camera: Gazebo stamps all camera messages with `openarm_zed_sim_camera_link`
  (x forward, y left, z up). The ROS optical frame `openarm_zed_sim_camera_optical_frame`
  (z forward, x right, y down) is a child of it with rpy (-pi/2, 0, -pi/2). Pixel
  back-projection with the CameraInfo K matrix yields coordinates in the optical frame.
* The sensor is placed 3 cm in front of the ZED Mini housing centre: inside the housing every
  pixel hit the housing (flat colour, depth all -inf).

## 3. Scene

| Object | Pose / size (world frame) | Notes |
|---|---|---|
| Table | top surface z = 0.30 m; x 0.12-0.62, y -0.35-0.45 | static, birch laminate (low saturation, never matches a cube colour) |
| Red cube | centre (0.30, 0.04, 0.32), 4 cm, 50 g | friction mu = 1.2 |
| Blue cube | centre (0.30, 0.16, 0.32), 4 cm, 50 g | friction mu = 1.2 |
| Coffee cup | bottom centre (0.36, 0.32, 0.30), handle towards +y | static; inner diameter 11 cm, height 9.5 cm, 8 mm floor; primitive collisions (floor + 24 wall boxes) |

The **cup pose comes from configuration**
(`openarm_sorting_manipulation/config/cup_sorting_scene.yaml`), it is not detected. The
**cube poses are always measured by the camera**; they are not configured anywhere.

Why these positions: an offline IK study of the left arm showed that top-down grasps are
nearly impossible for this arm (no solution at yaw 0, solutions at yaw 90 deg within
0.01-0.1 rad of the wrist limits). A grasp with the fingers **pitched 60 deg forward** has
continuous IK from grasp height to lift height with >= 0.26 rad joint-limit margin at both cube
positions and at the cup release points. The cubes stand side by side in y so the hand, which
trails behind the fingertips at this pitch, never passes over the other cube.

## 4. Simulation-specific engineering decisions

These were found and validated in the original project and are preserved unchanged.

1. **bullet-featherstone instead of DART.** `openarm_*_finger_joint2` is a URDF mimic of
   `finger_joint1` and is deliberately not part of ros2_control. DART (Gazebo's default) cannot
   create mimic constraints, so the second finger never moved. bullet-featherstone enforces the
   mimic (both fingers measured symmetric). Adding finger 2 to ros2_control makes
   gz_ros2_control fight the constraint (fingers lock) and must not be done.
2. **Gazebo-only box collisions for the fingers.** bullet-featherstone collides meshes as their
   convex hull inflated by ~10 mm. The inflated finger hulls pressed on the table and the cube
   before the pads touched it, so the gripper could not close around the cube. At launch time
   each finger's collision mesh is replaced by 5 boxes (one per part of the convex
   decomposition shipped with `openarm_description`, part 0 = fingertip pad). Only the
   description sent to `robot_state_publisher` / Gazebo is patched; MoveIt keeps the exact
   meshes. After the patch the fingers stop on a 4 cm cube at ~0.20-0.22 rad.
3. **Ready pose instead of the SRDF `home`.** In `home` the arm hangs with the hand below the
   table top behind the table edge; an early run wedged the hand on the edge while the
   controller reported SUCCEEDED (the trajectory controllers' goal tolerances are 0). Hence a
   ready pose above the table, a keep-out box over the table's front edge in the planning
   scene and an explicit joint-error check after every motion.
4. **Gripper squeeze without waiting for the action result.** With position control the
   fingers stop on the cube and never reach the commanded 0.0 rad; the controller keeps
   commanding it, which produces the grip force. The coordinator sends the goal, lets the
   fingers settle and verifies the measured finger angle.
5. **MoveIt runtime overrides** (no upstream file changed): KDL timeout 0.05 s instead of
   0.005 s; execution duration scaling 2.0, goal duration margin 1.0 s, start tolerance 0.05.
6. **No ZED SDK.** Only `zed_description` (meshes) is used. The ZED SDK and ZED ROS 2 wrapper
   take images from a physical camera or SVO recordings, not from ROS image topics, so they
   cannot consume the Gazebo sensor. The standard ROS RGB-D pipeline (CameraInfo, registered
   depth, TF) with OpenCV is used instead. **No neural network / YOLO is used.**

## 5. Data flow

```
/openarm_zed/{image, depth_image, camera_info} --> cube_detector --> /cup_sorting/{red,blue}_cube/pose (world)
                                                                 --> /cup_sorting/debug_image, /cup_sorting/detection_markers
/cup_sorting/*_cube/pose --> cup_sorting_coordinator --> move_group (/compute_ik, /move_action, /compute_cartesian_path,
                                                                     /execute_trajectory, /apply_planning_scene)
                                                     --> /left_gripper_controller/gripper_cmd
move_group --> /left_joint_trajectory_controller/follow_joint_trajectory --> gz_ros2_control
task_prompt --> /cup_sorting/ready (status), /cup_sorting/run (after "y")
```

## 6. Interfaces offered by the application

| Interface | Type | Provider |
|---|---|---|
| `/cup_sorting/{red,blue}_cube/pose` | `geometry_msgs/PoseStamped` (world, stable only) | cube_detector |
| `/cup_sorting/{red,blue}_cube/raw` | `geometry_msgs/PointStamped` (per frame) | cube_detector |
| `/cup_sorting/debug_image` | `sensor_msgs/Image` | cube_detector |
| `/cup_sorting/detection_markers` | `visualization_msgs/MarkerArray` | cube_detector |
| `/cup_sorting/run` | `std_srvs/Trigger`: red, then blue | coordinator |
| `/cup_sorting/pick_place_red`, `/cup_sorting/pick_place_blue` | `std_srvs/Trigger` | coordinator |
| `/cup_sorting/ready` | `std_srvs/Trigger`: read-only status | coordinator |
| `/cup_sorting/plan_check` | `std_srvs/Trigger`: plan only, no motion | coordinator |
| `/cup_sorting/go_ready`, `/cup_sorting/go_home` | `std_srvs/Trigger` | coordinator |
| `/cup_sorting/approach_test_red`, `/cup_sorting/grasp_test_red`, `/cup_sorting/grasp_test_blue` | `std_srvs/Trigger`: stage tests | coordinator |

## 7. Related documents

* [PACKAGE_ARCHITECTURE.md](PACKAGE_ARCHITECTURE.md): packages and their responsibilities
* [LAUNCH_ARCHITECTURE.md](LAUNCH_ARCHITECTURE.md): launch hierarchy, sequencing, shutdown
* [EXECUTION_FLOW.md](EXECUTION_FLOW.md): what happens from `ros2 launch` to the end of the task
* [../packages/](../packages/): per-package details
