# Manipulation: `openarm_sorting_manipulation`

Picks each cube with the **left arm** and **left gripper** and drops it into the single
coffee cup; red first, then blue. Uses MoveIt 2 only through `move_group`'s standard ROS
interfaces (no moveit_py).

Diagram: [../architecture/diagrams/manipulation_sequence.md](../architecture/diagrams/manipulation_sequence.md)

## Nodes

| Executable | Node name | Purpose |
|---|---|---|
| `cup_sorting_coordinator` | `cup_sorting_coordinator` | task services, sequence, verification, recovery |
| `task_prompt` | `cup_sorting_task_prompt` | waits until ready, asks y/N, calls `/cup_sorting/run` |
| `check_cup_contents` | (no ROS node) | test tool: are both cubes inside the cup (Gazebo ground truth) |

## Modules

| Module | Responsibility |
|---|---|
| `coordinator.py` | `CupSortingCoordinator`: parameters, inputs (joint states, joint limits, detections), readiness, services, planning scene, `pick()`, `place_in_cup()`, `safe_recover()` and the stage tests |
| `moveit_interface.py` | `MoveItInterface`: `/compute_ik` (collision-aware, 12 attempts: current state + random seeds, keeps the solution with the largest normalised joint-limit margin), `/move_action` (OMPL, plan only, retried up to 5x), `/compute_cartesian_path` (max step 5 mm, fraction >= 0.98), `/execute_trajectory`, `/apply_planning_scene`; `grasp_pose()` |
| `gripper.py` | `Gripper`: `GripperCommand` action client + measured `openarm_left_finger_joint1` angle; `command()` (waits for the result) and `squeeze()` (does not, see below) |
| `planning_scene.py` | box / cylinder / remove / attach / detach / diff-scene builders |
| `ros_utils.py` | `wait_future`, `call_service`, `run_action` for blocking calls from the worker thread |
| `task_prompt.py` | operator confirmation (see [../execution/TASK_EXECUTION.md](../execution/TASK_EXECUTION.md)) |

The coordinator spins in a `MultiThreadedExecutor` (4 threads) with a re-entrant callback
group; service callbacks run the sequence and block on futures without deadlocking.

## Grasp geometry

* **TCP** = fingertip-pad centre, `tcp_offset` = 0.160 m along `openarm_left_ee_base_link` -z.
* `grasp_pose(tcp, yaw, pitch, tcp_offset)` = orientation `Rz(yaw) * Ry(-pitch)`: at yaw =
  pitch = 0 the fingers point straight down and close along world y; `pitch` (60 deg) tilts
  them forward; the closing axis stays horizontal so the pads stay parallel to the cube faces.
* Grasp yaw candidates: the cube yaw and +-90 deg, wrapped, only those within +-50 deg,
  sorted by magnitude.
* Pad gap vs finger angle (from the meshes): 0 at 0 rad, 3.5 cm at 0.19, 4 cm at ~0.22,
  8.8 cm at 0.45 rad.

## Sequence and verification

See [../architecture/EXECUTION_FLOW.md](../architecture/EXECUTION_FLOW.md) phase 3 for the
step table. The verification primitives are:

| Check | Where | Meaning |
|---|---|---|
| `execute_verified` | every arm motion | after execution, max joint error to the planned end point < `tracking_tolerance` (0.03 rad) within `settle_timeout` (3 s); otherwise "possible collision or blocked joint" |
| contact check | Cartesian descent | finger angle must not change by more than 0.05 rad |
| grasp check | after squeeze and after lift / transport | finger angle in [`grasp_min_position`, `grasp_max_position`] = [0.12, 0.32] rad; an empty gripper closes to ~0 |
| open check | before grasp, at release | finger angle >= commanded - 0.05 rad |
| placement check | end of each cube | fresh camera detection inside the cup radius / height, or the cube no longer visible on the table (occluded by the rim) |

**Squeeze without waiting.** With position control and a cube between the fingers the goal
(0.0 rad) is never reached; the controller keeps commanding it, which is the grip force. The
coordinator sends the goal, waits 3 s for the fingers to settle and verifies the angle.

**Recovery** (`safe_recover`, after any `StepFailed` or exception): re-apply the scene, move
the hand straight up 6 cm if a Cartesian path exists (fraction >= 0.5), then plan to the ready
pose. A held cube is not dropped on purpose.

## Planning scene

`base_scene()`: `table` box; `table_edge_guard` 6 cm x table width x 4 cm over the front edge;
`cup` solid cylinder (outer radius + 5 mm); `red_cube` / `blue_cube` boxes at their detected
poses (unless already in the cup). The target cube is removed for the final descent and then
attached to `openarm_left_ee_base_link` (5 mm smaller, touch links: hand and both fingers)
while it is carried; it is detached and removed after the release.

## Services (`std_srvs/srv/Trigger`)

| Service | Moves the robot | Purpose |
|---|---|---|
| `/cup_sorting/run` | yes | red, then blue (the task) |
| `/cup_sorting/pick_place_red`, `/cup_sorting/pick_place_blue` | yes | one cube |
| `/cup_sorting/ready` | **no** | status: success when ready and idle, otherwise what is missing / "busy" |
| `/cup_sorting/plan_check` | **no** | IK + OMPL plan to the red pre-grasp, not executed |
| `/cup_sorting/go_ready` | yes | ready pose above the table |
| `/cup_sorting/go_home` | yes | SRDF `home` (arm hanging); not used by the sorter |
| `/cup_sorting/approach_test_red` | yes, touches nothing | ready -> pre-grasp -> free-space gripper cycle -> ready |
| `/cup_sorting/grasp_test_red`, `/cup_sorting/grasp_test_blue` | yes | grasp + lift (verified), put the cube back, release |

All motion services are serialised (a second call while one runs returns "busy") and first
wait up to 5 s for the coordinator's readiness.

## Parameters

`config/coordinator.yaml` (motion, gripper, verification; all defaults = validated values):
`planning_frame`, `auto_start`, `detection_namespace`, `tcp_offset`, `grasp_pitch_deg`,
`grasp_z_offset`, `approach_height`, `lift_height`, `release_clearance`, `open_position`,
`release_position`, `close_position`, `grasp_min_position`, `grasp_max_position`,
`gripper_max_effort`, `velocity_scaling`, `tracking_tolerance`, `settle_timeout`,
`finger_contact_tolerance`, `ready_tcp_candidates`, `required_controllers`.

`config/cup_sorting_scene.yaml` (must match the Gazebo world): `table.center`, `table.size`,
`cup.base`, `cup.inner_radius`, `cup.outer_radius`, `cup.height`, `cup.floor_thickness`,
`cup.release_offset_red`, `cup.release_offset_blue`, `cube_size`.

`ready_tcp_candidates`, `gripper_max_effort`, `required_controllers` and
`detection_namespace` were hard-coded in the original and are now parameters with the same
values. Two code defaults that differed from the YAML actually loaded at run time
(`cup.base` [0.28, 0.28, 0.30] and release offsets +-0.018) now equal the YAML values
([0.36, 0.32, 0.30], +-0.02); since the YAML was always loaded this does not change behaviour
(validated: all 27 original parameters identical at run time).

`task_prompt` parameters: `ready_timeout` (300 s), `task_timeout` (900 s),
`detection_max_age` (2 s), `run_service`, `ready_service`, `detection_namespace`.

## Test tool

```bash
ros2 run openarm_sorting_manipulation check_cup_contents                 # exit 0 = both inside
ros2 run openarm_sorting_manipulation check_cup_contents --scene-file my_scene.yaml
```
It reads the cup geometry from the same scene file as the coordinator (in the original it
had its own copy of the numbers).
