# Execution flow

What happens from `ros2 launch openarm_sorting_bringup cup_sorting.launch.py` until the cubes
are in the cup. Timings are from the validation runs on the test machine (headless and GUI).

Diagrams:
[diagrams/launch_sequence.md](diagrams/launch_sequence.md) ·
[diagrams/task_execution_flow.md](diagrams/task_execution_flow.md) ·
[diagrams/manipulation_sequence.md](diagrams/manipulation_sequence.md)

## Phase 1: startup (about 15-18 s)

1. Pre-flight: no other Gazebo server may be running.
2. Gazebo starts the world running (`-r`); robot_state_publisher publishes the Gazebo variant
   of the description (log: `[robot] Gazebo gripper collision patch: ... -> 5 boxes`);
   the bridge forwards `/clock` and the four camera topics.
3. `ros_gz_sim create` spawns `openarm_v20`; on success the spawner loads and activates
   `joint_state_broadcaster`, `left/right_joint_trajectory_controller`,
   `left/right_gripper_controller` (about 7 s after launch).
4. `wait_for_controllers` exits as soon as all 5 are active; then move_group, RViz,
   cube_detector, the coordinator and task_prompt start.
5. cube_detector publishes stable poses after 6 consistent frames (~0.4 s at 15 Hz).
6. The coordinator loads the joint limits from `/robot_description`; `/cup_sorting/ready`
   turns true when move_group, controllers, gripper and limits are available.

## Phase 2: operator confirmation

```text
[task_prompt] waiting until the system is ready (MoveIt, controllers, gripper, coordinator, cube detections)...
[task_prompt] READY after 1 s: coordinator ready (MoveIt, controllers, left gripper, joint limits), red + blue cube detected

========================================
 OpenArm Cup Sorting System
========================================

All required components are ready.

Do you want to execute the sorting task? [y/N]:
```

| Input | Effect |
|---|---|
| `y` or `Y` | `/cup_sorting/run` is called; progress is logged by the coordinator; result printed |
| `n`, `N` or Enter | "The sorting task was NOT executed. The simulation keeps running." + the command to run it later |
| anything else | "Please answer 'y' (execute) or 'n' (do not execute)." and the question again |
| end of input / no terminal | treated as "no": nothing is executed |

Nothing moves before an explicit `y` (validated: arm joints and cube ground-truth poses
unchanged after "n").

## Phase 3: task (about 48-53 s)

`/cup_sorting/run` -> `sort(['red', 'blue'])`: for each colour `pick()` then `place_in_cup()`.

| Step | Action | Verification |
|---|---|---|
| 0 | fresh stable detection of the cube | detector stability gate |
| 1 | planning scene: table, front-edge keep-out, cup cylinder (+5 mm), both cubes at detected poses | - |
| 1b | ready pose above the table (TCP (0.20, 0.28, 0.52), two fallbacks) | joint error < 0.03 rad |
| 2 | open gripper to 0.45 rad | measured finger angle |
| 3 | pre-grasp 10 cm above the cube, 60 deg pitch, closing axis along a cube face | IK + OMPL + joint error |
| 4 | Cartesian descent to TCP = cube centre + 13 mm (target cube removed from the scene) | joint error, finger angle unchanged (contact check) |
| 5 | squeeze (command 0.0 rad), settle 3 s | finger stops in [0.12, 0.32] rad = cube held |
| 6 | attach cube in the scene, Cartesian lift 12 cm | finger still in range |
| 7 | above the cup (+6 cm), release offset red (-2 cm, 0) / blue (+2 cm, 0) | IK + OMPL + joint error, still holding |
| 8 | Cartesian lowering until the cube bottom is 15 mm above the rim | joint error |
| 9 | open to 0.35 rad, detach | finger opened |
| 10 | retreat 8 cm, back to the ready pose | joint error |
| 11 | camera check | cube seen inside the cup, or no longer visible on the table |

Any failed check raises `StepFailed`: the sequence aborts and `safe_recover()` lifts the hand
6 cm (if possible) and plans back to the ready pose. A held cube is deliberately not dropped.

## Phase 4: after the task

The prompt prints `TASK FINISHED SUCCESSFULLY.` (or `TASK DID NOT COMPLETE ...`) and
`Press Ctrl+C to stop the simulation.` and exits. The simulation, MoveIt, perception and the
coordinator keep running: steps can be repeated or inspected with the services listed in
[../execution/COMMAND_REFERENCE.md](../execution/COMMAND_REFERENCE.md).

Ctrl+C stops every process of the run (validated: no Gazebo or ROS process left).
