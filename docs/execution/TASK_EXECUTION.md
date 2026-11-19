# Task execution

The task: the LEFT arm picks the **red** cube and drops it into the coffee cup, then picks
the **blue** cube and drops it into the **same** cup. Diagram:
[../architecture/diagrams/task_execution_flow.md](../architecture/diagrams/task_execution_flow.md)

## 1. With the confirmation prompt (default)

```bash
ros2 launch openarm_sorting_bringup cup_sorting.launch.py
```

Once MoveIt, the controllers, the left gripper, the coordinator and both cube detections are
ready:

```text
[task_prompt] READY after 1 s: coordinator ready (MoveIt, controllers, left gripper, joint limits), red + blue cube detected

========================================
 OpenArm Cup Sorting System
========================================

All required components are ready.

Do you want to execute the sorting task? [y/N]:
```

| Answer | Result |
|---|---|
| `y` or `Y` | `Executing the sorting task (red cube, then blue cube)...`; the coordinator logs each step; at the end `TASK FINISHED SUCCESSFULLY.` or `TASK DID NOT COMPLETE - see the coordinator messages above.` |
| `n`, `N` or Enter | `The sorting task was NOT executed. The simulation keeps running.` + the command to run it later. Nothing moves. |
| anything else | `Please answer 'y' (execute) or 'n' (do not execute).` and the question again |

In every case the simulation keeps running afterwards (`Press Ctrl+C to stop the simulation.`).

The question needs an interactive terminal. If the launch has none (e.g. started from a
script with no terminal), the prompt prints an error and the task is **not** started.

## 2. Without the prompt

```bash
# sort automatically as soon as everything is ready
ros2 launch openarm_sorting_bringup cup_sorting.launch.py auto_start:=true

# or: no question, start the task yourself from another terminal
ros2 launch openarm_sorting_bringup cup_sorting.launch.py prompt:=false
ros2 service call /cup_sorting/ready std_srvs/srv/Trigger     # wait for success=True
ros2 service call /cup_sorting/run std_srvs/srv/Trigger
```

## 3. Progress messages (coordinator)

Real output of a validation run (GUI, "y"), red cube shown; the blue cube follows the same
steps:

```text
[red] 0. detected at (0.299, 0.040, 0.320), yaw 0.0 deg
ready [0.2, 0.28, 0.52]: planned 43 points, 4.2 s -> executing
[red] 2. gripper open, finger 0.450 rad
[red] 3. pre-grasp (yaw 0 deg): planned 27 points, 2.5 s -> executing
[red] 4. descend to grasp: done (fraction 1.00, 1.8 s)
[red] 5. close: finger settled at 0.209 rad
[red] 5. GRASP VERIFIED (finger stopped at 0.209 rad, expected ~0.22 for 4 cm)
[red] 6. lift: done (fraction 1.00, 2.0 s)
[red] 6. lift verified, still holding (finger 0.208 rad)
[red] 7. above cup (yaw 0): planned 20 points, 1.9 s -> executing
[red] 8. lower into cup: done (fraction 1.00, 1.1 s)
[red] 9. released (finger 0.350 rad)
[red] 10. retreat: done (fraction 1.00, 1.3 s)
ready [0.2, 0.28, 0.52]: planned 21 points, 2.0 s -> executing
[red] 10. back at the ready pose
[red] 11. PLACEMENT VERIFIED by camera: seen inside the cup
[blue] 0. detected at (0.299, 0.159, 0.320), yaw 0.0 deg
...
[blue] 11. PLACEMENT VERIFIED by camera: seen inside the cup
```
A full run takes about 50 s after "y".

## 4. Single steps and tests

| Command | Moves the robot? |
|---|---|
| `ros2 service call /cup_sorting/plan_check std_srvs/srv/Trigger` | no (plan only) |
| `ros2 service call /cup_sorting/go_ready std_srvs/srv/Trigger` | yes |
| `ros2 service call /cup_sorting/approach_test_red std_srvs/srv/Trigger` | yes, touches nothing |
| `ros2 service call /cup_sorting/grasp_test_red std_srvs/srv/Trigger` | yes, puts the cube back |
| `ros2 service call /cup_sorting/pick_place_red std_srvs/srv/Trigger` | yes |
| `ros2 service call /cup_sorting/pick_place_blue std_srvs/srv/Trigger` | yes |

## 5. Checking the result independently (Gazebo ground truth)

```bash
ros2 run openarm_sorting_manipulation check_cup_contents
# red_cube: (0.3568, 0.3162, 0.3280)  0.5 cm from cup axis  -> INSIDE the cup (resting on the cup floor)
# blue_cube: (0.3856, 0.3220, 0.3684)  2.6 cm from cup axis  -> INSIDE the cup
```
Exit code 0 when both cubes are inside. The second cube usually rests leaning on the first
one (inside the cup, not flat); this is the known behaviour of the original project too.

## 6. Running the task again

The cubes stay in the cup. To start over, stop the launch (Ctrl+C) and launch again, which
resets the world.
