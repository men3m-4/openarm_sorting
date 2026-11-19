# Validation report

| | |
|---|---|
| Date | 2026-11-19 |
| Machine | Ubuntu 24.04.5, NVIDIA GeForce RTX 3070 Laptop GPU (driver 580.178.04), display available (Wayland, X11 for Gazebo/RViz) |
| Software | ROS 2 Jazzy (desktop 0.11.0), Gazebo Harmonic 8.15.0, MoveIt 2.12.4, ros_gz 1.0.25, gz_ros2_control 1.2.20 |
| Shell | terminal inside the VS Code **snap** (relevant for RViz/rqt, see §5.6) |
| Workspace | `~/openarm_sorting_ws_clean`, built from scratch; only `/opt/ros/jazzy` sourced |

Everything below was executed. Nothing is reported as passed that was not run; limitations
are listed in §9.

## 1. Summary

| Area | Result |
|---|---|
| Package manifests (format-3 schema) | 4/4 project manifests validate |
| Dependencies (`rosdep check`) | all satisfied |
| Build (`colcon build --symlink-install`) | 10/10 packages, 16.8 s, no errors (2 upstream compiler warnings) |
| Package discovery | 9 ament packages found (`openarm_can` is plain CMake); upstream resolves into the new workspace |
| Unit tests (`colcon test`) | **42/42 pass** |
| Launch files | 9/9 load (unit test) and list their arguments (`--show-args`) |
| Static equivalence with the original | robot URDF, controllers, Gazebo preset, scene geometry, 210 move_group parameters and 27 coordinator parameters **identical** |
| Simulation startup | Gazebo, spawn, 5 controllers active (~7 s), MoveIt, perception, coordinator, RViz |
| Readiness checks + y/N prompt | prompt only after readiness; invalid input re-asked |
| "n" path | nothing moves (joint change 0.0000 rad, cubes 0.0 mm from spawn poses); simulation keeps running |
| "y" path (complete task) | **4/4 complete runs succeeded** in the new workspace (+ 1 via the single-cube services); both cubes inside the cup by Gazebo ground truth every time |
| Comparison with the original | same-day run of the original: equivalent timings, grasp angles and final cube positions |
| Failure handling | pre-flight refusal, crash of a required node, Gazebo orphan cleanup: all verified |
| Original workspace | **unchanged** (SHA-256 of all 1042 files identical) |

## 2. Static checks

### 2.1 Manifests and dependencies

```bash
xmllint --noout --schema http://download.ros.org/schema/package_format3.xsd src/openarm_sorting_*/package.xml
# -> 4x "validates"
rosdep check --from-paths src --ignore-src --rosdistro jazzy
# -> All system dependencies have been satisfied
```
The first `rosdep check` failed with `Cannot locate rosdep definition for [ament_python]` in
all four project packages (an error carried over from the original manifests:
`ament_python` is not a rosdep key). Fixed by removing that `buildtool_depend`.

### 2.2 Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/openarm_sorting_ws_clean && colcon build --symlink-install
# Summary: 10 packages finished [16.8s]
#   2 packages had stderr output: openarm_can openarm_hardware   (upstream warnings:
#   unused parameter; deprecated hardware_interface on_init overload)
```

### 2.3 Discovery

```bash
source install/setup.bash
ros2 pkg list | grep openarm        # openarm, openarm_bimanual_moveit_config, openarm_bringup, openarm_description,
                                    # openarm_hardware, openarm_sorting_{bringup,description,manipulation,vision}
ros2 pkg executables openarm_sorting_vision          # cube_detector detection_validator
ros2 pkg executables openarm_sorting_manipulation    # check_cup_contents cup_sorting_coordinator task_prompt
ros2 pkg executables openarm_sorting_bringup         # wait_for_controllers
ros2 pkg prefix openarm_description                  # ~/openarm_sorting_ws_clean/install/openarm_description
```

### 2.4 Unit tests

```bash
colcon test --packages-select openarm_sorting_description openarm_sorting_vision \
    openarm_sorting_manipulation openarm_sorting_bringup && colcon test-result --all
# openarm_sorting_bringup 15, openarm_sorting_description 4, openarm_sorting_manipulation 16,
# openarm_sorting_vision 7  ->  Summary: 42 tests, 0 errors, 0 failures, 0 skipped
```
Covered: Xacro expansion, camera frames, controllers path, finger patch (5 boxes x 4 links),
joint4 limit; HSV segmentation on a synthetic scene, table rejection, back-projection, optical
frame rotation = URDF rpy, quaternion helpers; prompt answer parsing (12 cases), grasp-pose
geometry, planning-scene builders, installed scene file; controller names from YAML, all
launch files load with defaulted arguments, world models installed, world cup pose = scene
file, resource-path de-duplication, NVIDIA mode parsing, pre-flight helper.

### 2.5 Launch files parse

All 9 launch files load (unit test `test_launch_file_loads`) and
`ros2 launch openarm_sorting_bringup <file> --show-args` lists 10 / 4 / 2 / 3 / 2 / 4 / 3 / 5
arguments for cup_sorting / simulation / robot / controllers / moveit / rviz / perception /
coordinator (image_view: 1).

### 2.6 Equivalence with the original configuration

Generated with the original pipeline (original Xacro + original in-upstream preset +
original patch module, original install sourced read-only, bytecode writing disabled) and with
the new pipeline, then compared:

| Item | Result |
|---|---|
| expanded URDF, MoveIt variant | identical except comments and the controllers file path |
| expanded URDF, Gazebo variant (finger patch applied) | identical except the controllers file path |
| controllers YAML, scene YAML, Gazebo preset (parsed) | identical |
| all 210 effective move_group parameters (MoveItConfigsBuilder dict + overrides), robot_description compared separately | identical (after normalising the workspace path in the SRDF's auto-generated comment) |
| all 27 coordinator parameters at run time (`ros2 param dump`) vs original code defaults + original scene YAML | identical; 4 new parameters carry the former constants |
| coffee-cup generator | reproduces the shipped model byte for byte |

## 3. Runtime: startup

Headless smoke run `ros2 launch openarm_sorting_bringup cup_sorting.launch.py gz_gui:=false rviz:=false prompt:=false`:

| Check | Evidence |
|---|---|
| Gazebo starts, world loaded | `[simulation] world: .../openarm_cup_sorting_world.sdf (headless)`; `gz model --list` = blue_cube, coffee_cup, cup_sorting_table, ground_plane, openarm_v20, red_cube |
| finger patch | `[robot] Gazebo gripper collision patch: ... -> 5 boxes` for the 4 finger links |
| robot spawns | `[spawn_openarm]: Entity creation successful.` -> `[simulation] robot spawned -> loading the controllers` |
| controllers active | spawner: 5x `Configured and activated`; `wait_for_controllers: all 5 controllers active after 7 s`; `ros2 control list_controllers` 5x active |
| event sequencing | `[cup_sorting] controllers active -> starting MoveIt, RViz, perception, coordinator` only after the gate exited |
| MoveIt | move_group running; `/cup_sorting/plan_check` -> IK + 52-point OMPL plan |
| perception | `/cup_sorting/red_cube/pose` = (0.2988, 0.0398, 0.3204) for spawn (0.30, 0.04, 0.32); camera 15.0 Hz (14.0 Hz under full load) |
| coordinator | `/cup_sorting/ready` -> `success=True, message='ready'` ~15 s after launch |

`detection_validator` (10 s, Gazebo ground truth): red n=146, mean = max error **1.3 mm**;
blue n=145, **1.7 mm**; yaw error 0.0 deg; `RESULT: PASS`. Identical to the original
project's documented accuracy (1.3 / 1.7 mm).

## 4. Runtime: prompt and task (end-to-end, `tests/integration/e2e_cup_sorting.py`)

The test drives the real terminal through a pseudo-terminal (pexpect), answers `maybe` first
(must be rejected), then `y` or `n`, checks the outcome with ROS topics and Gazebo ground
truth, presses Ctrl+C and checks for leftover processes.

| Run | Mode | Ready -> prompt | Result |
|---|---|---|---|
| 155246 (final code) | headless, answer **n** | 15 s | PASS 11/11: "NOT executed", no sequence, max joint change 0.0000 rad, cubes 0.0 mm from spawn poses, simulation still running, clean Ctrl+C |
| 151856, 152030 | headless, answer **n** | 15-16 s | PASS 11/11 (earlier code revisions) |
| 152213 | headless, answer **y** | 15 s | PASS 14/14: task 53 s, grasps red 0.210 / blue 0.197 rad, both placements camera-verified; ground truth red 1.1 cm from cup axis on the floor, blue 2.5 cm inside |
| 152346 | **Gazebo GUI + RViz**, answer **y** | 18 s | PASS 14/14: task 48 s, grasps 0.209 / 0.196 rad; red 0.5 cm on the floor, blue 2.6 cm inside |
| 155336 (final code) | headless, answer **y** | 18 s | PASS 14/14: task 51 s, grasps 0.198 / 0.195 rad; red 0.9 cm on the floor, blue 2.3 cm inside |

Further modes (driver script, launch started without a controlling terminal):

| Mode | Result |
|---|---|
| `auto_start:=true` | sort started by itself, `auto_start finished: success=True` 76 s after launch, grasps 0.205 / 0.219 rad; ground truth red 2.4 cm on the floor, blue 2.9 cm inside; no prompt started |
| no terminal (stdin `/dev/null`, new session) | `[task_prompt] ERROR: no interactive terminal ... the task will NOT be started`; no sequence started within 15 s; clean shutdown |

Prompt output as captured (final code):

```text
[task_prompt] waiting until the system is ready (MoveIt, controllers, gripper, coordinator, cube detections)...
[task_prompt] READY after 1 s: coordinator ready (MoveIt, controllers, left gripper, joint limits), red + blue cube detected

========================================
 OpenArm Cup Sorting System
========================================

All required components are ready.

Do you want to execute the sorting task? [y/N]: maybe
Please answer 'y' (execute) or 'n' (do not execute).
Do you want to execute the sorting task? [y/N]: n

The sorting task was NOT executed. The simulation keeps running.
Run it later with:  ros2 service call /cup_sorting/run std_srvs/srv/Trigger
Press Ctrl+C to stop the simulation.
```

## 5. Runtime: subsystems, services, tools, failure handling

### 5.1 Individual launch files (`tests/integration/subsystem_launch_test.py --rviz`)

`simulation` -> (gate) -> `moveit`, `perception`, `coordinator prompt:=false`,
`rviz rviz_config:=camera_view.rviz` as separate launches: all 5 controllers active;
`/cup_sorting/ready` after 9 s; both cube poses published; `plan_check` succeeded (51-point
plan) with the arm unchanged (0.0000 rad); RViz running; all launches stopped with SIGINT,
nothing left. **PASS 8/8.**

### 5.2 Every documented service (headless, `prompt:=false`)

| Service | Result |
|---|---|
| `/cup_sorting/ready` | success, `ready` |
| `/cup_sorting/plan_check` | success, 52-point plan, NOT executed |
| `/cup_sorting/go_ready` | success, `ready: reached [0.2, 0.28, 0.52] (verified)` |
| `/cup_sorting/approach_test_red` | success, free-space gripper cycle 0.441 -> 0.000 -> 0.441 rad |
| `/cup_sorting/grasp_test_red`, `/cup_sorting/grasp_test_blue` | success, grasp + lift verified, cube put back |
| `/cup_sorting/pick_place_red`, `/cup_sorting/pick_place_blue` | success, placed in cup (verified); `check_cup_contents` exit 0 (red 2.4 cm, blue 2.6 cm from the axis) |
| `/cup_sorting/run` (cubes already in the cup) | success in 2 s (both skipped as already inside, see §9) |
| `/cup_sorting/go_home` | success, `home: reached (verified)` |

Also executed: `ros2 control list_controllers`, `ros2 node list`, `ros2 topic hz`,
`gz model --list`, `ros2 param dump` (coordinator, detector), `ros2 run tf2_tools view_frames`,
`xacro ... | check_urdf` (`Successfully Parsed XML`), `wait_for_controllers --help`.

### 5.3 Same-day run of the ORIGINAL workspace (baseline)

`ros2 launch ~/openarm_sorting_ws/simulation_config/launch/cup_sorting.launch.py auto_start:=true gz_gui:=false rviz:=false`
with the original install sourced, `PYTHONDONTWRITEBYTECODE=1` and `ROS_LOG_DIR` outside the
original (read-only use):

| | Original (today) | New workspace (4 runs) | Original (documented, 17 Nov run 2) |
|---|---|---|---|
| task duration | 53 s after READY | 48-53 s after "y" | 46 s after READY |
| grasp angle red / blue | 0.200 / 0.197 rad | 0.198-0.210 / 0.195-0.219 rad | 0.209 / 0.225 rad |
| red cube | inside, on the floor, 2.1 cm from the axis | inside, on the floor, 0.5-2.4 cm | inside, on the floor, 0.3 cm |
| blue cube | inside (leaning), 2.6 cm | inside (leaning), 2.3-2.9 cm | inside (leaning), 2.5 cm |

The variation between runs comes from OMPL's randomised planning and is the same in both
workspaces. Conclusion: the demonstrated behaviour is preserved.

### 5.4 Failure handling

| Test | Result |
|---|---|
| second `cup_sorting.launch.py` while a simulation runs | refused: `[cup_sorting] ERROR: a Gazebo simulation is already running ...` + the process list; nothing started |
| `kill -9` of the coordinator during a run | `process[cup_sorting_coordinator-9] was required: shutting down launched system`; everything stopped; leftover Gazebo server stopped by the cleanup; no processes left |
| SIGINT to `ros2 launch` only (no process-group signal), the original's orphan scenario | launch escalated the `gz` front-end to SIGTERM; `[simulation] stopped 1 leftover Gazebo process(es) [<pid>]`; no processes left |
| Ctrl+C in a terminal (pty) | 8 end-to-end runs: launch stopped, no Gazebo/ROS process left |

### 5.5 RViz and Gazebo GUI

In the GUI run, `gz sim server` + `gz sim gui` and `rviz2` (`cup_sorting.rviz`) ran without
errors; RViz was subscribed to `/openarm_zed/image`, `/openarm_zed/depth_image`,
`/openarm_zed/points`, `/cup_sorting/debug_image` and `/cup_sorting/detection_markers`.
`camera_view.rviz` ran in the subsystem test. `image_view.launch.py image:=/openarm_zed/depth_image`
started rqt_image_view.

### 5.6 VS Code snap terminal

`ros2 run rqt_image_view rqt_image_view` aborts in this terminal
(`GLib-GIO-ERROR: Settings schema ... does not contain a key named 'antialiasing'`); with the
environment fix it runs. RViz and rqt started through the launch files (which apply the fix)
ran without errors.

## 6. Problems found and fixed during validation

| # | Problem | Fix |
|---|---|---|
| 1 | `ament_python` buildtool dependency is not a rosdep key (inherited) | removed from the four manifests |
| 2 | unit test called `get_launch_description(None)` | uses a `LaunchContext` |
| 3 | `/dev/tty` opened as one `'r+'` text stream -> "not seekable", prompt could not ask | separate read and write streams (`Terminal` class) |
| 4 | the prompt's own log line arrived after the question (launch pipes are asynchronous) | `--disable-stdout-logs`; operator output written to the terminal directly |
| 5 | the prompt could ask before the coordinator had its joint limits (race also present in the original wait script) | new read-only `/cup_sorting/ready` service; the prompt waits for it |
| 6 | `gz sim` server orphaned when the front-end is SIGTERM'd (known issue of the original, solved there in bash) | run marker + `OnProcessExit` cleanup in `simulation.launch.py` |
| 7 | pre-flight refusal still started the readiness gate, which then printed a misleading error | pre-flight in `cup_sorting.launch.py` before anything starts; gate killed by a signal -> no error |
| 8 | rqt_image_view unusable from the VS Code snap terminal after removing `view_camera.sh` | `image_view.launch.py` with the environment fix |

## 7. Code quality checks

`python3 -m flake8 --max-line-length 120 --select E,W,F` on the project packages and tests:
no pyflakes errors (unused imports, undefined names); remaining 12 findings are cosmetic and
in carried-over code (6x E702 semicolons in `generate_coffee_cup.py`, 5x E501 lines of
121-131 characters in `coordinator.py`, 1x in a test) and were left unchanged on purpose
(no unrelated rewrites of validated motion code).

## 8. Original workspace unchanged

```bash
# before the migration and again at the end:
cd ~/openarm_sorting_ws && find . -type f -not -path './build/*' -not -path './install/*' -not -path './log/*' -print0 \
    | sort -z | xargs -0 sha256sum > snapshot.sha256
```

| Check | Before | After |
|---|---|---|
| files outside build/install/log (incl. `.git`) | 1042 | 1042, **all SHA-256 identical** |
| files in build/install/log | 1182 | 1182 |
| `git status --short` | 3 modified + 1 untracked (pre-existing) | identical |

The only metadata difference: the modification time of the four `.git` **directories**
(workspace and the three upstream clones) changed, from read-only git commands
(`git status`, `git rev-parse`, cloning from them), which create and remove a transient lock
file. No entry inside them is newer than the snapshot; their `index` files keep their
17 Nov timestamps.

## 9. Known limitations and remaining issues

* **Mermaid diagrams were syntax-reviewed but not rendered**: Node.js / mermaid-cli are not
  installed, and the diagrams were not sent to an online renderer. A structural check
  (balanced brackets/quotes, no `;` in sequence diagrams) passed for all six.
* **No human visual inspection of the GUI** in this validation: Gazebo GUI and RViz were
  verified as running processes with the right subscriptions, not by looking at the screen.
* **Fresh-machine install not executed**: all dependencies were already installed;
  `rosdep install` therefore had nothing to do, and `vcs import` from GitHub was not run
  (the repositories are reachable: `vcs validate`). The upstream clones were made from the
  local repositories at the same commits.
* **Reliability is not statistically characterised**: 5 complete sorts in the new workspace
  (4 via run/auto_start, 1 via the single-cube services) and 1 in the original today, all
  successful. OMPL is randomised; the coordinator retries planning and recovers on failures.
* **Inherited behaviours (unchanged on purpose):** the second cube usually ends up leaning on
  the first one (inside the cup, not flat); `/cup_sorting/run` with both cubes already in the
  cup reports `red: placed in cup (verified) | blue: ...` because the coordinator skips cubes
  it sees inside the cup with that message; the right arm is not used; `move_group` and the
  `gz` front-end often need the 5 s SIGTERM escalation on shutdown (harmless).
* `ros2 launch` exits with code 0 when the pre-flight check refuses to start (launch's
  `Shutdown` semantics); the refusal is visible in the output.
* The system tests are not part of `colcon test` (they need Gazebo and 1-2 min each); run them
  manually ([../../tests/README.md](../../tests/README.md)).
* The workspace is not a git repository yet (no commits were made).
