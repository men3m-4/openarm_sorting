# File migration report

Source: `~/openarm_sorting_ws` (original, **unchanged**, kept as reference and
rollback). Target: `~/openarm_sorting_ws_clean`. Date: 2026-11-19.

The original was read only. Its state at migration time was the working tree of its git
repository at `97f0b50` **plus uncommitted changes** (modified `simulation_config/CUP_SORTING_PROJECT.md`,
`simulation_config/launch/cup_sorting.launch.py`, `simulation_config/start_cup_sorting.sh`,
new `simulation_config/tools/wait_cup_sorting_ready.py`). The working tree (the latest
working system) was used as the source. A SHA-256 snapshot of all 1042 files outside
`build/`, `install/`, `log/` was taken before the migration and compared afterwards (see
[VALIDATION_REPORT.md](VALIDATION_REPORT.md) §8).

Legend: **Copied** = identical content, new location; **Copied+edited** = same file with the
listed edits; **Consolidated** = content merged into another file; **Replaced** = function
re-implemented elsewhere; **Excluded** = not in the new workspace.

Abbreviations: `O/` = original workspace root, `N/` = new workspace root,
`SC/` = `O/simulation_config/`, `DESC/` = `N/src/openarm_sorting_description/`,
`VIS/` = `N/src/openarm_sorting_vision/`, `MAN/` = `N/src/openarm_sorting_manipulation/`,
`BR/` = `N/src/openarm_sorting_bringup/`.

## 1. Upstream repositories

| Original | New | Action | Reason |
|---|---|---|---|
| `O/src/openarm_ros2/` (git, `8f23aea`, clean) | `N/src/upstream/openarm_ros2/` | Copied (git clone of the local repository at the same commit; `origin` reset to https://github.com/enactic/openarm_ros2.git) | upstream, unmodified; grouped under `src/upstream/` so it is clearly identifiable |
| `O/src/openarm_description/` (git, `8dbfcc9`, 1 untracked file) | `N/src/upstream/openarm_description/` | Copied (clone at the same commit, clean) | upstream, unmodified |
| `O/src/openarm_description/presets/v2/default_bimanual_gazebo.yaml` (untracked project file inside upstream) | `DESC/presets/default_bimanual_gazebo.yaml` | Moved (copied; header comment added, YAML content identical) | project file did not belong inside the upstream clone; loaded through the new Xacro's `robot_preset_file` argument |
| `O/src/openarm_can/` (git, `26b266a`, clean) | `N/src/upstream/openarm_can/` | Copied (clone at the same commit) | build dependency of `openarm_hardware` |
| `O/src/openarm_ros2/openarm.repos` | (inside the clone) + `N/upstream.repos` | Copied + new file | `upstream.repos` pins all three repositories to exact commits for `vcs import` |

## 2. simulation_config/

| Original | New | Action | Reason / edits |
|---|---|---|---|
| `SC/openarm_v20_gazebo.urdf.xacro` | `DESC/urdf/openarm_v20_gazebo.urdf.xacro` | Copied+edited | (1) absolute controllers path replaced by `$(arg controllers_file)` (default from the package); (2) preset loaded from this package via a local copy of the upstream `openarm_robot` macro body (`openarm_sorting_robot`, preset path is the only difference); (3) comment about passing the URDF on the command line removed (`start_simulation.sh` is gone). Expanded URDF identical to the original except comments and the controllers path (validated) |
| `SC/openarm_bimanual_gazebo_controllers.yaml` | `DESC/config/gazebo_controllers.yaml` | Renamed (header comment added, YAML content identical) | owned by the simulated robot; referenced by its URDF |
| `SC/launch/gripper_collision_patch.py` | `DESC/openarm_sorting_description/gripper_collision_patch.py` | Copied+edited | now an installed Python module (was imported via `sys.path` hack); docstrings and type hints only, logic identical |
| `SC/launch/cup_sorting.launch.py` | `BR/launch/{cup_sorting,simulation,robot,controllers,moveit,rviz,perception,coordinator}.launch.py` + `BR/openarm_sorting_bringup/launch_support.py` + `BR/config/moveit_overrides.yaml` + `DESC/openarm_sorting_description/robot_description.py` | Consolidated / split | one monolithic file with absolute paths -> launch hierarchy with standalone subsystem files; MoveIt overrides moved from Python dicts to YAML (effective parameters identical, validated); startup sequencing kept event-driven and extended with return-code checks, pre-flight check, required-process shutdown and Gazebo leftover cleanup |
| `SC/start_cup_sorting.sh` | `cup_sorting.launch.py` + `MAN/openarm_sorting_manipulation/task_prompt.py` + `launch_support.py` | Replaced | the one command is now `ros2 launch openarm_sorting_bringup cup_sorting.launch.py`. Its functions: GPU env -> `nvidia_offload` argument; VS Code snap env fix -> `snap_terminal_fix_actions()`; single-Gazebo check -> `running_gazebo_servers()`; build-if-stale -> removed (standard `colcon build`); per-run `ROS_LOG_DIR` under the workspace -> standard `~/.ros/log` (documented how to override); process-group handling + marker cleanup -> launch-native cleanup in `simulation.launch.py`; readiness wait + `read -p` question -> `task_prompt` node |
| `SC/tools/wait_cup_sorting_ready.py` | `MAN/openarm_sorting_manipulation/task_prompt.py` + coordinator service `/cup_sorting/ready` | Replaced | readiness conditions now come from the coordinator itself (no duplicated checks) plus the detection check |
| `SC/config/cup_sorting_scene.yaml` | `MAN/config/cup_sorting_scene.yaml` | Copied+edited | owned by the coordinator (its consumer); comments updated (world path, cup not detected, no cube poses); YAML content identical (validated) |
| `SC/worlds/openarm_cup_sorting_world.sdf` | `BR/worlds/openarm_cup_sorting_world.sdf` | Copied+edited | comments only (paths of the scene file / resource path) |
| `SC/models/coffee_cup/` (model.config, model.sdf, meshes/cup_body.obj, meshes/cup_handle.obj) | `BR/models/coffee_cup/` | Copied+edited | generator path in the "generated by" comment / description updated; geometry identical; the generator reproduces the shipped files byte for byte |
| `SC/models/cup_sorting_table/` | `BR/models/cup_sorting_table/` | Copied | used by the world |
| `SC/models/red_cube/`, `SC/models/blue_cube/` | `BR/models/red_cube/`, `BR/models/blue_cube/` | Copied | used by the world |
| `SC/models/archive/{red_bin,blue_bin,red_cube,blue_cube,sorting_table}.sdf` | - | **Excluded** | not referenced by any world, launch file or script (searched); the original documentation calls them "previous ... SDFs (kept, not used)"; the two bins belong to an abandoned two-bin design that the current application explicitly does not use. Still available in the original |
| `SC/tools/generate_coffee_cup.py` | `BR/scripts/generate_coffee_cup.py` | Copied+edited | development tool that regenerates the cup model; path comments updated; still writes `../models/coffee_cup` relative to itself, i.e. into the new package |
| `SC/rviz/cup_sorting.rviz` | `BR/rviz/cup_sorting.rviz` | Copied | RViz config of the complete application |
| `SC/camera_view.rviz` | `BR/rviz/camera_view.rviz` | Copied | camera-only view, now `rviz.launch.py rviz_config:=camera_view.rviz` |
| `SC/view_camera.sh` | `BR/launch/rviz.launch.py rviz_config:=camera_view.rviz` (mode `rviz`), `BR/launch/image_view.launch.py` (modes `rqt` and `depth`) | Replaced | same three viewers, started through launch files that apply the snap terminal fix (validated: plain `ros2 run rqt_image_view` aborts in the VS Code snap terminal, the launch file works) |
| `SC/start_simulation.sh` | `BR/launch/simulation.launch.py` | Replaced / **Excluded** | legacy setup: default Gazebo world with DART (no finger mimic), fixed `sleep`s, URDF via `/tmp` and the command line, bidirectional camera bridges, one spawner per controller. Superseded by `simulation.launch.py` (cup-sorting world, events, no sleeps) |
| `SC/move_group_gazebo.launch.py` | `BR/launch/moveit.launch.py` | Replaced / **Excluded** | legacy, hard-coded `~/openarm_sorting_ws` path, no runtime overrides, not referenced by the working system |
| `SC/rviz_gazebo.launch.py` | `BR/launch/rviz.launch.py` | Replaced / **Excluded** | legacy, hard-coded path, not referenced by the working system |
| `SC/CUP_SORTING_PROJECT.md` | `N/README.md` + `N/docs/**` | Consolidated | all technical content (architecture, decisions, perception, MoveIt, sequence, test results, known issues, troubleshooting) migrated into the new documentation with new paths/commands; the original test results are the baseline in VALIDATION_REPORT.md |
| `SC/__pycache__/`, `SC/launch/__pycache__/` (4 `.pyc`) | - | **Excluded** | generated Python bytecode |

## 3. Project ROS 2 packages

### src/openarm_sorting_vision

| Original | New | Action | Reason / edits |
|---|---|---|---|
| `package.xml` | `VIS/package.xml` | Copied+edited | removed invalid rosdep key `ament_python` (buildtool); removed unused `sensor_msgs_py`, `tf2_msgs`; added `gz_transport_vendor`, `gz_msgs_vendor` (ground truth), `python3-pytest`; version 0.2.0 |
| `setup.py` | `VIS/setup.py` | Copied+edited | installs `config/`; entry point `red_cube_detector` removed |
| `setup.cfg`, `resource/openarm_sorting_vision` | same | Copied | |
| `openarm_sorting_vision/cube_detector.py` | same | Copied+edited | topic names, output namespace and HSV thresholds are parameters (defaults = previous constants); algorithm unchanged |
| `openarm_sorting_vision/color_segmentation.py` | same | Copied+edited | `make_spec()` / `flat_hue_ranges()` added for parameterised thresholds; `COLOR_SPECS`, masks and blob filter unchanged |
| `openarm_sorting_vision/camera_geometry.py` | same | Copied+edited | docstrings only |
| `openarm_sorting_vision/ground_truth.py` | same | Copied | |
| `openarm_sorting_vision/detection_validator.py` | same | Copied+edited | world name, detection namespace and pass threshold are parameters (same defaults) |
| `openarm_sorting_vision/__init__.py` | same | Copied+edited | package docstring |
| `openarm_sorting_vision/red_cube_detector.py` | - | **Excluded** | first prototype (red only, point-cloud centroid, publishes `/detected_cube_position`); superseded by `cube_detector`; no launch file, script or node subscribes to its output (searched); the original documentation describes it as "the original red_cube_detector was kept". Available in the original |
| `openarm_sorting_vision/__pycache__/` (7 files) | - | **Excluded** | generated bytecode |
| - | `VIS/config/cube_detector.yaml` | New | all detector parameters with the validated values |
| - | `VIS/test/test_perception_core.py` | New | 7 unit tests |

### src/openarm_sorting_manipulation

| Original | New | Action | Reason / edits |
|---|---|---|---|
| `package.xml` | `MAN/package.xml` | Copied+edited | removed invalid `ament_python` key; added missing `tf2_ros`, `builtin_interfaces`, `ament_index_python`, `python3-yaml`, test dependency |
| `setup.py` | `MAN/setup.py` | Copied+edited | installs `config/`; new entry point `task_prompt` |
| `setup.cfg`, `resource/` | same | Copied | |
| `openarm_sorting_manipulation/coordinator.py` | same | Copied+edited | (1) code defaults `cup.base` and release offsets aligned with the YAML that was always loaded; (2) `ready_tcp_candidates`, `gripper_max_effort`, `required_controllers`, `detection_namespace` parameters (previously constants, same values); (3) new read-only service `/cup_sorting/ready` + `missing_conditions()` extracted from `wait_ready()` (same conditions); (4) inline imports moved to the top. Sequence, motion and verification logic unchanged |
| `openarm_sorting_manipulation/gripper.py` | same | Copied+edited | `max_effort` constructor argument (default 7.0 as before) |
| `openarm_sorting_manipulation/moveit_interface.py`, `planning_scene.py`, `ros_utils.py` | same | Copied | |
| `openarm_sorting_manipulation/check_cup_contents.py` | same | Copied+edited | cup geometry read from `config/cup_sorting_scene.yaml` instead of a duplicated constant; `--scene-file`, `--world` options |
| `openarm_sorting_manipulation/__init__.py` | same | Copied+edited | docstring |
| `openarm_sorting_manipulation/__pycache__/` (7 files) | - | **Excluded** | generated bytecode |
| - | `MAN/openarm_sorting_manipulation/task_prompt.py` | New | operator confirmation node |
| - | `MAN/config/coordinator.yaml` | New | all coordinator motion/gripper/verification parameters (validated values) |
| - | `MAN/test/test_manipulation_core.py` | New | 16 unit tests |

### New packages

| New | Reason |
|---|---|
| `N/src/openarm_sorting_description/` | the simulated robot (Xacro wrapper, Gazebo preset, controllers, finger patch) had no package: it lived in `simulation_config/` and inside the upstream clone |
| `N/src/openarm_sorting_bringup/` | launch files, world, models, RViz configs had no package and were addressed with absolute paths |

## 4. Workspace root

| Original | New | Action | Reason |
|---|---|---|---|
| `O/.git/` | - | **Excluded** | history of the original workspace (3 commits); it remains in the original. The new workspace is not a git repository (see DEVELOPMENT_GUIDE.md "Version control") |
| `O/.gitignore` | `N/.gitignore` | Replaced | new layout (`src/upstream/`, Python caches, editors) |
| `O/PROJECT_TECHNICAL_AUDIT.md` | - | **Excluded** (content consolidated) | audit of the original workspace (its upstream comparison, file-by-file changes and decisions); the decisions are in SYSTEM_ARCHITECTURE.md §4, the file-level facts are superseded by this report. Kept in the original |
| `O/PROJECT_NEXT_PHASE_PLAN.md` | - | **Excluded** | plan for a future phase (dual-arm, two cups, ZED Mini emulation) that was never implemented ("awaiting approval"); including it would describe behaviour the workspace does not have. Kept in the original |
| `O/ZED_SDK_GAZEBO_FEASIBILITY.md` | - | **Excluded** (conclusion consolidated) | feasibility study for that future phase; its conclusion for the current system (the ZED SDK cannot consume the Gazebo sensor) is in SYSTEM_ARCHITECTURE.md §4. Kept in the original |
| `O/frames_2026-11-17_01.47.13.gv`, `.pdf` | - | **Excluded** | `tf2_tools view_frames` output (a debugging snapshot); regenerate with `ros2 run tf2_tools view_frames` |
| `O/build/` (22 MB), `O/install/` (6.2 MB), `O/log/` (3.5 MB) | - | **Excluded** | colcon artefacts; rebuilt from source in the new workspace |
| `O/logs/cup_sorting/ros_*` (30 run directories, 6.8 MB) | - | **Excluded** | ROS logs of earlier runs (generated); the validation evidence that matters is summarised in VALIDATION_REPORT.md (original results as baseline) |
| `O/logs/simulation/robot_state_publisher.log` | - | **Excluded** | log of the legacy `start_simulation.sh` |
| `O/logs/next_phase_analysis/{kin.py,grid.py,cups.py,grid.json,grid_output.txt}` | - | **Excluded** | offline FK/IK reachability study for the unimplemented dual-arm/two-cup phase; not used by any package. Kept in the original |
| - | `N/README.md`, `N/CONTRIBUTING.md`, `N/LICENSE` (Apache-2.0, the licence all packages declare), `N/upstream.repos`, `N/docs/**`, `N/tests/**` | New | documentation, licence, reproducibility, system tests |

## 5. Not copied for safety

No credentials, keys or personal configuration were found in the original workspace. The
maintainer e-mail in the manifests is `muhammad.abdelmoniem4@gmail.com`. No
machine-specific temporary files (`/tmp/openarm_v20_gazebo_final.urdf` was only referenced by
the excluded `start_simulation.sh`) were copied.

## 6. Absolute / machine-specific paths removed

| Where (original) | Path | Now |
|---|---|---|
| Xacro gz_ros2_control `<parameters>` | `~/openarm_sorting_ws/simulation_config/openarm_bimanual_gazebo_controllers.yaml` | `$(arg controllers_file)`, default `$(find openarm_sorting_description)/config/gazebo_controllers.yaml` |
| `cup_sorting.launch.py` | `~/openarm_sorting_ws`, `simulation_config/...`, `/opt/ros/jazzy/share` | `get_package_share_directory()` |
| `start_cup_sorting.sh`, `start_simulation.sh`, `view_camera.sh` | `$HOME/openarm_sorting_ws` | replaced by launch files |
| `move_group_gazebo.launch.py`, `rviz_gazebo.launch.py` | `~/openarm_sorting_ws/simulation_config/...` | excluded (replaced) |
| world comment | `~/openarm_sorting_ws/simulation_config/models` | resource path set by the launch |

`grep -rn "/home/\|openarm_sorting_ws[^_]\|simulation_config" src/openarm_sorting_*` in the
new workspace finds nothing.
