# Development guide

## Workflow

```bash
source /opt/ros/jazzy/setup.bash
cd ~/openarm_sorting_ws_clean
colcon build --symlink-install
source install/setup.bash
```

With `--symlink-install`, edits to Python modules, launch files and YAML/SDF files take
effect at the next start without rebuilding. Rebuild after adding or renaming files,
changing `setup.py`, `package.xml` or entry points.

During development start the subsystems separately
([../execution/SIMULATION_RUNBOOK.md](../execution/SIMULATION_RUNBOOK.md) §B) and restart only
the one you change, e.g. the coordinator:

```bash
ros2 launch openarm_sorting_bringup coordinator.launch.py prompt:=false
ros2 service call /cup_sorting/plan_check std_srvs/srv/Trigger      # no motion
```

## Where to change what

| Change | File |
|---|---|
| cube colours / HSV thresholds / stability | `openarm_sorting_vision/config/cube_detector.yaml` |
| detection algorithm | `openarm_sorting_vision/openarm_sorting_vision/{cube_detector,color_segmentation,camera_geometry}.py` |
| grasp geometry, gripper angles, speeds, tolerances | `openarm_sorting_manipulation/config/coordinator.yaml` |
| table / cup geometry used for planning | `openarm_sorting_manipulation/config/cup_sorting_scene.yaml` **and** the world/models |
| task sequence | `openarm_sorting_manipulation/openarm_sorting_manipulation/coordinator.py` |
| world, object poses, physics | `openarm_sorting_bringup/worlds/openarm_cup_sorting_world.sdf` (+ scene YAML) |
| cup model | `openarm_sorting_bringup/scripts/generate_coffee_cup.py`, then re-run it |
| MoveIt runtime settings | `openarm_sorting_bringup/config/moveit_overrides.yaml` |
| controllers | `openarm_sorting_description/config/gazebo_controllers.yaml` (the launch files derive the controller list from it) |
| robot sensor / frames | `openarm_sorting_description/urdf/openarm_v20_gazebo.urdf.xacro` |
| startup order / arguments | `openarm_sorting_bringup/launch/*.launch.py` |

## Rules

1. **Upstream stays untouched.** Nothing under `src/upstream/` is edited. If a change in an
   upstream package is unavoidable, prefer an override in a project package (as done for the
   Gazebo preset and the MoveIt runtime parameters); otherwise document the patch in
   [../architecture/FILE_MIGRATION_REPORT.md](../architecture/FILE_MIGRATION_REPORT.md) and
   in the README.
2. **No absolute paths.** Locate files with `get_package_share_directory()` / `$(find ...)`.
3. **Configuration in YAML, defaults in code identical.** Every parameter has a declared
   default equal to the YAML value, so a node behaves the same with and without the file.
4. **Verify, do not assume.** Robot motions are checked with the joint-error check; grasps
   with the finger angle; placements with the camera. Keep it that way for new steps.
5. **Ground truth only in test tools.** `ground_truth.py` must never be used by robot logic.
6. **No fixed sleeps for startup.** Use readiness checks (services, actions, controller
   states, topics) and `OnProcessExit` events.
7. **Behaviour changes are documented and re-validated** with the end-to-end test
   (`tests/integration/e2e_cup_sorting.py --answer y`) and `check_cup_contents`.

## Tests

| Level | Command | Needs Gazebo |
|---|---|---|
| unit (42 tests) | `colcon test --packages-select openarm_sorting_description openarm_sorting_vision openarm_sorting_manipulation openarm_sorting_bringup && colcon test-result --verbose` | no |
| subsystems | `python3 tests/integration/subsystem_launch_test.py [--rviz]` | yes |
| end-to-end | `python3 tests/integration/e2e_cup_sorting.py --answer {y,n} [--gui]` | yes |
| perception accuracy | `ros2 run openarm_sorting_vision detection_validator` (running simulation) | yes |
| placement | `ros2 run openarm_sorting_manipulation check_cup_contents` (running simulation) | yes |

## Code style

Python 3.12, PEP 8 with 120-character lines, single quotes, type hints on public helpers,
docstrings that explain *why*. Logging through the node logger (`self.get_logger()`), with
`[colour] N. step` prefixes in the coordinator so a run can be followed step by step.
Check with `python3 -m flake8 --max-line-length 120 --select E,W,F src/openarm_sorting_*`.

## Version control

The workspace is not a git repository yet. A suggested start:

```bash
cd ~/openarm_sorting_ws_clean
git init && git add . && git commit -m "OpenArm cup sorting: clean workspace"
```
`.gitignore` excludes `build/`, `install/`, `log/` and `src/upstream/` (the upstream
repositories are restored with `vcs import src/upstream < upstream.repos`).
