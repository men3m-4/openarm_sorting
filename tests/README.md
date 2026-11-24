# System tests

Unit tests live in each package's `test/` directory and run with `colcon test`. This folder
holds the **system-level** tests that start the real simulation (Gazebo, MoveIt, perception,
coordinator). They need a built and sourced workspace and must be run when no other Gazebo
simulation is running.

```bash
source /opt/ros/jazzy/setup.bash
source ~/openarm_sorting_ws_clean/install/setup.bash
cd ~/openarm_sorting_ws_clean
```

## integration/e2e_cup_sorting.py

Starts `cup_sorting.launch.py` in a pseudo-terminal (exactly like a user), answers the
operator question and checks the outcome; requires `python3-pexpect`.

```bash
python3 tests/integration/e2e_cup_sorting.py --answer n           # ~1 min
python3 tests/integration/e2e_cup_sorting.py --answer y           # ~2 min, full sort
python3 tests/integration/e2e_cup_sorting.py --answer y --gui     # with Gazebo GUI and RViz
```

| Checks | `--answer n` | `--answer y` |
|---|---|---|
| prompt only after readiness, banner, nothing moved before confirmation | yes | yes |
| invalid input rejected, question repeated | yes | yes |
| "NOT executed", no sequence started, arm unchanged, cubes at spawn poses (ground truth), simulation still running | yes | - |
| task success, red before blue, both grasps verified, both placements camera-verified, both cubes in the cup (ground truth), simulation still running | - | yes |
| Ctrl+C stops the launch, no Gazebo / ROS process left | yes | yes |

Output: one line per check and `RESULT: PASS|FAIL`; the full terminal transcript, a
`result.txt` and all ROS logs are written to `--log-dir` (default `./e2e_logs/<run>/`).

## integration/subsystem_launch_test.py

Starts `simulation`, `moveit`, `perception`, `coordinator` (and with `--rviz` `rviz` with
`camera_view.rviz`) as separate `ros2 launch` processes, checks `/cup_sorting/ready`, both cube
poses and a `plan_check` (plan only; verifies the arm did not move), then stops everything
with SIGINT and checks that nothing is left.

```bash
python3 tests/integration/subsystem_launch_test.py [--rviz] [--log-dir DIR]
```

## integration/e2e_coffee.py (coffee / portafilter task)

Starts `coffee_portafilter.launch.py` headless without the prompt, checks readiness and
`plan_check` (nothing moves), then executes the task and checks the outcome against Gazebo
ground truth (`check_portafilter`), and that Ctrl+C (SIGINT) leaves no process behind.

```bash
python3 tests/integration/e2e_coffee.py                         # one /run (install + remove), ~3 min
python3 tests/integration/e2e_coffee.py --runs 3                # repeatability on one simulation
python3 tests/integration/e2e_coffee.py --mode install-remove   # /install (LOCKED by ground truth), then /remove
python3 tests/integration/e2e_coffee.py --no-grasp-assist       # pure friction grasp (alternative)
python3 tests/integration/e2e_coffee.py --rgb-only              # perception without depth (alternative)
```

Output: one PASS/FAIL line per check, `RESULT: PASS|FAIL`, logs in `--log-dir`
(default `./e2e_coffee_logs/`).
