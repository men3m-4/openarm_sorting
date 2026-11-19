# Launch architecture

All launch files are in `openarm_sorting_bringup/launch/` and are installed, so they are
started with `ros2 launch openarm_sorting_bringup <file>`.

Diagram: [diagrams/launch_sequence.md](diagrams/launch_sequence.md)

## 1. Hierarchy

```text
cup_sorting.launch.py                       complete application (main entry point)
├── simulation.launch.py                    Gazebo world + bridges + robot + controllers
│   ├── ros_gz_sim/gz_sim.launch.py         (upstream) Gazebo server [+ GUI]
│   ├── robot.launch.py                     robot_state_publisher (Gazebo description)
│   ├── ros_gz_sim create                   spawn the robot from /robot_description
│   └── controllers.launch.py               [after a successful spawn] one spawner, 5 controllers
├── wait_for_controllers                    readiness gate (exits when all controllers are active)
└── [after the gate succeeds]
    ├── moveit.launch.py                    move_group
    ├── rviz.launch.py                      RViz              (rviz:=true)
    ├── perception.launch.py                cube_detector     (perception:=true)
    └── coordinator.launch.py               coordinator       (coordinator:=true)
                                            + task_prompt     (prompt:=true, auto_start:=false)

image_view.launch.py                        rqt_image_view on a camera topic (separate tool)
```

Every child launch file also works on its own (separate terminals, for development):
`simulation.launch.py` first, then any of `moveit`, `perception`, `coordinator`, `rviz`.
This composition is tested by `tests/integration/subsystem_launch_test.py`.

## 2. Launch files and arguments

| File | Starts | Arguments (default) |
|---|---|---|
| `cup_sorting.launch.py` | everything below | `gz_gui` (true), `rviz` (true), `perception` (true), `coordinator` (true), `prompt` (true), `auto_start` (false), `detector_output` (own_log), `nvidia_offload` (auto), `controllers_file` (''), `controllers_timeout` (240) |
| `simulation.launch.py` | Gazebo, robot_state_publisher, ros_gz_bridge, spawn, controllers | `gz_gui` (true), `world` (installed world), `controllers_file` (''), `nvidia_offload` (auto) |
| `robot.launch.py` | robot_state_publisher | `use_sim_time` (true), `controllers_file` ('') |
| `controllers.launch.py` | controller spawner | `controllers_file` (''), `controller_manager` (/controller_manager), `controller_manager_timeout` (180) |
| `moveit.launch.py` | move_group | `use_sim_time` (true), `controllers_file` ('') |
| `rviz.launch.py` | rviz2 | `rviz_config` (cup_sorting.rviz), `use_sim_time` (true), `controllers_file` (''), `nvidia_offload` (auto) |
| `perception.launch.py` | cube_detector | `params_file` (vision config), `output` (screen), `use_sim_time` (true) |
| `image_view.launch.py` | rqt_image_view | `image` (/openarm_zed/image) |
| `coordinator.launch.py` | coordinator [+ task_prompt] | `auto_start` (false), `prompt` (true), `coordinator_params`, `scene_file`, `controllers_file` ('') |

`controllers_file` empty means `openarm_sorting_description/config/gazebo_controllers.yaml`.
`ros2 launch openarm_sorting_bringup <file> --show-args` prints the arguments with their
descriptions.

## 3. Sequencing: event-driven, no fixed sleeps

| Step | Waits for | Mechanism |
|---|---|---|
| Spawn robot | `/robot_description` | `ros_gz_sim create -topic` waits for the topic |
| Load controllers | spawn finished successfully | `OnProcessExit(spawn)`, return code checked |
| Activate controllers | `/controller_manager` services | the spawner waits (`--controller-manager-timeout`) |
| MoveIt, RViz, perception, coordinator | all 5 controllers ACTIVE | `wait_for_controllers` gate + `OnProcessExit(gate)` |
| Operator question | coordinator ready + both cubes detected | `task_prompt` polls `/cup_sorting/ready` and the pose topics |
| Robot motion | operator "y" (or `auto_start:=true`) | `/cup_sorting/run` |

The coordinator's own readiness check (move_group interfaces, controllers, gripper, joint
limits from `/robot_description`) is exposed read-only as `/cup_sorting/ready`, so the prompt
asks only when a "y" can really start the task.

The only time-based waits left are poll intervals of readiness checks (e.g. the controller
list every 0.5 s) and the settle times inside the manipulation sequence (finger settling),
which are part of the validated robot behaviour.

## 4. Failure handling and shutdown

| Event | Result |
|---|---|
| Another Gazebo server already running | `cup_sorting` / `simulation` refuse to start (error lists the processes), nothing is started |
| World file missing | error + shutdown |
| Robot spawn fails (non-zero exit) | error + shutdown "robot spawn failed" |
| Controllers not active within `controllers_timeout` | error + shutdown "controllers not active" |
| Gazebo exits (e.g. window closed) | shutdown (`on_exit_shutdown` of gz_sim.launch.py) |
| robot_state_publisher, ros_gz_bridge, move_group, cube_detector or coordinator exits | shutdown, reason logged ("... exited") |
| RViz closed | nothing else stops |
| task_prompt finishes (any answer) | nothing else stops; the simulation keeps running |
| Ctrl+C | launch sends SIGINT to every process (SIGTERM after 5 s) |
| `gz` front-end exits but its server/GUI child survives | the child is stopped by the launch (see below) |

**Gazebo leftover cleanup.** `ros2 launch` only signals its direct child, the `gz` ruby
front-end. When that front-end does not exit within 5 s of SIGINT it gets SIGTERM and dies
without stopping the `gz sim server` / `gz sim gui` processes it created, which then keep
running and keep owning `/controller_manager` (the original project handled this in its
shell wrapper). `simulation.launch.py` therefore sets a unique environment marker
`OPENARM_SORTING_SIM_RUN` for every process of the run and, when the Gazebo front-end has
exited, stops any `gz sim` process that still carries this run's marker (SIGTERM, then
SIGKILL after 5 s). Processes of other runs or other users are never touched. Validated:
`[simulation] stopped 1 leftover Gazebo process(es)` in exactly the failure case.

## 5. Process environment

`launch_support.environment_actions()` (used by `simulation.launch.py` and `rviz.launch.py`;
`image_view.launch.py` uses only the snap fix):

* `GZ_SIM_RESOURCE_PATH` = installed `share/openarm_sorting_bringup/models` + the parent
  directories of `openarm_description` and `zed_description` (for the robot's `package://`
  meshes) + whatever the user already had. No absolute paths in any file.
* `nvidia_offload:=auto|true|false`: NVIDIA PRIME render offload variables
  (`__NV_PRIME_RENDER_OFFLOAD`, `__GLX_VENDOR_LIBRARY_NAME`, `__EGL_VENDOR_LIBRARY_FILENAMES`).
  `auto` sets them only if `/usr/share/glvnd/egl_vendor.d/10_nvidia.json` exists (the
  validated camera configuration on a hybrid-GPU laptop).
* VS Code **snap** terminal fix: when `SNAP` is set, restores `XDG_DATA_DIRS` /
  `XDG_CONFIG_DIRS` from their `*_VSCODE_SNAP_ORIG` copies and unsets the snap GTK/GIO
  variables, which otherwise crash rviz2 (libpthread symbol error) and rqt. No-op elsewhere.

## 6. Output on the terminal

* `cube_detector` reports "cube STABLE" every 2 s per cube. In the complete application it
  runs with `detector_output:=own_log` (its own log file in the ROS log directory) so the
  question is not scrolled away; use `detector_output:=screen` to see it.
* `task_prompt` runs with `--disable-stdout-logs` and writes everything meant for the operator
  directly to the terminal (`/dev/tty`), so its lines appear in order. Its ROS log messages
  still go to its log file and `/rosout`.
* `move_group`, `robot_state_publisher`, the bridge and RViz use `output='log'`.

## 7. Why the user confirmation is a node and not a shell script

The original project started the launch from a bash script that waited with a separate
Python readiness checker and then asked with `read -p`. Now a single `ros2 launch` command
starts everything and the question is asked by `task_prompt`, a small ROS 2 node in
`openarm_sorting_manipulation`:

* it uses ROS interfaces for readiness (`/cup_sorting/ready`, the detection topics) instead of
  re-implementing the coordinator's checks,
* it holds no task logic: "y" calls `/cup_sorting/run`, everything else does nothing,
* `ros2 launch` connects child stdout/stdin to pipes (line-buffered, so a question without a
  newline would not even appear), but its children stay in the terminal's session; the node
  therefore talks to `/dev/tty` directly. Without a terminal it refuses to start the task.
