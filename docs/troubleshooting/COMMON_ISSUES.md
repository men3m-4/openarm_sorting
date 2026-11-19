# Common issues

| Symptom | Cause / check | Fix |
|---|---|---|
| `[cup_sorting] ERROR: a Gazebo simulation is already running (it owns /controller_manager)` | another Gazebo server of your user is running (listed below the message) | stop it (Ctrl+C in its terminal); if it is an orphan: `pkill -f "gz sim"` |
| `Package 'openarm_sorting_bringup' not found` | workspace not built or not sourced | `colcon build --symlink-install` and `source ~/openarm_sorting_ws_clean/install/setup.bash` |
| Packages resolve to the wrong workspace (`ros2 pkg prefix openarm_description` does not point into `openarm_sorting_ws_clean`) | another OpenArm workspace is sourced in the same terminal | open a new terminal and source only `/opt/ros/jazzy` and this workspace |
| `[cup_sorting] ERROR: the controllers did not become active` | Gazebo did not start the controller manager within `controllers_timeout` (240 s) | look for Gazebo errors above; check that `gz_ros2_control` is installed; `ros2 control list_controllers` |
| `[simulation] ERROR: spawning the robot failed` | `ros_gz_sim create` could not create the entity | check that robot_state_publisher published `/robot_description`; Gazebo errors above |
| Robot invisible / mesh errors in Gazebo (`Unable to find uri[package://...]`) | `GZ_SIM_RESOURCE_PATH` does not contain the parent of the `openarm_description` / `zed_description` share directories | launch through `simulation.launch.py` (it sets the path); do not override `GZ_SIM_RESOURCE_PATH` afterwards |
| Camera image grey, depth all `-inf` | the sensor is inside the ZED housing | keep the sensor pose of the Xacro (3 cm in front of the camera centre) |
| No camera images at all / `ros2 topic hz /openarm_zed/image` silent | Gazebo paused, or sensors not rendering | the launch starts the world running (`-r`); on hybrid-GPU laptops try `nvidia_offload:=true`, on machines without NVIDIA `nvidia_offload:=false` |
| Second gripper finger does not move | world not using bullet-featherstone (DART cannot do mimic joints) | use the provided world; never add `finger_joint2` to ros2_control |
| Gripper does not close around the cube / grasp fails at ~0.0 rad | finger collision patch not applied | the launch log must show `[robot] Gazebo gripper collision patch: ... -> 5 boxes` for 4 links |
| `[task_prompt] waiting for: coordinator (waiting for: ...)` for a long time | the coordinator is missing one of its inputs | the message lists what is missing (move_group interfaces / active controllers / left gripper / joint limits) |
| `[task_prompt] waiting for: red cube detection` | detector not running or not seeing the cube | `perception:=true`; `ros2 launch openarm_sorting_bringup image_view.launch.py image:=/cup_sorting/debug_image`; was the cube moved out of view? |
| `[task_prompt] ERROR: no interactive terminal ...` | launch started without a terminal (script, IDE task, `nohup`) | run it in a terminal, or use `auto_start:=true`, or `prompt:=false` + `ros2 service call /cup_sorting/run std_srvs/srv/Trigger` |
| `/cup_sorting/run` returns `busy: another sequence is running` | a motion service is still executing | wait for it to finish |
| `/cup_sorting/run` returns `not ready (see coordinator log)` | coordinator readiness failed within 5 s | `ros2 service call /cup_sorting/ready std_srvs/srv/Trigger` tells what is missing |
| `... arm did not reach the planned goal (max joint error ... > 0.03)` | the arm was blocked (collision) or Gazebo lagged | the coordinator recovers to the ready pose; check the scene in RViz; rerun |
| `grasp NOT verified: finger angle ... outside [0.12, 0.32]` | the cube was missed (finger closed to ~0) or something larger was grasped | check the detection (debug image) and that the cube was not pushed |
| `No 3D sensor plugin(s) defined for octomap updates` (move_group, ERROR) | no octomap is configured | expected, harmless |
| `/recognize_objects not available` (RViz) / one-time `No robot state or robot model loaded` | optional server / startup order | expected, harmless |
| `nodes in the graph that share an exact name` from `ros2 node list` | MoveIt's internal `move_group` helper nodes | expected, harmless |
| `move_group failed to terminate '5' seconds after receiving 'SIGINT'` on Ctrl+C | MoveIt teardown is slow; launch escalates to SIGTERM | harmless |
| `gazebo-1 failed to terminate ... escalating to 'SIGTERM'` then `[simulation] stopped 1 leftover Gazebo process(es)` | the gz front-end was slow to exit; its server was stopped by the launch | harmless; this is the cleanup working |
| RViz crashes immediately with a `/snap/core20/.../libpthread` symbol error, or rqt aborts with a GLib-GIO schema error | terminal inside the VS Code **snap** | the launch files fix the environment for their children automatically; use `ros2 launch openarm_sorting_bringup image_view.launch.py image:=<topic>` instead of `ros2 run rqt_image_view ...` |
| RViz shows nothing / wrong fixed frame | `cup_sorting.rviz` uses `world` | wait until robot_state_publisher runs; check `ros2 run tf2_tools view_frames` |
| Build: `openarm_can` / `openarm_hardware` compiler warnings | upstream code | harmless |
| `rosdep`: `Cannot locate rosdep definition for [ament_python]` | (only in the original workspace's manifests) | fixed in this workspace |

## Logs

The first line of every `ros2 launch` prints the log directory
(`~/.ros/log/<date>-<host>-<pid>/`). The cube detector writes there only (in the complete
application). Coordinator steps are printed on the terminal and logged there.

## Getting a clean state

```bash
pkill -f "ros2 launch openarm_sorting_bringup"; pkill -f "gz sim"     # only if something hangs
cd ~/openarm_sorting_ws_clean && rm -rf build install log && colcon build --symlink-install
```
