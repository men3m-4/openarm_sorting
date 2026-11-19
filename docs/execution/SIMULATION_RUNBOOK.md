# Simulation runbook

Every terminal first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/openarm_sorting_ws_clean/install/setup.bash
```

## A. Complete application (one command)

```bash
ros2 launch openarm_sorting_bringup cup_sorting.launch.py
```

Opens Gazebo (GUI) and RViz, starts MoveIt, perception and the coordinator, waits until
everything is ready and asks `Do you want to execute the sorting task? [y/N]:`
(see [TASK_EXECUTION.md](TASK_EXECUTION.md)). Stop everything with **Ctrl+C** in the same
terminal.

Variants:

```bash
ros2 launch openarm_sorting_bringup cup_sorting.launch.py gz_gui:=false rviz:=false   # headless
ros2 launch openarm_sorting_bringup cup_sorting.launch.py auto_start:=true            # no question, sorts once ready
ros2 launch openarm_sorting_bringup cup_sorting.launch.py prompt:=false               # no question, trigger by service
ros2 launch openarm_sorting_bringup cup_sorting.launch.py coordinator:=false          # simulation + MoveIt + perception only
ros2 launch openarm_sorting_bringup cup_sorting.launch.py detector_output:=screen     # show detector messages
ros2 launch openarm_sorting_bringup cup_sorting.launch.py nvidia_offload:=false       # do not force the NVIDIA GPU
ros2 launch openarm_sorting_bringup cup_sorting.launch.py --show-args                 # list all arguments
```

### What you should see

| Time (validation machine) | Terminal |
|---|---|
| ~1 s | `[simulation] world: .../openarm_cup_sorting_world.sdf (GUI)` and `[robot] Gazebo gripper collision patch: ... -> 5 boxes` (4x) |
| ~2 s | `[spawn_openarm]: Entity creation successful.` then `[simulation] robot spawned -> loading the controllers` |
| ~7 s | `[controllers] active: joint_state_broadcaster, ...` and `[wait_for_controllers]: all 5 controllers active` |
| ~8 s | `[cup_sorting] controllers active -> starting MoveIt, RViz, perception, coordinator` |
| ~9 s | `[move_group ...] No 3D sensor plugin(s) defined for octomap updates` (expected, harmless) |
| ~15-18 s | `[task_prompt] READY ...`, the banner and the question |

## B. Subsystems in separate terminals (development)

```bash
# terminal 1: Gazebo + robot + bridges + controllers
ros2 launch openarm_sorting_bringup simulation.launch.py            # gz_gui:=false for headless
# terminal 2: MoveIt
ros2 launch openarm_sorting_bringup moveit.launch.py
# terminal 3: perception
ros2 launch openarm_sorting_bringup perception.launch.py
# terminal 4: coordinator (+ the y/N question; prompt:=false to trigger by service)
ros2 launch openarm_sorting_bringup coordinator.launch.py
# terminal 5: RViz (full or camera only)
ros2 launch openarm_sorting_bringup rviz.launch.py
ros2 launch openarm_sorting_bringup rviz.launch.py rviz_config:=camera_view.rviz
```

Start `moveit`, `perception` and `coordinator` once `simulation` reports
`[controllers] active: ...` (or wait with
`ros2 run openarm_sorting_bringup wait_for_controllers joint_state_broadcaster left_joint_trajectory_controller right_joint_trajectory_controller left_gripper_controller right_gripper_controller`).
Stop them in reverse order with Ctrl+C. `robot.launch.py` and `controllers.launch.py` are
included by `simulation.launch.py` and are only started on their own for debugging.

## C. Health checks while it runs

```bash
ros2 control list_controllers                 # 5 controllers, all "active"
ros2 node list                                # includes /move_group /cube_detector /cup_sorting_coordinator
ros2 topic hz /openarm_zed/image              # ~15 Hz
ros2 topic echo --once /cup_sorting/red_cube/pose
ros2 service call /cup_sorting/ready std_srvs/srv/Trigger    # success=True, message='ready'
gz model --list                               # openarm_v20, cup_sorting_table, coffee_cup, red_cube, blue_cube, ground_plane
```

(The warning "nodes in the graph that share an exact name" from `ros2 node list` refers to
MoveIt's internal `move_group` helper nodes and is expected.)

## D. Logs

`ros2 launch` writes every node's log below `~/.ros/log/<date-time-host-pid>/` (printed in the
first line of the launch output). With `detector_output:=own_log` (the default of
`cup_sorting.launch.py`) the cube detector's messages are only there. To keep the logs of a
run together elsewhere:

```bash
ROS_LOG_DIR=~/openarm_sorting_ws_clean/log/runs/$(date +%Y%m%d_%H%M%S) \
    ros2 launch openarm_sorting_bringup cup_sorting.launch.py
```

## E. Stopping

* Ctrl+C in the launch terminal stops every process of the run. If the Gazebo front-end does
  not exit within 5 s, launch escalates and the leftover Gazebo server/GUI of this run is
  stopped automatically (`[simulation] stopped N leftover Gazebo process(es)`).
* Closing the Gazebo window also stops the whole application. Closing RViz does not.
* Only one simulation can run at a time; a second launch refuses to start and lists the
  running Gazebo processes.
