# Command reference

All commands assume:

```bash
source /opt/ros/jazzy/setup.bash
source ~/openarm_sorting_ws_clean/install/setup.bash
cd ~/openarm_sorting_ws_clean
```

## Build and test

| Command | Purpose |
|---|---|
| `rosdep install --from-paths src --ignore-src -r -y` | install dependencies |
| `colcon build --symlink-install` | build all 10 packages |
| `colcon build --symlink-install --packages-select openarm_sorting_bringup` | build one package |
| `colcon test --packages-select openarm_sorting_description openarm_sorting_vision openarm_sorting_manipulation openarm_sorting_bringup && colcon test-result --verbose` | unit tests |
| `python3 tests/integration/e2e_cup_sorting.py --answer y [--gui]` | end-to-end test, full sort |
| `python3 tests/integration/e2e_cup_sorting.py --answer n` | end-to-end test, "no" path |
| `python3 tests/integration/subsystem_launch_test.py [--rviz]` | individual launch files |

## Launch

| Command | Starts |
|---|---|
| `ros2 launch openarm_sorting_bringup cup_sorting.launch.py` | everything + y/N question |
| `ros2 launch openarm_sorting_bringup cup_sorting.launch.py gz_gui:=false rviz:=false` | headless |
| `ros2 launch openarm_sorting_bringup cup_sorting.launch.py auto_start:=true` | everything, sorts by itself |
| `ros2 launch openarm_sorting_bringup cup_sorting.launch.py prompt:=false` | everything, no question |
| `ros2 launch openarm_sorting_bringup simulation.launch.py [gz_gui:=false]` | Gazebo, robot, bridges, controllers |
| `ros2 launch openarm_sorting_bringup moveit.launch.py` | move_group |
| `ros2 launch openarm_sorting_bringup perception.launch.py [output:=own_log]` | cube_detector |
| `ros2 launch openarm_sorting_bringup coordinator.launch.py [prompt:=false] [auto_start:=true]` | coordinator (+ prompt) |
| `ros2 launch openarm_sorting_bringup rviz.launch.py [rviz_config:=camera_view.rviz]` | RViz |
| `ros2 launch openarm_sorting_bringup robot.launch.py` | robot_state_publisher only |
| `ros2 launch openarm_sorting_bringup controllers.launch.py` | controller spawner only |
| `ros2 launch openarm_sorting_bringup image_view.launch.py [image:=<topic>]` | rqt_image_view on a camera topic |
| `ros2 launch openarm_sorting_bringup <file> --show-args` | list a launch file's arguments |

## Task services (`std_srvs/srv/Trigger`)

```bash
ros2 service call /cup_sorting/ready std_srvs/srv/Trigger            # status, no motion
ros2 service call /cup_sorting/run std_srvs/srv/Trigger              # red, then blue
ros2 service call /cup_sorting/pick_place_red std_srvs/srv/Trigger
ros2 service call /cup_sorting/pick_place_blue std_srvs/srv/Trigger
ros2 service call /cup_sorting/plan_check std_srvs/srv/Trigger       # plan only, no motion
ros2 service call /cup_sorting/go_ready std_srvs/srv/Trigger
ros2 service call /cup_sorting/go_home std_srvs/srv/Trigger          # SRDF home (arm hanging)
ros2 service call /cup_sorting/approach_test_red std_srvs/srv/Trigger
ros2 service call /cup_sorting/grasp_test_red std_srvs/srv/Trigger
ros2 service call /cup_sorting/grasp_test_blue std_srvs/srv/Trigger
```

## Inspection

| Command | Shows |
|---|---|
| `ros2 control list_controllers` | controller states |
| `ros2 topic echo --once /cup_sorting/red_cube/pose` | detected red cube pose (world) |
| `ros2 topic echo /cup_sorting/blue_cube/raw` | per-frame blue estimate |
| `ros2 topic hz /openarm_zed/image` | camera rate (~15 Hz) |
| `ros2 launch openarm_sorting_bringup image_view.launch.py` | RGB image (rqt_image_view) |
| `ros2 launch openarm_sorting_bringup image_view.launch.py image:=/openarm_zed/depth_image` | depth image |
| `ros2 launch openarm_sorting_bringup image_view.launch.py image:=/cup_sorting/debug_image` | detections drawn on the image |
| `ros2 param dump /cup_sorting_coordinator` | coordinator parameters in effect |
| `ros2 param dump /cube_detector` | detector parameters in effect |
| `ros2 run tf2_tools view_frames` | TF tree (PDF in the current directory) |
| `gz model --list` | models in the Gazebo world |
| `ros2 run openarm_sorting_vision detection_validator` | detection error vs ground truth (10 s) |
| `ros2 run openarm_sorting_manipulation check_cup_contents` | cubes inside the cup? (ground truth) |

## Robot description

```bash
xacro $(ros2 pkg prefix openarm_sorting_description)/share/openarm_sorting_description/urdf/openarm_v20_gazebo.urdf.xacro > /tmp/openarm_v20.urdf
check_urdf /tmp/openarm_v20.urdf
python3 -c "from openarm_sorting_description.robot_description import gazebo_robot_description as g; print('\n'.join(g()[1]))"
```
