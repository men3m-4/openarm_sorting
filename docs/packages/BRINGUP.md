# Bringup: `openarm_sorting_bringup`

Starts the system. Contains the launch files, the Gazebo world and models, the RViz
configurations, the MoveIt runtime overrides and the controller readiness gate. It contains
no application logic. Launch details:
[../architecture/LAUNCH_ARCHITECTURE.md](../architecture/LAUNCH_ARCHITECTURE.md).

## Contents

| Path | Purpose |
|---|---|
| `launch/cup_sorting.launch.py` | complete application (main entry point) |
| `launch/simulation.launch.py` | Gazebo world, robot_state_publisher, bridges, spawn, controllers |
| `launch/robot.launch.py` | robot_state_publisher with the Gazebo description |
| `launch/controllers.launch.py` | spawner for all controllers of the controllers YAML |
| `launch/moveit.launch.py` | move_group |
| `launch/rviz.launch.py` | RViz (`cup_sorting.rviz` or `camera_view.rviz`) |
| `launch/perception.launch.py` | cube_detector |
| `launch/coordinator.launch.py` | coordinator + task_prompt |
| `launch/image_view.launch.py` | rqt_image_view on a camera topic (with the snap terminal fix) |
| `openarm_sorting_bringup/launch_support.py` | helpers used by the launch files |
| `openarm_sorting_bringup/wait_for_controllers.py` | `wait_for_controllers` readiness gate |
| `config/moveit_overrides.yaml` | KDL timeout 0.05 s; execution scaling 2.0, goal margin 1.0 s, start tolerance 0.05 |
| `worlds/openarm_cup_sorting_world.sdf` | world (bullet-featherstone, 2 ms step, ogre2 sensors, lights) |
| `models/cup_sorting_table` | static table, top at z = 0.30 |
| `models/coffee_cup` | static mug: OBJ visual meshes + primitive collisions (floor + 24 wall boxes + handle) |
| `models/red_cube`, `models/blue_cube` | 4 cm, 50 g, mu 1.2 |
| `rviz/cup_sorting.rviz` | MoveIt MotionPlanning + camera image, debug image, depth, point cloud, detection markers |
| `rviz/camera_view.rviz` | camera image, depth image, point cloud, robot model and TF only (no MoveIt) |
| `scripts/generate_coffee_cup.py` | regenerates `models/coffee_cup` in the source tree (not installed) |

## `wait_for_controllers`

```bash
ros2 run openarm_sorting_bringup wait_for_controllers <controller> [...] \
    [--controller-manager /controller_manager] [--timeout 240]
```
Waits for `<manager>/list_controllers`, then polls it until every listed controller is
`active`. Exit 0 = all active, 1 = timeout, 130 = interrupted. `cup_sorting.launch.py` starts
MoveIt, perception and the coordinator from its `OnProcessExit` handler.

## Regenerating the coffee cup

```bash
cd ~/openarm_sorting_ws_clean
python3 src/openarm_sorting_bringup/scripts/generate_coffee_cup.py
colcon build --symlink-install --packages-select openarm_sorting_bringup
```
Validated: the script reproduces the shipped model byte for byte. If a cup dimension changes,
update `openarm_sorting_manipulation/config/cup_sorting_scene.yaml` as well
(`test_world_cup_matches_scene_file` checks the cup position).

## World / scene consistency

The cup pose and size and the table geometry exist in two places: the world/models (what
Gazebo simulates) and `cup_sorting_scene.yaml` (what the coordinator plans with). The unit
test `test_bringup.py::test_world_cup_matches_scene_file` checks that the cup position in the
world equals `cup.base` in the scene file.
