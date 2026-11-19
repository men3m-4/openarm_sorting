# Robot description: `openarm_sorting_description` + upstream `openarm_description`

## What the robot is

OpenArm v2.0 bimanual (robot name `openarm_v20`): torso (`openarm_body_*`), two 7-DOF arms
(`openarm_left_joint1..7`, `openarm_right_joint1..7`), a pinch gripper on each arm
(`openarm_*_finger_joint1`; `finger_joint2` is a URDF mimic), and a ZED Mini on the
`zed_mount` accessory. **Only the left arm and left gripper are used by the task.** The right
arm and gripper are simulated and controlled (their controllers are active) but never moved.

## Files

| File | Purpose |
|---|---|
| `urdf/openarm_v20_gazebo.urdf.xacro` | the robot used in Gazebo **and** by MoveIt |
| `presets/default_bimanual_gazebo.yaml` | Gazebo robot preset (see below) |
| `config/gazebo_controllers.yaml` | gz_ros2_control controllers |
| `openarm_sorting_description/robot_description.py` | Python API used by the launch files |
| `openarm_sorting_description/gripper_collision_patch.py` | Gazebo-only finger box collisions |

### The Xacro

`openarm_v20_gazebo.urdf.xacro` includes `openarm_description/assets/common/robot.xacro`
(for `openarm_assemble_robot`, `openarm_ros2_control`, `openarm_require`) and defines
`openarm_sorting_robot`, a copy of the upstream `openarm_robot` macro whose **only**
difference is that the preset is loaded from an explicit file path (`robot_preset_file`)
instead of `$(find openarm_description)/presets/<id>.yaml`. This keeps the Gazebo preset out
of the upstream clone. It then adds:

* the `gz_ros2_control-system` plugin with `<parameters>$(arg controllers_file)</parameters>`
  (in the original this was the absolute path
  `~/openarm_sorting_ws/simulation_config/openarm_bimanual_gazebo_controllers.yaml`),
* `openarm_zed_sim_camera_link` (3 cm in front of `openarm_body_zed_camera_center`) and
  `openarm_zed_sim_camera_optical_frame` (rpy -pi/2, 0, -pi/2),
* the `rgbd_camera` sensor `openarm_zed_rgbd` on `openarm_body_link0`
  (pose `0.0827288 0.000966585 0.7875494 0 1.082104 0`, HFOV 1.5708, 640x480, 15 Hz,
  clip 0.10-5.0 m, topic prefix `/openarm_zed`, frame `openarm_zed_sim_camera_link`).

Xacro arguments: `controllers_file`, `robot_preset_file`, plus the upstream ones
(`emit_grasp_frame`, `ros2_control`, `use_fake_hardware`, `fake_sensor_commands`,
`left_can_interface`, `right_can_interface`, `can_fd`).

**Equivalence with the original** (validated): the expanded URDF of the new Xacro and of the
original Xacro (with the original in-upstream preset) are identical except for comments and
the controllers file path, both for the MoveIt variant and for the patched Gazebo variant.

### The Gazebo preset

Copy of upstream `presets/v2/default_bimanual.yaml` with two changes:
`systems.*.hardware_plugin: gz_ros2_control/GazeboSimSystem` with command interface
`position` and state interfaces `position, velocity`; and a joint4 limit override for both
arms (`lower: -1e-9, upper: 2.4435, velocity: 5.4454, effort: 27`).

### Controllers (`config/gazebo_controllers.yaml`)

| Controller | Type | Joints |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all |
| `left_joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | `openarm_left_joint1..7` |
| `right_joint_trajectory_controller` | same | `openarm_right_joint1..7` |
| `left_gripper_controller` | `position_controllers/GripperActionController` | `openarm_left_finger_joint1` |
| `right_gripper_controller` | same | `openarm_right_finger_joint1` |

Update rate 100 Hz, position command interface, splines. The trajectory controllers' goal and
trajectory tolerances are 0 (= not checked): a SUCCEEDED result does not prove the arm
arrived, which is why the coordinator checks joint errors itself.

## The finger collision patch

See [../architecture/SYSTEM_ARCHITECTURE.md](../architecture/SYSTEM_ARCHITECTURE.md) §4 for
why. Mechanism (`patch_gripper_collisions`): for every link whose name ends with
`ee_link1` / `ee_link2`, the single collision mesh is replaced by one box per STL part in
`meshes/collision/<finger>/normal_mesh_5parts/` of `openarm_description` (box = axis-aligned
bounding box of the part, URDF mesh scale applied). The launch log shows
`[robot] Gazebo gripper collision patch: openarm_left_ee_link1: mesh collision -> 5 boxes ...`
for all four finger links.

## Python API

```python
from openarm_sorting_description import robot_description as rd
urdf_xml, report = rd.gazebo_robot_description()   # patched, for robot_state_publisher/Gazebo
rd.xacro_file(), rd.xacro_mappings()               # unpatched, for MoveItConfigsBuilder
rd.default_controllers_file()
```

## Useful commands

```bash
# expand the Xacro (MoveIt variant, unpatched)
xacro $(ros2 pkg prefix openarm_sorting_description)/share/openarm_sorting_description/urdf/openarm_v20_gazebo.urdf.xacro > /tmp/openarm_v20.urdf
check_urdf /tmp/openarm_v20.urdf
# TF tree of the running simulation
ros2 run tf2_tools view_frames
```
