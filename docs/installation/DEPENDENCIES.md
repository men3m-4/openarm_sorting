# Dependencies

All dependencies are declared in the `package.xml` files and are installed with
`rosdep install --from-paths src --ignore-src -y` (see [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md)).
`rosdep check --from-paths src --ignore-src` reports "All system dependencies have been
satisfied" on the validation machine.

## Source dependencies (in this workspace)

| Repository | Path | Commit | Packages |
|---|---|---|---|
| https://github.com/enactic/openarm_ros2 | `src/upstream/openarm_ros2` | `8f23aeabb372c256f7807b9a74d48e035fc68386` | `openarm`, `openarm_bimanual_moveit_config`, `openarm_bringup`, `openarm_hardware` |
| https://github.com/enactic/openarm_description | `src/upstream/openarm_description` | `8dbfcc93419f26c41731c3c98c2a46a82e718e65` | `openarm_description` |
| https://github.com/enactic/openarm_can | `src/upstream/openarm_can` | `26b266ae90c36ead9f84dcba2d747c4762a94c18` | `openarm_can` (plain CMake) |

They are listed in `upstream.repos` and used unmodified.

## System dependencies of the project packages (apt, resolved by rosdep)

| Package | Key dependencies |
|---|---|
| `openarm_sorting_description` | `ament_index_python`, `openarm_description`, `xacro`, `python3-numpy`, `gz_ros2_control`, `joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_controllers` |
| `openarm_sorting_vision` | `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `cv_bridge`, `message_filters`, `tf2_ros`, `python3-opencv`, `python3-numpy`, `gz_transport_vendor` + `gz_msgs_vendor` (Gazebo Python bindings for the ground-truth test tools) |
| `openarm_sorting_manipulation` | `rclpy`, `ament_index_python`, `builtin_interfaces`, `std_msgs`, `std_srvs`, `geometry_msgs`, `sensor_msgs`, `shape_msgs`, `moveit_msgs`, `control_msgs`, `controller_manager_msgs`, `tf2_ros`, `python3-numpy`, `python3-yaml`, `openarm_sorting_vision` |
| `openarm_sorting_bringup` | `launch`, `launch_ros`, `ament_index_python`, `python3-yaml`, `rclpy`, `controller_manager_msgs`, the three project packages, `openarm_description`, `openarm_bimanual_moveit_config`, `zed_description`, `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `controller_manager`, `moveit_configs_utils`, `moveit_ros_move_group`, `moveit_kinematics`, `moveit_planners_ompl`, `moveit_simple_controller_manager`, `moveit_ros_visualization`, `rviz2` |
| all four (tests) | `python3-pytest` |

Resolved apt packages for the four project packages (`rosdep resolve`):
`python3-numpy python3-opencv python3-pytest python3-yaml ros-jazzy-ament-index-python
ros-jazzy-builtin-interfaces ros-jazzy-controller-manager ros-jazzy-controller-manager-msgs
ros-jazzy-control-msgs ros-jazzy-cv-bridge ros-jazzy-geometry-msgs ros-jazzy-gripper-controllers
ros-jazzy-gz-msgs-vendor ros-jazzy-gz-ros2-control ros-jazzy-gz-transport-vendor
ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller ros-jazzy-launch
ros-jazzy-launch-ros ros-jazzy-message-filters ros-jazzy-moveit-configs-utils
ros-jazzy-moveit-kinematics ros-jazzy-moveit-msgs ros-jazzy-moveit-planners-ompl
ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-ros-visualization
ros-jazzy-moveit-simple-controller-manager ros-jazzy-rclpy ros-jazzy-robot-state-publisher
ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-rviz2 ros-jazzy-sensor-msgs
ros-jazzy-shape-msgs ros-jazzy-std-msgs ros-jazzy-std-srvs ros-jazzy-tf2-ros
ros-jazzy-visualization-msgs ros-jazzy-xacro ros-jazzy-zed-description`

The upstream packages add (among others) `cli11` (`libcli11-dev`, for `openarm_can`),
`hardware_interface`, `pluginlib`, `joint_state_publisher(_gui)`, `ros_gz`.

## Versions on the validation machine

| Package | Version |
|---|---|
| ros-jazzy-desktop | 0.11.0 |
| ros-jazzy-moveit-ros-move-group | 2.12.4 |
| ros-jazzy-ros-gz / ros-gz-sim | 1.0.25 |
| Gazebo (gz sim) | 8.15.0 |
| ros-jazzy-gz-ros2-control | 1.2.20 |
| ros-jazzy-ros2-controllers | 4.43.1 |
| ros-jazzy-rviz2 | 14.1.24 |
| ros-jazzy-xacro | 2.1.1 |
| ros-jazzy-zed-description | 0.2.0 |
| ros-jazzy-cv-bridge | 4.1.0 |
| OpenCV (python3-opencv) | 4.6.0 |
| NumPy | 1.26.4 |
| libcli11-dev | 2.4.1 |

## Not required

* **ZED SDK / ZED ROS 2 wrapper / CUDA**: not used (they cannot consume a Gazebo sensor);
  only `zed_description` (meshes) is needed.
* **Neural-network frameworks** (PyTorch, YOLO, ...): not used; detection is OpenCV only.
* **moveit_py**: not used; the coordinator talks to move_group's ROS interfaces.
* **CAN hardware / `can-utils`**: `openarm_can` and `openarm_hardware` are built because they
  are part of the upstream repositories, but the simulation uses `gz_ros2_control`.
