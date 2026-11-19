# Ubuntu setup

Target platform (the one everything was validated on):

| Item | Version |
|---|---|
| OS | Ubuntu 24.04 LTS (Noble), validated on 24.04.5 |
| ROS 2 | Jazzy Jalisco (`ros-jazzy-desktop`) |
| Gazebo | Harmonic (gz sim 8.15.0, through `ros-jazzy-ros-gz`) |
| MoveIt 2 | 2.12.4 (Jazzy binaries) |
| Python | 3.12 (system) |
| GPU | any OpenGL 3.3 capable GPU for Gazebo's ogre2 renderer; validated on an NVIDIA RTX 3070 Laptop GPU (driver 580) with PRIME render offload |

## 1. ROS 2 Jazzy

Follow the official instructions (https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
In short:

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

## 2. rosdep

```bash
sudo rosdep init        # only once per machine; skip if /etc/ros/rosdep/sources.list.d exists
rosdep update
```

## 3. Gazebo Harmonic, MoveIt 2, ros2_control

They are installed by `rosdep install` from the package manifests (see
[BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md)). To install them up front instead:

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-moveit ros-jazzy-zed-description ros-jazzy-cv-bridge python3-opencv
```

## 4. Shell

Every new terminal needs the ROS environment (and, after building, the workspace):

```bash
source /opt/ros/jazzy/setup.bash
source ~/openarm_sorting_ws_clean/install/setup.bash
```

Do **not** also source another OpenArm workspace (e.g. the original `~/openarm_sorting_ws`)
in the same terminal: packages with the same name would shadow each other.

## 5. Optional

* `python3-pexpect` for the end-to-end system test (`tests/integration/e2e_cup_sorting.py`):
  `sudo apt install -y python3-pexpect`.
* `ros-jazzy-rqt-image-view` (part of `ros-jazzy-desktop`) to look at camera images.
