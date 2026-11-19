"""The Gazebo robot description expands and the finger collision patch is applied."""
import xml.etree.ElementTree as ET

from openarm_sorting_description import robot_description


def _urdf():
    urdf, report = robot_description.gazebo_robot_description()
    return ET.fromstring(urdf), report


def test_xacro_expands_with_camera_frames():
    root, _ = _urdf()
    assert root.get('name') == 'openarm_v20'
    links = {link.get('name') for link in root.findall('link')}
    assert {'openarm_zed_sim_camera_link', 'openarm_zed_sim_camera_optical_frame'} <= links
    joints = {j.get('name'): j for j in root.findall('joint')}
    assert all(f'openarm_left_joint{i}' in joints for i in range(1, 8))


def test_gz_ros2_control_uses_installed_controllers_file():
    root, _ = _urdf()
    params = [p.text for p in root.iter('parameters')]
    assert params == [robot_description.default_controllers_file()]
    assert all(h.text == 'gz_ros2_control/GazeboSimSystem' for h in root.iter('plugin')
               if h.text and 'System' in h.text)


def test_finger_collisions_replaced_by_boxes():
    root, report = _urdf()
    fingers = [link for link in root.findall('link')
               if link.get('name', '').endswith(('ee_link1', 'ee_link2'))]
    assert len(fingers) == 4                              # 2 fingers x 2 grippers
    for link in fingers:
        cols = link.findall('collision')
        assert len(cols) == 5, link.get('name')
        assert all(c.find('geometry/box') is not None for c in cols)
    assert all('5 boxes' in line for line in report)


def test_joint4_limit_override_from_gazebo_preset():
    root, _ = _urdf()
    lim = [j for j in root.findall('joint') if j.get('name') == 'openarm_left_joint4'][0].find('limit')
    assert abs(float(lim.get('upper')) - 2.4435) < 1e-6
