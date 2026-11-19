"""Launch files load, configuration is consistent, helpers behave."""
import glob
import os
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pytest

from openarm_sorting_bringup import launch_support

EXPECTED_CONTROLLERS = ['joint_state_broadcaster', 'left_joint_trajectory_controller',
                        'right_joint_trajectory_controller', 'left_gripper_controller',
                        'right_gripper_controller']


def test_controller_names_from_yaml():
    assert launch_support.controller_names() == EXPECTED_CONTROLLERS


@pytest.mark.parametrize('path', sorted(glob.glob(launch_support.share_path('launch', '*.launch.py'))))
def test_launch_file_loads(path):
    ld = PythonLaunchDescriptionSource(path).get_launch_description(LaunchContext())
    assert ld.entities
    # every argument is documented with a default (no required arguments)
    for arg in ld.entities:
        if isinstance(arg, DeclareLaunchArgument):
            assert arg.default_value is not None, arg.name


def test_world_and_models_installed():
    world = launch_support.share_path('worlds', 'openarm_cup_sorting_world.sdf')
    uris = [u.text for u in ET.parse(world).getroot().iter('uri')]
    for uri in uris:
        name = uri.replace('model://', '')
        assert os.path.isfile(launch_support.share_path('models', name, 'model.sdf')), uri


def test_world_cup_matches_scene_file():
    import yaml
    world = ET.parse(launch_support.share_path('worlds', 'openarm_cup_sorting_world.sdf')).getroot()
    cup = [i for i in world.iter('include') if i.find('name').text == 'coffee_cup'][0]
    x, y, z = (float(v) for v in cup.find('pose').text.split()[:3])
    from ament_index_python.packages import get_package_share_directory
    scene = os.path.join(get_package_share_directory('openarm_sorting_manipulation'), 'config',
                         'cup_sorting_scene.yaml')
    base = yaml.safe_load(open(scene))['cup_sorting_coordinator']['ros__parameters']['cup']['base']
    assert [x, y, z] == pytest.approx(base)


def test_gz_resource_path_is_deduplicated(monkeypatch):
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', '')
    first = launch_support.gz_resource_path()
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', first + ':/extra')
    assert launch_support.gz_resource_path() == first + ':/extra'


def test_nvidia_offload_modes():
    assert launch_support.nvidia_offload_wanted('true') is True
    assert launch_support.nvidia_offload_wanted('false') is False


def test_running_gazebo_servers_returns_list():
    assert isinstance(launch_support.running_gazebo_servers(), list)
