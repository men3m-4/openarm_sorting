"""Pure-Python parts of the manipulation package (no ROS graph needed)."""
import math

from moveit_msgs.msg import CollisionObject
import pytest

from openarm_sorting_manipulation import check_cup_contents, planning_scene as ps
from openarm_sorting_manipulation.moveit_interface import grasp_pose
from openarm_sorting_manipulation.task_prompt import parse_answer


@pytest.mark.parametrize('text,expected', [
    ('y\n', True), ('Y', True), (' y ', True),
    ('n', False), ('N\n', False), ('', False), ('\n', False), (None, False),
    ('yes', None), ('no', None), ('x', None), ('1', None),
])
def test_prompt_answers(text, expected):
    assert parse_answer(text) is expected


def test_grasp_pose_vertical():
    p = grasp_pose((0.3, 0.1, 0.33), yaw=0.0, pitch=0.0, tcp_offset=0.16)
    assert (p.position.x, p.position.y) == pytest.approx((0.3, 0.1))
    assert p.position.z == pytest.approx(0.49)               # hand above the TCP
    assert (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w) == pytest.approx((0, 0, 0, 1))


def test_grasp_pose_pitched_keeps_tcp_distance():
    tcp = (0.3, 0.04, 0.333)
    p = grasp_pose(tcp, yaw=math.radians(10), pitch=math.radians(60), tcp_offset=0.16)
    d = math.dist((p.position.x, p.position.y, p.position.z), tcp)
    assert d == pytest.approx(0.16)
    assert p.position.x < tcp[0]                              # hand trails behind the fingertips


def test_planning_scene_builders():
    box = ps.box('table', 'world', [0.37, 0.05, 0.2875], [0.5, 0.8, 0.025])
    assert box.operation == CollisionObject.ADD and box.primitives[0].dimensions.tolist() == [0.5, 0.8, 0.025]
    att = ps.attach(box, 'openarm_left_ee_base_link', ['a', 'b'])
    scene = ps.scene_with([ps.remove('red_cube', 'world')], attached=[att])
    assert scene.is_diff and scene.robot_state.is_diff
    assert scene.world.collision_objects[0].operation == CollisionObject.REMOVE


def test_installed_scene_file_matches_world_layout():
    scene = check_cup_contents.load_scene(check_cup_contents.default_scene_file())
    assert scene['base'] == [0.36, 0.32, 0.30]
    assert scene['inner_radius'] == 0.055 and scene['cube_size'] == 0.040
