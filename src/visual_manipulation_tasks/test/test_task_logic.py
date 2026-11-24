"""Pure-logic parts of the portafilter task (no running ROS graph needed)."""
import math

from visual_manipulation_core.se3 import Transform
from visual_manipulation_tasks.portafilter_task import PortafilterTask
from visual_manipulation_tasks.task_prompt import parse_answer


def test_servo_error_decomposition():
    T_des = Transform.from_xyz_rpy([0.4, 0.1, 0.42], [0, 0, 2.0])
    T = Transform.from_xyz_rpy([0.403, 0.1, 0.425], [0, math.radians(2), 2.0 + math.radians(1)])
    e = PortafilterTask.servo_error(T, T_des, [0, 0, 1])
    assert math.isclose(e['radial'], 0.003, abs_tol=1e-9)
    assert math.isclose(e['axial'], 0.005, abs_tol=1e-9)
    assert math.isclose(e['angle'], math.radians(1), abs_tol=1e-3)
    assert math.isclose(e['tilt'], math.radians(2), abs_tol=1e-3)


def test_prompt_answers():
    assert parse_answer('y') is True and parse_answer('Y\n') is True
    assert parse_answer('') is False and parse_answer('n') is False and parse_answer(None) is False
    assert parse_answer('yes') is None
