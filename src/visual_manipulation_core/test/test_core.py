"""Unit tests of the framework core (no ROS needed)."""
import math

import numpy as np
import pytest

from visual_manipulation_core.se3 import (Transform, average_quaternions, average_transforms, matrix_to_quat,
                                          pose_error, quat_to_matrix, rotation_log, rpy_to_matrix)
from visual_manipulation_core.filters import PoseFilter
from visual_manipulation_core.grasp import GraspSpec, Motion, generate_grasps, flange_from_tcp
from visual_manipulation_core.bayonet import BayonetPlanner, BayonetSpec
from visual_manipulation_core.servo import pbvs_step
from visual_manipulation_core.state_machine import DONE, FAILED, StepFailed, TaskStateMachine

RNG = np.random.default_rng(3)


def rand_tf():
    q = RNG.normal(size=4)
    return Transform.from_xyz_quat(RNG.normal(size=3), q / np.linalg.norm(q))


def test_quat_matrix_roundtrip():
    for _ in range(50):
        q = RNG.normal(size=4)
        q /= np.linalg.norm(q)
        q = -q if q[3] < 0 else q
        assert np.allclose(matrix_to_quat(quat_to_matrix(q)), q, atol=1e-9)


def test_rpy_roundtrip_and_convention():
    T = Transform.from_xyz_rpy([1, 2, 3], [0.1, -0.4, 2.0])
    assert np.allclose(T.rpy(), [0.1, -0.4, 2.0])
    # yaw 90 deg maps x to y
    assert np.allclose(rpy_to_matrix(0, 0, math.pi / 2) @ [1, 0, 0], [0, 1, 0])


def test_inverse_and_composition():
    A, B = rand_tf(), rand_tf()
    assert np.allclose((A @ A.inverse()).matrix(), np.eye(4), atol=1e-12)
    assert np.allclose((A @ B).matrix(), A.matrix() @ B.matrix())
    p = RNG.normal(size=3)
    assert np.allclose(A @ p, A.matrix()[:3, :3] @ p + A.matrix()[:3, 3])


def test_from_flat_and_dict():
    T = Transform.from_flat([0.1, 0.2, 0.3, 0.0, 0.0, math.pi])
    assert np.allclose(T.t, [0.1, 0.2, 0.3])
    T2 = Transform.from_dict(T.to_dict())
    assert pose_error(T, T2)[0] < 1e-6 and pose_error(T, T2)[1] < 1e-5
    with pytest.raises(ValueError):
        Transform.from_flat([1, 2, 3])


def test_average_quaternions_sign_invariant():
    q = matrix_to_quat(rpy_to_matrix(0.2, 0.1, 0.5))
    avg = average_quaternions([q, -q, q])
    assert np.allclose(avg, q, atol=1e-9)


def test_rotation_log():
    R = rpy_to_matrix(0, 0, 0.7)
    assert np.allclose(rotation_log(R), [0, 0, 0.7])


def test_pose_filter_stability():
    f = PoseFilter(window=5, max_age=1.0, position_tolerance=0.002, angle_tolerance=0.02)
    base = Transform.from_xyz_rpy([0.3, 0.1, 0.4], [0, 0, 1.0])
    for k in range(5):
        f.add(k * 0.1, Transform(base.R, base.t + RNG.normal(scale=0.0003, size=3)))
    est = f.estimate(0.45)
    assert est.stable and est.num_samples == 5 and pose_error(est.pose, base)[0] < 0.001
    f.add(0.5, Transform(base.R, base.t + [0.02, 0, 0]))      # object moved
    assert not f.estimate(0.5).stable
    assert f.estimate(5.0) is None                           # everything too old


def test_grasp_generation():
    T_obj = Transform.from_xyz_rpy([0.4, 0.3, 0.4], [0, 0, math.pi])
    spec = GraspSpec('mid', Transform.from_xyz_rpy([0.14, 0, -0.034], [0, -math.pi / 3, math.pi]),
                     approach=Motion((0, 0, -1), 0.08, 'base'), retreat=Motion((0, 0, 1), 0.1, 'base'))
    c = generate_grasps(T_obj, [spec])[0]
    assert np.allclose(c.grasp.t, [0.4 - 0.14, 0.3, 0.4 - 0.034])
    assert np.allclose(c.pregrasp.t - c.grasp.t, [0, 0, 0.08])
    assert np.allclose(c.lift.t - c.grasp.t, [0, 0, 0.1])
    # tool-frame approach: along the tool z axis
    spec_t = GraspSpec('t', spec.object_to_tcp, approach=Motion((0, 0, 1), 0.05, 'tool'))
    ct = generate_grasps(T_obj, [spec_t])[0]
    assert np.allclose(ct.pregrasp.t, ct.grasp.t - 0.05 * ct.grasp.R[:, 2])
    tool = Transform.translation(0, 0, -0.16)
    fl = flange_from_tcp(c.grasp, tool)
    assert np.allclose((fl @ tool).t, c.grasp.t)


def bayonet():
    return BayonetPlanner(BayonetSpec(Transform.translation(0, 0, 0.019), math.radians(45), 0.045,
                                      (0.1, 0, -0.01), math.radians(5)))


def test_bayonet_sequence_geometry():
    T_B_M = Transform.from_xyz_rpy([0.455, 0.08, 0.45], [0, 0, math.pi])
    b = bayonet()
    locked = b.locked(T_B_M)
    assert np.allclose(locked.t, [0.455, 0.08, 0.469])
    # locked: object x along mount x = world -x
    assert np.allclose(locked.R[:, 0], [-1, 0, 0], atol=1e-9)
    pre = b.pre_insert(T_B_M)
    assert np.allclose(pre.t, [0.455, 0.08, 0.469 - 0.045])
    ins = b.inserted(T_B_M)
    # inserted heading = locked heading - 45 deg (about +z)
    assert math.isclose(math.atan2(ins.R[1, 0], ins.R[0, 0]), math.pi - math.radians(45), abs_tol=1e-9)
    st = b.staging(T_B_M)
    assert np.allclose(st.t, pre.t + [-0.1, 0, -0.01])          # +x_M = -x_world
    rot = b.rotation_path(T_B_M, lock=True)
    assert len(rot) == 9 and pose_error(rot[-1], locked)[1] < 1e-9
    for p in rot:                                               # the axis point never moves
        assert np.allclose(p.t, locked.t)
    unl = b.rotation_path(T_B_M, lock=False)
    assert pose_error(unl[-1], ins)[1] < 1e-9
    assert pose_error(b.insertion_path(T_B_M)[-1], ins)[0] < 1e-12
    assert pose_error(b.extraction_path(T_B_M)[-1], pre)[0] < 1e-12


def test_bayonet_error_and_classify():
    T_B_M = Transform.from_xyz_rpy([0.455, 0.08, 0.45], [0, 0, math.pi])
    b = bayonet()
    assert b.classify(T_B_M, b.locked(T_B_M)) == 'locked'
    assert b.classify(T_B_M, b.inserted(T_B_M)) == 'inserted'
    # just below / beside the mount is NOT 'out' (must not be treated as free to move)
    assert b.classify(T_B_M, b.pre_insert(T_B_M)) == 'misaligned'
    assert b.classify(T_B_M, Transform.translation(0, 0, -0.06) @ b.locked(T_B_M)) == 'out'
    assert b.classify(T_B_M, Transform.translation(0.2, 0, 0) @ b.locked(T_B_M)) == 'out'
    # released part hanging off-centre and tilted within the clearances: still locked
    hang = b.locked(T_B_M) @ Transform.from_xyz_rpy([0.0045, 0, -0.001], [0, math.radians(7), 0])
    assert b.classify(T_B_M, hang) == 'locked'
    off = Transform.translation(0.002, 0, 0) @ b.locked(T_B_M) @ Transform.rotation_about_z(math.radians(3))
    e = b.error(T_B_M, off)
    assert math.isclose(e['radial'], 0.002, abs_tol=1e-9)
    assert math.isclose(e['angle'], math.radians(3), abs_tol=1e-9)


def test_pbvs_converges_with_rigid_grasp():
    T_P_tcp = Transform.from_xyz_rpy([0.14, 0, -0.03], [0, -1.0, math.pi])
    T_des = Transform.from_xyz_rpy([0.45, 0.1, 0.42], [0, 0, 2.4])
    T_obj = Transform.from_xyz_rpy([0.47, 0.08, 0.41], [0.02, 0, 2.3])
    tcp = T_obj @ T_P_tcp
    for _ in range(10):
        r = pbvs_step(tcp, T_obj, T_des, gain=0.8, max_translation=0.02)
        if r.converged:
            break
        tcp = r.target_tcp
        T_obj = tcp @ T_P_tcp.inverse()
    assert r.converged


def test_pbvs_no_tilt_keeps_tilt():
    T_des = Transform.from_xyz_rpy([0.45, 0.1, 0.42], [0, 0, 2.4])
    T_obj = Transform.from_xyz_rpy([0.45, 0.1, 0.42], [0.05, 0, 2.3])
    tcp = T_obj @ Transform.translation(0.1, 0, 0)
    r = pbvs_step(tcp, T_obj, T_des, gain=1.0, max_rotation=1.0, correct_tilt=False)
    new_obj = r.target_tcp @ Transform.translation(0.1, 0, 0).inverse()
    assert np.allclose(new_obj.rpy()[0], 0.05, atol=1e-6)     # tilt unchanged
    assert math.isclose(new_obj.rpy()[2], 2.4, abs_tol=1e-6)


def test_state_machine_retry_and_recovery():
    calls = []

    def flaky(ctx):
        calls.append('A')
        if len(calls) < 2:
            raise StepFailed('first try fails')
        return 'B'

    def broken(ctx):
        raise StepFailed('always')

    sm = TaskStateMachine('t')
    sm.add_state('A', flaky, max_attempts=3)
    sm.add_state('B', broken, max_attempts=2, on_failure='RECOVER')
    sm.add_state('RECOVER', lambda ctx: FAILED)
    res = sm.run(None, 'A')
    assert not res.success and res.final_state == FAILED
    assert [t.state for t in res.history] == ['A', 'A', 'B', 'B', 'RECOVER']
    assert 'B: always' in res.message

    sm2 = TaskStateMachine('t2')
    sm2.add_state('X', lambda ctx: DONE)
    assert sm2.run(None, 'X').success


def test_state_machine_unknown_reference():
    sm = TaskStateMachine('t')
    sm.add_state('A', lambda c: DONE, on_failure='NOPE')
    with pytest.raises(ValueError):
        sm.run(None, 'A')
