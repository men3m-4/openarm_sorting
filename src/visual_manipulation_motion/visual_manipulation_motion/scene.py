"""Planning-scene objects from declarative shape descriptions.

A shape is a dict {type, pose: [x y z roll pitch yaw] in the OBJECT frame, dimensions}:
  box       dimensions [sx, sy, sz]
  cylinder  dimensions [height, radius]          (axis = object-shape z)
  ring      dimensions [height, r_in, r_out, segments, gap_deg]  annulus of boxes, optional
            opening of gap_deg centred on the shape's +x axis
Objects are placed with a measured object pose T_frame_object, so the same description works
wherever the camera finds the object.
"""
import math
from typing import Dict, Iterable, List

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

from visual_manipulation_core.ros_conversions import pose_from_transform
from visual_manipulation_core.se3 import Transform, rot_z


def _primitives(shape: Dict, T: Transform):
    """[(SolidPrimitive, Transform)] of one shape placed at T (frame <- shape)."""
    kind = shape['type']
    dims = [float(v) for v in shape['dimensions']]
    T_s = T @ Transform.from_flat(shape.get('pose', [0.0] * 6))
    if kind == 'box':
        return [(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dims[:3]), T_s)]
    if kind == 'cylinder':
        return [(SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=dims[:2]), T_s)]
    if kind == 'ring':
        h, r_in, r_out = dims[:3]
        n = int(dims[3]) if len(dims) > 3 else 16
        gap = math.radians(dims[4]) if len(dims) > 4 else 0.0
        out = []
        dth = 2 * math.pi / n
        chord = 2 * r_out * math.tan(dth / 2) * 1.02
        rc = (r_in + r_out) / 2
        for k in range(n):
            th = k * dth
            if gap > 0 and abs((th + math.pi) % (2 * math.pi) - math.pi) < gap / 2 + dth / 2:
                continue
            seg = T_s @ Transform(rot_z(th), (rc * math.cos(th), rc * math.sin(th), 0.0))
            out.append((SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[r_out - r_in, chord, h]), seg))
        return out
    raise ValueError(f'unknown shape type {kind!r}')


def collision_object(obj_id: str, frame: str, T_frame_obj: Transform, shapes: Iterable[Dict]) -> CollisionObject:
    o = CollisionObject()
    o.header.frame_id = frame
    o.id = obj_id
    o.pose = pose_from_transform(Transform())
    for s in shapes:
        for prim, T in _primitives(s, T_frame_obj):
            o.primitives.append(prim)
            o.primitive_poses.append(pose_from_transform(T))
    o.operation = CollisionObject.ADD
    return o


def box(obj_id, frame, center, size) -> CollisionObject:
    return collision_object(obj_id, frame, Transform.translation(*center),
                            [{'type': 'box', 'dimensions': list(size)}])


def remove(obj_id: str, frame: str) -> CollisionObject:
    o = CollisionObject()
    o.header.frame_id = frame
    o.id = obj_id
    o.operation = CollisionObject.REMOVE
    return o


def attach(obj: CollisionObject, link: str, touch_links: List[str]) -> AttachedCollisionObject:
    a = AttachedCollisionObject()
    a.link_name = link
    a.object = obj
    a.object.operation = CollisionObject.ADD
    a.touch_links = list(touch_links)
    return a


def detach(obj_id: str, link: str, frame: str) -> AttachedCollisionObject:
    a = AttachedCollisionObject()
    a.link_name = link
    a.object.id = obj_id
    a.object.header.frame_id = frame
    a.object.operation = CollisionObject.REMOVE
    return a


def scene_diff(objects=(), attached=()) -> PlanningScene:
    s = PlanningScene()
    s.is_diff = True
    s.robot_state.is_diff = True
    s.world.collision_objects.extend(objects)
    s.robot_state.attached_collision_objects.extend(attached)
    return s


def shapes_from_parameters(node, prefix: str) -> List[Dict]:
    """Read `<prefix>.shape_names` and `<prefix>.shapes.<name>.{type,pose,dimensions}` ROS
    parameters (declaring them) into shape dicts."""
    names = node.declare_parameter(f'{prefix}.shape_names', ['']).value
    out = []
    for n in names:
        if not n:
            continue
        out.append({
            'name': n,
            'type': node.declare_parameter(f'{prefix}.shapes.{n}.type', 'box').value,
            'pose': list(node.declare_parameter(f'{prefix}.shapes.{n}.pose', [0.0] * 6).value),
            'dimensions': list(node.declare_parameter(f'{prefix}.shapes.{n}.dimensions', [0.0]).value),
        })
    return out
