"""Collision objects for the MoveIt planning scene (table, cup, cubes)."""
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


def _pose(x, y, z, qz=0.0, qw=1.0):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.z, p.orientation.w = float(qz), float(qw)
    return p


def box(obj_id, frame, center, size, qz=0.0, qw=1.0):
    o = CollisionObject()
    o.header.frame_id = frame
    o.id = obj_id
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    o.primitives.append(prim)
    o.primitive_poses.append(_pose(*center, qz=qz, qw=qw))
    o.operation = CollisionObject.ADD
    return o


def cylinder(obj_id, frame, center, height, radius):
    o = CollisionObject()
    o.header.frame_id = frame
    o.id = obj_id
    o.primitives.append(SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(height), float(radius)]))
    o.primitive_poses.append(_pose(*center))
    o.operation = CollisionObject.ADD
    return o


def remove(obj_id, frame):
    o = CollisionObject()
    o.header.frame_id = frame
    o.id = obj_id
    o.operation = CollisionObject.REMOVE
    return o


def scene_with(objects=(), attached=()):
    s = PlanningScene()
    s.is_diff = True
    s.robot_state.is_diff = True
    s.world.collision_objects.extend(objects)
    s.robot_state.attached_collision_objects.extend(attached)
    return s


def attach(obj: CollisionObject, link, touch_links):
    a = AttachedCollisionObject()
    a.link_name = link
    a.object = obj
    a.object.operation = CollisionObject.ADD
    a.touch_links = list(touch_links)
    return a


def detach(obj_id, link, frame):
    a = AttachedCollisionObject()
    a.link_name = link
    a.object.id = obj_id
    a.object.header.frame_id = frame
    a.object.operation = CollisionObject.REMOVE
    return a
