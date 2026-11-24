"""fiducial_object_tracker: marker detections -> filtered OBJECT poses in a reference frame.

For every configured object:
    T_ref_object = T_ref_camera(t_image) @ T_camera_marker @ T_marker_object
  * T_ref_camera from TF2 at the image time stamp (works for fixed and moving cameras,
    e.g. eye-to-hand or eye-in-hand, as long as TF knows the camera),
  * T_camera_marker from the detector,
  * T_marker_object = configured `objects.<name>.marker_to_object` [x y z roll pitch yaw]
    (the marker is NOT the object frame and NOT the grasp frame).
A temporal filter (visual_manipulation_core.filters.PoseFilter) averages the samples and
reports STABLE only if all recent samples agree within tolerance.

Subscribes: detections_topic (visual_manipulation_interfaces/FiducialDetectionArray)
Publishes:  ~/objects          visual_manipulation_interfaces/TrackedObjectArray (every cycle)
            ~/<name>/pose      geometry_msgs/PoseStamped (filtered, only while STABLE)
            TF  reference_frame -> <name> (filtered, only while STABLE; if publish_tf)
Services:   ~/reset            std_srvs/Trigger (forget all samples)
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from visual_manipulation_core.filters import PoseFilter
from visual_manipulation_core.ros_conversions import pose_from_transform, tf_from_transform, transform_from_pose, \
    transform_from_tf
from visual_manipulation_core.se3 import Transform
from visual_manipulation_interfaces.msg import FiducialDetectionArray, TrackedObject, TrackedObjectArray


class ObjectTrackerNode(Node):

    def __init__(self):
        super().__init__('fiducial_object_tracker')
        P = self.declare_parameter
        self.ref = P('reference_frame', 'world').value
        topic = P('detections_topic', 'aruco_detector/detections').value
        names = list(P('tracked_objects', ['object']).value)
        self.min_conf = P('min_confidence', 0.0).value
        self.require_depth = P('require_depth_refined', False).value
        self.publish_tf = P('publish_tf', True).value
        self.tf_timeout = P('tf_timeout', 0.2).value
        fw = P('filter.window', 8).value
        fa = P('filter.max_age', 1.5).value
        fp = P('filter.position_tolerance', 0.004).value
        fang = P('filter.angle_tolerance', 0.05).value
        self.objects = {}
        for n in names:
            mid = P(f'objects.{n}.marker_id', -1).value
            m2o = P(f'objects.{n}.marker_to_object', [0.0] * 6).value
            if mid < 0:
                raise ValueError(f'objects.{n}.marker_id is not configured')
            self.objects[n] = {'marker_id': int(mid), 'T_mk_obj': Transform.from_flat(m2o),
                               'filter': PoseFilter(fw, fa, fp, fang),
                               'pose_pub': self.create_publisher(PoseStamped, f'~/{n}/pose', 10),
                               'last_conf': 0.0, 'last_seen': None}
        self.by_marker = {o['marker_id']: n for n, o in self.objects.items()}
        if len(self.by_marker) != len(self.objects):
            raise ValueError('two tracked objects use the same marker id')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.array_pub = self.create_publisher(TrackedObjectArray, '~/objects', 10)
        self.create_subscription(FiducialDetectionArray, topic, self.on_detections, 10)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.get_logger().info(f'tracking {[(n, o["marker_id"]) for n, o in self.objects.items()]} in {self.ref}, '
                               f'detections from {topic}')

    def on_reset(self, _req, res):
        for o in self.objects.values():
            o['filter'].reset()
        res.success, res.message = True, 'filters reset'
        return res

    def on_detections(self, msg: FiducialDetectionArray):
        stamp = Time.from_msg(msg.header.stamp)
        now_s = stamp.nanoseconds * 1e-9
        T_ref_cam = None
        for d in msg.detections:
            name = self.by_marker.get(d.marker_id)
            if name is None or d.confidence < self.min_conf or (self.require_depth and not d.depth_refined):
                continue
            if T_ref_cam is None:
                try:
                    tf = self.tf_buffer.lookup_transform(self.ref, d.pose.header.frame_id, stamp,
                                                         timeout=Duration(seconds=self.tf_timeout))
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'no TF {self.ref} <- {d.pose.header.frame_id}: {exc}',
                                           throttle_duration_sec=5.0)
                    return
                T_ref_cam = transform_from_tf(tf)
            o = self.objects[name]
            T = T_ref_cam @ transform_from_pose(d.pose.pose) @ o['T_mk_obj']
            o['filter'].add(now_s, T, d.confidence)
            o['last_conf'] = d.confidence
            o['last_seen'] = msg.header.stamp

        out = TrackedObjectArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.ref
        tfs = []
        for name, o in self.objects.items():
            est = o['filter'].estimate(now_s)
            if est is None:
                continue
            t = TrackedObject()
            t.name, t.marker_id = name, o['marker_id']
            t.pose.header = out.header
            t.pose.pose = pose_from_transform(est.pose)
            t.confidence, t.stable, t.num_samples = float(o['last_conf']), bool(est.stable), int(est.num_samples)
            t.position_spread, t.angular_spread = float(est.position_spread), float(est.angular_spread)
            if o['last_seen'] is not None:
                t.last_seen = o['last_seen']
            out.objects.append(t)
            if est.stable:
                o['pose_pub'].publish(t.pose)
                if self.tf_pub is not None:
                    tfs.append(tf_from_transform(est.pose, self.ref, name, msg.header.stamp))
        self.array_pub.publish(out)
        if tfs:
            self.tf_pub.sendTransform(tfs)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
