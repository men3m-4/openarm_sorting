"""aruco_detector: camera images -> fiducial marker poses in the camera optical frame.

Camera- and robot-independent. Subscribes (names are parameters, remap freely):
  image_topic        sensor_msgs/Image        colour or mono image
  camera_info_topic  sensor_msgs/CameraInfo   intrinsics K + distortion D (calibrated camera)
  depth_topic        sensor_msgs/Image        OPTIONAL, registered to the image (32FC1 metres or
                                              16UC1 millimetres); empty string = RGB only
Publishes:
  ~/detections       visual_manipulation_interfaces/FiducialDetectionArray
  ~/debug_image      sensor_msgs/Image (annotated, if publish_debug_image)
  TF                 <tf_prefix><id> in the camera optical frame (if publish_tf)

Marker sizes: `marker_ids` + `marker_sizes` (parallel lists); ids not listed use
`default_marker_size` (<= 0: unknown ids are ignored).

Frames: image headers of some drivers / simulators carry a non-optical frame id (Gazebo
stamps `<camera>_link`, x forward). The pose is always computed in the OPTICAL convention
(x right, y down, z forward), so set `camera_frame` to the optical frame id in that case;
empty = use the image header frame id.
"""

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from visual_manipulation_interfaces.msg import FiducialDetection, FiducialDetectionArray

from .aruco_pose import ArucoBackend, estimate_pose, refine_with_depth
from visual_manipulation_core.ros_conversions import rotation_to_quaternion_msg


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector')
        P = self.declare_parameter
        self.dictionary = P('aruco_dictionary', 'DICT_4X4_50').value
        image_topic = P('image_topic', 'image').value
        info_topic = P('camera_info_topic', 'camera_info').value
        depth_topic = P('depth_topic', '').value
        self.camera_frame = P('camera_frame', '').value
        ids = [int(i) for i in P('marker_ids', [0]).value]
        sizes = [float(s) for s in P('marker_sizes', [0.0]).value]
        if len(ids) != len(sizes):
            raise ValueError('marker_ids and marker_sizes must have the same length')
        self.sizes = {i: s for i, s in zip(ids, sizes) if s > 0}
        self.default_size = P('default_marker_size', 0.0).value
        self.use_depth = bool(depth_topic) and P('use_depth', True).value
        self.min_side_px = P('min_side_px', 12.0).value
        self.max_reproj = P('max_reprojection_error', 2.0).value
        self.min_confidence = P('min_confidence', 0.0).value
        self.publish_tf = P('publish_tf', True).value
        self.tf_prefix = P('tf_prefix', 'aruco_').value
        self.publish_debug = P('publish_debug_image', True).value
        self.depth_shrink = P('depth.shrink', 0.8).value
        self.depth_min_points = P('depth.min_points', 30).value
        self.depth_max_plane_rms = P('depth.max_plane_rms', 0.004).value
        self.backend = ArucoBackend(self.dictionary, P('corner_refinement', 'apriltag').value)

        self.bridge = CvBridge()
        self.K = self.D = None
        self.pub = self.create_publisher(FiducialDetectionArray, '~/detections', 10)
        self.debug_pub = self.create_publisher(Image, '~/debug_image', 2)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        if self.use_depth:
            self.sync = ApproximateTimeSynchronizer(
                [Subscriber(self, Image, image_topic, qos_profile=qos_profile_sensor_data),
                 Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)],
                queue_size=5, slop=0.03)
            self.sync.registerCallback(self.on_image)
        else:
            self.create_subscription(Image, image_topic, lambda m: self.on_image(m, None), qos_profile_sensor_data)
        self.get_logger().info(
            f'aruco_detector: {self.dictionary}, image {image_topic}, '
            f'depth {"(" + depth_topic + ")" if self.use_depth else "not used (RGB only)"}, markers {self.sizes}, '
            f'frame {self.camera_frame or "<image header>"}')

    def on_info(self, msg: CameraInfo):
        K = np.array(msg.k, float).reshape(3, 3)
        if K[0, 0] <= 0:
            return
        self.K = K
        self.D = np.array(msg.d, float) if len(msg.d) else np.zeros(5)

    def marker_size(self, marker_id):
        return self.sizes.get(marker_id, self.default_size)

    def on_image(self, img_msg: Image, depth_msg):
        if self.K is None:
            self.get_logger().info('waiting for CameraInfo', throttle_duration_sec=5.0)
            return
        img = self.bridge.imgmsg_to_cv2(img_msg)
        gray = img if img.ndim == 2 else cv2.cvtColor(
            img, cv2.COLOR_RGB2GRAY if img_msg.encoding.lower().startswith('rgb') else cv2.COLOR_BGR2GRAY)
        depth = None
        if depth_msg is not None:
            depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float32)
            if depth_msg.encoding in ('16UC1', 'mono16'):
                depth = depth * 0.001
        frame = self.camera_frame or img_msg.header.frame_id
        out = FiducialDetectionArray()
        out.header = img_msg.header
        out.header.frame_id = frame
        debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if self.publish_debug else None
        corners, ids = self.backend.detect(gray)
        tfs = []
        for c, mid in zip(corners, ids):
            size = self.marker_size(mid)
            if size <= 0:
                continue
            mp = estimate_pose(mid, c, size, self.K, self.D)
            if mp is None or mp.side_length_px < self.min_side_px or mp.reprojection_error > self.max_reproj:
                continue
            if depth is not None:
                refine_with_depth(mp, depth, self.K, self.depth_min_points, self.depth_shrink, self.depth_max_plane_rms)
            if mp.confidence < self.min_confidence:
                continue
            d = FiducialDetection()
            d.marker_id = mid
            d.dictionary = self.dictionary
            d.marker_size = size
            d.pose.header = out.header
            d.pose.pose.position.x, d.pose.pose.position.y, d.pose.pose.position.z = (float(v) for v in mp.t)
            d.pose.pose.orientation = rotation_to_quaternion_msg(mp.R)
            d.confidence = float(mp.confidence)
            d.reprojection_error = float(mp.reprojection_error)
            d.ambiguity_ratio = float(mp.ambiguity_ratio)
            d.side_length_px = float(mp.side_length_px)
            d.view_angle = float(mp.view_angle)
            d.depth_refined = bool(mp.depth_refined)
            d.corners = [float(v) for v in mp.corners.reshape(-1)]
            out.detections.append(d)
            if self.tf_pub is not None:
                t = TransformStamped()
                t.header = out.header
                t.child_frame_id = f'{self.tf_prefix}{mid}'
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = d.pose.pose.position.x, \
                    d.pose.pose.position.y, d.pose.pose.position.z
                t.transform.rotation = d.pose.pose.orientation
                tfs.append(t)
            if debug is not None:
                self.draw(debug, mp)
        self.pub.publish(out)
        if tfs:
            self.tf_pub.sendTransform(tfs)
        if debug is not None:
            dbg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            dbg.header = img_msg.header
            self.debug_pub.publish(dbg)

    def draw(self, img, mp):
        pts = mp.corners.astype(np.int32)
        cv2.polylines(img, [pts], True, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.circle(img, tuple(pts[0]), 3, (0, 0, 255), -1)
        rvec, _ = cv2.Rodrigues(mp.R)
        try:
            cv2.drawFrameAxes(img, self.K, self.D, rvec, mp.t, mp.size * 0.75, 1)
        except AttributeError:  # very old OpenCV
            pass
        u, v = int(mp.corners[:, 0].min()), int(mp.corners[:, 1].min()) - 4
        cv2.putText(img, f'id {mp.marker_id} {mp.t[2]:.2f}m c{mp.confidence:.2f}{" D" if mp.depth_refined else ""}',
                    (max(u, 0), max(v, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
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
