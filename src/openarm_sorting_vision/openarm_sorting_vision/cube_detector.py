"""Red / blue cube detector for the OpenArm cup-sorting simulation.

Inputs (simulated ZED Mini RGB-D sensor, Gazebo; topic names are parameters):
  /openarm_zed/image         sensor_msgs/Image  rgb8                        (image_topic)
  /openarm_zed/depth_image   sensor_msgs/Image  32FC1, metres, z-depth,     (depth_topic)
                             registered to RGB
  /openarm_zed/camera_info   sensor_msgs/CameraInfo                         (camera_info_topic)

Outputs (frame `world`, the MoveIt planning frame):
  /cup_sorting/red_cube/pose,  /cup_sorting/blue_cube/pose   geometry_msgs/PoseStamped
        cube CENTRE + yaw, published only when the estimate is STABLE
  /cup_sorting/red_cube/raw,   /cup_sorting/blue_cube/raw    geometry_msgs/PointStamped
        unfiltered per-frame estimate (diagnostics)
  /cup_sorting/debug_image                                   sensor_msgs/Image (rgb8)
  /cup_sorting/detection_markers                             visualization_msgs/MarkerArray

All tuning values are ROS parameters (defaults = the validated values, also listed in
config/cube_detector.yaml of this package).

Pipeline per synchronized RGB + depth frame:
  1. HSV segmentation (color_segmentation.py) -> largest compact blob per colour
  2. erode the blob (drops edge pixels whose depth mixes cube and background)
  3. keep finite depths inside [min_depth, max_depth]; require enough valid pixels
  4. back-project with K into the optical frame, transform to `world` with TF
  5. top-face points = points within `top_band` of the highest points; cube centre
     = centre of their min-area rectangle, z = top - cube_size / 2; yaw from the rectangle
  6. sanity checks (top-face size close to the cube size)
  7. temporal filter: publish the median of the last N estimates only if they all lie
     within `stability_tol` of it (rejects noisy / partially occluded detections)
"""
import math
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

from .color_segmentation import COLOR_SPECS, color_mask, find_blobs, flat_hue_ranges, make_spec
from . import camera_geometry as cg

DRAW_COLOR = {'red': (255, 60, 60), 'blue': (60, 120, 255)}


class CubeDetector(Node):

    def __init__(self):
        super().__init__('cube_detector')
        p = self.declare_parameter
        self.target_frame = p('target_frame', 'world').value
        self.optical_frame = p('optical_frame', 'openarm_zed_sim_camera_optical_frame').value
        self.colors = list(p('colors', ['red', 'blue']).value)
        self.cube_size = p('cube_size', 0.040).value
        self.min_area = p('min_area_px', 60).value
        self.max_area = p('max_area_px', 20000).value
        self.min_fill = p('min_fill_ratio', 0.55).value
        self.erode_px = p('erode_px', 2).value
        self.min_depth = p('min_depth', 0.15).value
        self.max_depth = p('max_depth', 2.0).value
        self.min_valid_px = p('min_valid_depth_px', 25).value
        self.top_band = p('top_band', 0.006).value
        self.size_tol = p('size_tolerance', 0.012).value
        self.window = p('stability_window', 6).value
        self.window_sec = p('stability_window_sec', 1.5).value
        self.stab_tol = p('stability_tolerance', 0.004).value
        image_topic = p('image_topic', '/openarm_zed/image').value
        depth_topic = p('depth_topic', '/openarm_zed/depth_image').value
        info_topic = p('camera_info_topic', '/openarm_zed/camera_info').value
        out_ns = p('output_namespace', '/cup_sorting').value.rstrip('/')
        # HSV thresholds per colour: hsv.<colour>.hue_ranges = [lo, hi, lo, hi, ...], s_min, v_min
        self.specs = {}
        for c in self.colors:
            if c not in COLOR_SPECS:
                raise ValueError(f'unknown colour {c!r}; known: {sorted(COLOR_SPECS)}')
            d = COLOR_SPECS[c]
            self.specs[c] = make_spec(c, p(f'hsv.{c}.hue_ranges', flat_hue_ranges(d)).value,
                                      p(f'hsv.{c}.s_min', d.s_min).value,
                                      p(f'hsv.{c}.v_min', d.v_min).value)
        unknown_draw = [c for c in self.colors if c not in DRAW_COLOR]
        if unknown_draw:
            raise ValueError(f'no debug drawing colour for {unknown_draw}')

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.K = None
        self.history = {c: deque(maxlen=self.window) for c in self.colors}

        self.pose_pub = {c: self.create_publisher(PoseStamped, f'{out_ns}/{c}_cube/pose', 10) for c in self.colors}
        self.raw_pub = {c: self.create_publisher(PointStamped, f'{out_ns}/{c}_cube/raw', 10) for c in self.colors}
        self.debug_pub = self.create_publisher(Image, f'{out_ns}/debug_image', 2)
        self.marker_pub = self.create_publisher(MarkerArray, f'{out_ns}/detection_markers', 2)

        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, image_topic, qos_profile=qos_profile_sensor_data),
             Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)],
            queue_size=5, slop=0.03)
        self.sync.registerCallback(self.on_frames)
        self.last_log = {c: 0.0 for c in self.colors}
        self.frames = 0
        self.get_logger().info(f'cube detector started: colours={self.colors}, output frame={self.target_frame}, '
                               f'input {image_topic} + {depth_topic}, output {out_ns}/<colour>_cube/pose')

    # ------------------------------------------------------------------ callbacks
    def on_info(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=float).reshape(3, 3)
        if K[0, 0] <= 0 or K[1, 1] <= 0:
            self.get_logger().warn('invalid CameraInfo K, ignoring', throttle_duration_sec=5.0)
            return
        self.K = K

    def on_frames(self, rgb_msg: Image, depth_msg: Image):
        if self.K is None:
            self.get_logger().info('waiting for CameraInfo...', throttle_duration_sec=5.0)
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, self.optical_frame, Time(),
                                                 timeout=Duration(seconds=0.1))
        except Exception as exc:  # noqa: BLE001 - TF not ready yet
            self.get_logger().info(f'waiting for TF {self.target_frame} <- {self.optical_frame}: {exc}',
                                   throttle_duration_sec=5.0)
            return
        T = cg.transform_to_matrix(tf)

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        debug = rgb.copy()
        markers = MarkerArray()
        now_sec = Time.from_msg(rgb_msg.header.stamp).nanoseconds * 1e-9

        for idx, color in enumerate(self.colors):
            est = self.detect(color, hsv, depth, T, debug)
            hist = self.history[color]
            if est is None or est[0] is None:
                # missing / rejected detection: forget old samples so a stale pose is never re-published
                while hist and now_sec - hist[0][0] > self.window_sec:
                    hist.popleft()
                if est is not None:
                    self.annotate(debug, color, est[2], False)
                continue
            center, yaw, info = est
            hist.append((now_sec, center, yaw))
            raw = PointStamped()
            raw.header.stamp = rgb_msg.header.stamp
            raw.header.frame_id = self.target_frame
            raw.point.x, raw.point.y, raw.point.z = (float(v) for v in center)
            self.raw_pub[color].publish(raw)

            stable = self.stable_estimate(color, now_sec)
            if stable is not None:
                c_med, yaw_med = stable
                ps = PoseStamped()
                ps.header = raw.header
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in c_med)
                q = cg.yaw_to_quaternion(yaw_med)
                ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
                self.pose_pub[color].publish(ps)
                markers.markers.append(self.cube_marker(ps, color, idx))
                if now_sec - self.last_log[color] > 2.0:
                    self.last_log[color] = now_sec
                    self.get_logger().info(
                        f'{color} cube STABLE at ({c_med[0]:.3f}, {c_med[1]:.3f}, {c_med[2]:.3f}) m, '
                        f'yaw {math.degrees(yaw_med):.1f} deg, {info}')
            self.annotate(debug, color, info, stable is not None)

        dbg = self.bridge.cv2_to_imgmsg(debug, encoding='rgb8')
        dbg.header = rgb_msg.header
        self.debug_pub.publish(dbg)
        if markers.markers:
            self.marker_pub.publish(markers)

    # ------------------------------------------------------------------ algorithm
    def detect(self, color, hsv, depth, T, debug):
        mask = color_mask(hsv, self.specs[color])
        blobs = find_blobs(mask, color, self.min_area, self.max_area, self.min_fill)
        if not blobs:
            return None
        blob = blobs[0]
        inner = blob.mask
        if self.erode_px > 0:
            inner = cv2.erode(blob.mask, np.ones((2 * self.erode_px + 1,) * 2, np.uint8))
        vs, us = np.nonzero(inner)
        d = depth[vs, us]
        ok = np.isfinite(d) & (d > self.min_depth) & (d < self.max_depth)
        info = {'px': blob.area, 'u': round(blob.centroid[0]), 'v': round(blob.centroid[1]),
                'valid_depth': int(ok.sum())}
        contours, _ = cv2.findContours(blob.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, DRAW_COLOR[color], 2)
        if ok.sum() < self.min_valid_px:
            info['reject'] = 'too few valid depth pixels'
            return None, None, info  # handled below
        pts_opt = cg.back_project(us[ok].astype(float), vs[ok].astype(float), d[ok].astype(float), self.K)
        pts = cg.apply(T, pts_opt)
        top_z = np.percentile(pts[:, 2], 95)
        top = pts[pts[:, 2] > top_z - self.top_band]
        if len(top) < 8:
            info['reject'] = 'top face too small'
            return None, None, info
        (cx, cy), (w, h), ang = cv2.minAreaRect(top[:, :2].astype(np.float32))
        info['top_face_mm'] = (round(w * 1000), round(h * 1000))
        # the visible top face can be slightly smaller than the cube (eroded edges), never much larger
        if max(w, h) > self.cube_size + self.size_tol or max(w, h) < self.cube_size * 0.45:
            info['reject'] = 'top face size does not match a cube'
            return None, None, info
        center = np.array([cx, cy, top_z - self.cube_size / 2.0])
        yaw = math.radians(ang)
        yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4      # cube symmetry: (-45, 45] deg
        return center, yaw, info

    def stable_estimate(self, color, now_sec):
        hist = [h for h in self.history[color] if now_sec - h[0] <= self.window_sec]
        if len(hist) < self.window:
            return None
        cs = np.array([h[1] for h in hist])
        med = np.median(cs, axis=0)
        if np.max(np.linalg.norm(cs - med, axis=1)) > self.stab_tol:
            return None
        yaws = np.array([h[2] for h in hist])
        # circular median on the 90-degree symmetric yaw
        ang = math.atan2(np.median(np.sin(4 * yaws)), np.median(np.cos(4 * yaws))) / 4
        return med, ang

    # ------------------------------------------------------------------ visualisation
    def cube_marker(self, ps, color, idx):
        m = Marker()
        m.header = ps.header
        m.ns, m.id, m.type, m.action = 'cubes', idx, Marker.CUBE, Marker.ADD
        m.pose = ps.pose
        m.scale.x = m.scale.y = m.scale.z = self.cube_size
        r, g, b = DRAW_COLOR[color]
        m.color.r, m.color.g, m.color.b, m.color.a = r / 255, g / 255, b / 255, 0.6
        m.lifetime.sec = 1
        return m

    def annotate(self, img, color, info, stable):
        if info is None:
            return
        u, v = info.get('u', 0), info.get('v', 0)
        txt = f"{color}: {'STABLE' if stable else info.get('reject', 'settling')}"
        cv2.putText(img, txt, (max(u - 40, 0), max(v - 22, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    DRAW_COLOR[color], 1, cv2.LINE_AA)
        cv2.drawMarker(img, (u, v), DRAW_COLOR[color], cv2.MARKER_CROSS, 10, 1)


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
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
