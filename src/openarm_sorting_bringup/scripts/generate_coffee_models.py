#!/usr/bin/env python3
"""Generate the coffee-workstation Gazebo models, the world and the geometry YAML.

Single source of truth for every dimension of the coffee scenario. Writes (SOURCE tree):

  models/espresso_machine/      single-group espresso machine with a BAYONET group head
  models/portafilter/           58 mm portafilter (two bayonet lugs, handle, ArUco tag)
  models/portafilter_dock/      portafilter stand the portafilter starts in / is returned to
  models/coffee_workstation_table/
  worlds/openarm_coffee_world.sdf
  config/coffee/coffee_geometry.yaml   object geometry + marker->object transforms for the
                                       perception / task configuration (generated)

Re-run after changing a dimension (from the workspace root), then rebuild:
    python3 src/openarm_sorting_bringup/scripts/generate_coffee_models.py

Frames (all right-handed, metres):
  portafilter frame  P : origin on the basket axis in the RIM plane, +z = basket opening (up),
                         +x = handle direction (basket -> handle end)
  mount frame        M : origin on the group-head axis in the group-head BOTTOM plane, +z = up
                         (insertion direction), +x = direction the handle points when LOCKED
  When locked, P = M * Trans(0, 0, LOCKED_Z).

Bayonet (simplified but physical - only primitive collisions, no plugin):
  * the group head has a LIP ring (z_M 0..LIP_H) whose inner radius just clears the basket and
    blocks the lugs, except in two NOTCHES at +-90 deg - LOCK_ANGLE from the mount x axis
  * above the lip a lug channel (wall radius CHANNEL_R) lets the lugs turn; the shower-screen
    disk closes the top
  * insert with the handle rotated by -LOCK_ANGLE (lugs in the notches), lift by INSERT_DEPTH,
    rotate by +LOCK_ANGLE: the lugs sit over the lip -> the portafilter cannot be pulled down
    (it hangs on the lip if released). Unlock = reverse.

ArUco markers: DICT_4X4_50, textured OBJ quads (white quiet zone of one cell around the black
square). Marker frame (OpenCV convention): origin at the square centre, x to the right of the
printed marker, y up, z out of the marker face.
"""
import math
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, '..'))
MODELS = os.path.join(PKG, 'models')

# ------------------------------------------------------------------ world layout (world frame)
TABLE_TOP = 0.30
TABLE_CENTER_XY = (0.485, 0.10)        # x 0.12..0.85, y -0.35..0.55
TABLE_SIZE = (0.73, 0.90, 0.025)
MOUNT_XY = (0.455, 0.08)               # group-head axis (chosen from the LEFT-arm IK study)
MOUNT_Z = 0.45                         # group-head bottom plane (15 cm above the table)
MOUNT_YAW = math.pi                    # mount x = world -x (the handle points at the robot)
DOCK_PF_XY = (0.44, 0.32)              # basket axis of the portafilter in the dock
DOCK_PF_YAW = math.pi                  # handle points at the robot
DOCK_PF_Z = 0.434                      # rim plane of the portafilter resting in the dock

# ------------------------------------------------------------------ ArUco
ARUCO_DICT = 'DICT_4X4_50'
PF_MARKER_ID = 7
PF_MARKER_SIZE = 0.040                 # black square edge (m)
MACHINE_MARKER_ID = 21
MACHINE_MARKER_SIZE = 0.045

# ------------------------------------------------------------------ portafilter (frame P)
BASKET_R = 0.035                       # body outer radius (58 mm basket, ~70 mm body)
BODY_H = 0.050                         # rim (z=0) down to the body bottom
LUG_R_OUT = 0.042                      # lugs reach out to this radius
LUG_W = 0.012                          # tangential width
LUG_Z = (-0.008, -0.002)
NECK_X = (0.030, 0.062)
NECK_YW = 0.024
NECK_Z = (-0.044, -0.024)
GRIP_X = (0.060, 0.190)                # handle grip along +x
GRIP_YW = 0.032                        # width across the gripper pads
GRIP_MU = 3.0                          # friction of the grip (rubberised handle)
GRIP_Z = (-0.048, -0.020)
SPOUT = dict(x=0.0, size=(0.030, 0.016, 0.016), z=-0.058)   # visual only
TAG_PLATE = dict(x=0.090, size=(0.054, 0.054, 0.003))       # on top of the grip, near the neck
TAG_TILT_DEG = 30.0                    # tag wedge: marker normal tilted towards the handle end (+x_P),
                                       # which faces the operator/camera in the dock and in the machine
PF_MASS_BODY = 0.30                    # stainless body + basket
PF_MASS_HANDLE = 0.09
PF_MASS_TAG = 0.005

# ------------------------------------------------------------------ group head (frame M)
LOCK_ANGLE_DEG = 45.0                  # rotation from 'inserted' to 'locked' about +z_M
LIP_H = 0.0095                         # lip top 1.5 mm below the lugs of the locked portafilter
LIP_R_IN = 0.0375                      # 2.5 mm radial clearance to the basket
COLLAR_R_OUT = 0.050
CHANNEL_R = 0.046                      # lug channel wall (4 mm clearance to the lugs)
CHANNEL_TOP = 0.0215                   # shower screen plane, 2.5 mm above the locked rim
# (tested: 0.8 / 1.5 mm gaps make the lugs bind during the locking turn)
COLLAR_TOP = 0.045                     # meets the head block
NOTCH_HALF_DEG = 15.0                  # notch half width (lug half width = 8.2 deg)
LOCKED_Z = 0.019                       # rim plane of the LOCKED portafilter above the mount plane
INSERT_DEPTH = 0.045                   # rim travel from 'pre-insert' (below) to 'inserted'

# ------------------------------------------------------------------ machine (world frame boxes)
BODY = dict(x=(0.50, 0.78), y=(-0.045, 0.205), z=(0.30, 0.64))
DRIP = dict(x=(0.385, 0.50), y=(-0.035, 0.195), z=(0.30, 0.33))
HEAD = dict(x=(0.405, 0.50), y=(0.02, 0.14), z=(0.495, 0.585))
CHAMFER = 0.05                          # 45 deg chamfer on the head block's front-top edge

# ------------------------------------------------------------------ dock (frame P of the docked portafilter)
DOCK_RING_R_IN = 0.046                  # funnel top radius (11 mm lead-in)
DOCK_SEAT_R = 0.038                     # seat: 3 mm radial clearance to the basket body
DOCK_SEAT_H = 0.005                     # below the neck: the seat never catches the handle
DOCK_RING_R_OUT = 0.054
DOCK_RING_H = 0.024                     # holder walls above the dock floor
DOCK_FLOOR_Z = -BODY_H                  # the body bottom rests on the floor
DOCK_REST_X = (0.175, 0.190)            # handle rest under the grip end (clear of the grasp zone)
DOCK_SLOT_HALF_DEG = 28.0               # opening in the lead-in funnel for the handle neck
DOCK_KEY_HALF_DEG = 28.0                # seat-ring slot for the neck


# ============================================================================ math helpers
def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def T(R=None, t=(0, 0, 0)):
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    M[:3, 3] = t
    return M


def rpy_of(R):
    """Fixed-axis roll, pitch, yaw (R = Rz(y) Ry(p) Rx(r))."""
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    else:
        r, y = math.atan2(-R[1, 2], R[1, 1]), 0.0
    return r, p, y


def pose_str(M):
    r, p, y = rpy_of(M[:3, :3])
    x, yy, z = M[:3, 3]
    return f'{x:.5f} {yy:.5f} {z:.5f} {r:.6f} {p:.6f} {y:.6f}'


def xyz_rpy(M):
    r, p, y = rpy_of(M[:3, :3])
    return [round(float(v), 6) for v in M[:3, 3]], [round(float(v), 6) for v in (r, p, y)]


# ============================================================================ SDF helpers
def material(rgb, rough=0.6, metal=0.0, spec=0.2):
    r, g, b = rgb
    return (f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>'
            f'<specular>{spec:.2f} {spec:.2f} {spec:.2f} 1</specular>'
            f'<pbr><metal><metalness>{metal:.2f}</metalness><roughness>{rough:.2f}</roughness></metal></pbr></material>')


STEEL = dict(rgb=(0.74, 0.75, 0.77), rough=0.35, metal=0.3, spec=0.6)
DARK = dict(rgb=(0.10, 0.10, 0.11), rough=0.55, metal=0.0, spec=0.2)
WHITE = dict(rgb=(0.95, 0.95, 0.95), rough=0.9, metal=0.0, spec=0.05)
BLACKSTEEL = dict(rgb=(0.20, 0.20, 0.21), rough=0.45, metal=0.3, spec=0.4)
RED = dict(rgb=(0.62, 0.08, 0.06), rough=0.35, metal=0.2, spec=0.4)
WOOD = dict(rgb=(0.55, 0.38, 0.22), rough=0.6, metal=0.0, spec=0.15)


def surface(mu):
    return f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>'


class LinkBuilder:
    def __init__(self):
        self.parts = []

    def box(self, name, pose, size, mat=None, collide=True, visual=True, mu=0.8):
        g = f'<geometry><box><size>{size[0]:.5f} {size[1]:.5f} {size[2]:.5f}</size></box></geometry>'
        self._add(name, pose, g, mat, collide, visual, mu)

    def cyl(self, name, pose, r, h, mat=None, collide=True, visual=True, mu=0.8):
        g = f'<geometry><cylinder><radius>{r:.5f}</radius><length>{h:.5f}</length></cylinder></geometry>'
        self._add(name, pose, g, mat, collide, visual, mu)

    def mesh_visual(self, name, pose, uri):
        self.parts.append(f'      <visual name="{name}"><pose>{pose_str(pose)}</pose>'
                          f'<geometry><mesh><uri>{uri}</uri></mesh></geometry></visual>')

    def _add(self, name, pose, g, mat, collide, visual, mu):
        ps = pose_str(pose)
        if collide:
            self.parts.append(f'      <collision name="{name}_c"><pose>{ps}</pose>{g}{surface(mu)}</collision>')
        if visual:
            self.parts.append(f'      <visual name="{name}_v"><pose>{ps}</pose>{g}{material(**(mat or STEEL))}</visual>')

    def ring(self, name, r_in, r_out, z0, z1, n, mat, skip=(), collide=True, visual=True, frame=np.eye(4)):
        """Annulus of n box segments between r_in and r_out, z0..z1 (in `frame`). `skip`: list of
        (centre_rad, half_width_rad) angular ranges left open."""
        dth = 2 * math.pi / n
        rc = (r_in + r_out) / 2
        chord = 2 * r_out * math.tan(dth / 2) * 1.02
        for k in range(n):
            th = k * dth
            if any(abs((th - c + math.pi) % (2 * math.pi) - math.pi) < hw + dth / 2 for c, hw in skip):
                continue
            pose = frame @ T(rz(th), (rc * math.cos(th), rc * math.sin(th), (z0 + z1) / 2))
            self.box(f'{name}_{k:02d}', pose, (r_out - r_in, chord, z1 - z0), mat, collide, visual)


def _cone_ring(self, name, r_bot, r_top, z0, z1, thick, n, mat, skip=()):
    """Conical funnel of n tilted box segments: inner surface from radius r_bot at z0 to r_top at
    z1 (a lead-in chamfer), wall thickness `thick`."""
    dth = 2 * math.pi / n
    a = math.atan2(r_top - r_bot, z1 - z0)                 # tilt of the wall from vertical
    L = math.hypot(r_top - r_bot, z1 - z0)
    chord = 2 * (r_top + thick) * math.tan(dth / 2) * 1.02
    for k in range(n):
        th = k * dth
        if any(abs((th - c + math.pi) % (2 * math.pi) - math.pi) < hw + dth / 2 for c, hw in skip):
            continue
        mid_r, mid_z = (r_bot + r_top) / 2, (z0 + z1) / 2
        cr = mid_r + thick / 2 * math.cos(a)
        cz = mid_z - thick / 2 * math.sin(a)
        pose = T(rz(th) @ ry(a), (cr * math.cos(th), cr * math.sin(th), cz))
        self.box(f'{name}_{k:02d}', pose, (thick, chord, L), mat)


LinkBuilder.cone_ring = _cone_ring


def model_sdf(name, body, static, inertial='', comment=''):
    return f'''<?xml version="1.0"?>
<!-- Generated by openarm_sorting_bringup/scripts/generate_coffee_models.py - edit the script, not this file. -->
<sdf version="1.9">
  <model name="{name}">
    <static>{'true' if static else 'false'}</static>
    <!-- {comment} -->
    <link name="link">
{inertial}
{body}
    </link>
  </model>
</sdf>
'''


def model_config(name, desc):
    return f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{desc} Generated by openarm_sorting_bringup/scripts/generate_coffee_models.py.</description>
</model>
'''


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(text)


# ============================================================================ ArUco textures
def aruco_image(marker_id, cells_px=80):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICT))
    side = 6 * cells_px                                   # 4x4 bits + black border
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(d, marker_id, side)
    else:
        img = cv2.aruco.drawMarker(d, marker_id, side)
    img = cv2.copyMakeBorder(img, cells_px, cells_px, cells_px, cells_px, cv2.BORDER_CONSTANT, value=255)
    return img                                             # 8 cells wide, white quiet zone


def write_marker_mesh(model, stem, marker_id, marker_size):
    """Textured quad (OBJ + MTL + PNG). Quad = marker + one-cell white border; local frame =
    ArUco marker frame (x right, y up, z out of the face). The file stem must be unique across
    all models: Gazebo caches meshes / textures by file name."""
    mdir = os.path.join(MODELS, model, 'meshes')
    os.makedirs(mdir, exist_ok=True)
    cv2.imwrite(os.path.join(mdir, f'{stem}.png'), aruco_image(marker_id))
    h = marker_size * 8 / 6 / 2
    write(os.path.join(mdir, f'{stem}.mtl'),
          f'# generated\nnewmtl {stem}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nNs 1\nillum 1\nmap_Kd {stem}.png\n')
    write(os.path.join(mdir, f'{stem}.obj'),
          f'# {stem}: ArUco {ARUCO_DICT} id {marker_id}, black square {marker_size * 1000:.1f} mm - generated\n'
          f'mtllib {stem}.mtl\no {stem}\n'
          f'v {-h:.6f} {-h:.6f} 0\nv {h:.6f} {-h:.6f} 0\nv {h:.6f} {h:.6f} 0\nv {-h:.6f} {h:.6f} 0\n'
          'vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n'
          f'usemtl {stem}\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n')
    return f'model://{model}/meshes/{stem}.obj'


# ============================================================================ portafilter
def portafilter():
    lb = LinkBuilder()
    lb.cyl('body', T(t=(0, 0, -BODY_H / 2)), BASKET_R, BODY_H, STEEL, mu=1.0)
    # basket opening (dark, visual only) so the top reads as a basket
    lb.cyl('basket_well', T(t=(0, 0, 0.0005)), BASKET_R - 0.004, 0.001, DARK, collide=False)
    for k, th in enumerate((math.pi / 2, -math.pi / 2)):                # lugs at +-y_P
        rc = (BASKET_R + LUG_R_OUT) / 2 - 0.001
        lb.box(f'lug_{k}', T(rz(th), (rc * math.cos(th), rc * math.sin(th), sum(LUG_Z) / 2)),
               (LUG_R_OUT - BASKET_R + 0.002, LUG_W, LUG_Z[1] - LUG_Z[0]), STEEL, mu=0.6)
    lb.box('neck', T(t=(sum(NECK_X) / 2, 0, sum(NECK_Z) / 2)),
           (NECK_X[1] - NECK_X[0], NECK_YW, NECK_Z[1] - NECK_Z[0]), STEEL, mu=1.0)
    lb.box('grip', T(t=(sum(GRIP_X) / 2, 0, sum(GRIP_Z) / 2)),
           (GRIP_X[1] - GRIP_X[0], GRIP_YW, GRIP_Z[1] - GRIP_Z[0]), DARK, mu=GRIP_MU)
    lb.cyl('grip_end', T(ry(math.pi / 2), (GRIP_X[1], 0, sum(GRIP_Z) / 2)), 0.015, 0.006, DARK, collide=False)
    sx, sy, sz = SPOUT['size']
    lb.box('spout', T(t=(SPOUT['x'], 0, SPOUT['z'])), (sx, sy, sz), STEEL, collide=False)
    for s in (-1, 1):
        lb.cyl(f'spout_tip_{s}', T(t=(0, s * 0.006, SPOUT['z'] - sz / 2 - 0.004)), 0.003, 0.008, STEEL, collide=False)
    px, (psx, psy, psz) = TAG_PLATE['x'], TAG_PLATE['size']
    tilt = math.radians(TAG_TILT_DEG)
    R_tag = ry(tilt)                                                # normal -> (sin, 0, cos)
    # the lower (+x) edge of the plate rests on the grip top
    cz = GRIP_Z[1] + (psx / 2) * math.sin(tilt) + (psz / 2) * math.cos(tilt)
    nrm = R_tag[:, 2]
    lb.box('tag_plate', T(R_tag, (px, 0, cz)), (psx, psy, psz), WHITE, mu=1.0)
    lb.box('tag_wedge', T(R_tag, np.array([px, 0, cz]) - nrm * (psz / 2 + 0.007)), (psx * 0.9, 0.028, 0.014), DARK, mu=1.0)
    T_P_marker = T(R_tag, np.array([px, 0, cz]) + nrm * (psz / 2 + 0.0004))
    lb.mesh_visual('aruco_marker', T_P_marker,
                   write_marker_mesh('portafilter', f'aruco_4x4_id{PF_MARKER_ID}', PF_MARKER_ID, PF_MARKER_SIZE))

    # inertial: lumped masses (body cylinder, handle box, tag)
    parts = [(PF_MASS_BODY, np.array([0, 0, -BODY_H / 2]), 'cyl', (BASKET_R, BODY_H)),
             (PF_MASS_HANDLE, np.array([(NECK_X[0] + GRIP_X[1]) / 2, 0, sum(GRIP_Z) / 2]), 'box',
              (GRIP_X[1] - NECK_X[0], GRIP_YW, GRIP_Z[1] - GRIP_Z[0])),
             (PF_MASS_TAG, T_P_marker[:3, 3], 'box', (psx, psy, psz))]
    m = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / m
    I = np.zeros((3, 3))
    for mi, c, kind, dims in parts:
        if kind == 'cyl':
            r, h = dims
            Ic = np.diag([mi * (3 * r * r + h * h) / 12] * 2 + [mi * r * r / 2])
        else:
            a, b, cc = dims
            Ic = np.diag([mi * (b * b + cc * cc) / 12, mi * (a * a + cc * cc) / 12, mi * (a * a + b * b) / 12])
        d = c - com
        I += Ic + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    inertial = (f'      <inertial><pose>{com[0]:.5f} {com[1]:.5f} {com[2]:.5f} 0 0 0</pose><mass>{m:.3f}</mass>'
                f'<inertia><ixx>{I[0, 0]:.4e}</ixx><iyy>{I[1, 1]:.4e}</iyy><izz>{I[2, 2]:.4e}</izz>'
                f'<ixy>{I[0, 1]:.4e}</ixy><ixz>{I[0, 2]:.4e}</ixz><iyz>{I[1, 2]:.4e}</iyz></inertia></inertial>')
    write(os.path.join(MODELS, 'portafilter', 'model.sdf'),
          model_sdf('portafilter', '\n'.join(lb.parts), False, inertial,
                    'Frame P: origin on the basket axis in the rim plane, +z up, +x along the handle. '
                    f'Mass {m:.3f} kg. ArUco {ARUCO_DICT} id {PF_MARKER_ID} ({PF_MARKER_SIZE * 1000:.0f} mm) on the '
                    'handle near the neck.'))
    write(os.path.join(MODELS, 'portafilter', 'model.config'),
          model_config('portafilter', '58 mm espresso portafilter with two bayonet lugs, a 32 mm wide '
                       'handle grip and an ArUco tag.'))
    return T_P_marker, m, com


# ============================================================================ espresso machine
def espresso_machine():
    """Static model; model origin = world origin, so the model pose in the world is identity
    and every coordinate below is a world coordinate (the layout is fixed by this script)."""
    lb = LinkBuilder()

    def bx(name, d, mat, **kw):
        c = [(d[a][0] + d[a][1]) / 2 for a in 'xyz']
        s = [d[a][1] - d[a][0] for a in 'xyz']
        lb.box(name, T(t=c), s, mat, **kw)

    # body tower (side panels steel, dark front fascia, cup tray on top)
    bx('body', BODY, STEEL)
    bx('front_fascia', dict(x=(0.497, 0.500), y=(BODY['y'][0] + 0.01, BODY['y'][1] - 0.01), z=(0.34, 0.63)),
       BLACKSTEEL, collide=False)
    bx('cup_tray', dict(x=(0.52, 0.76), y=(BODY['y'][0] + 0.01, BODY['y'][1] - 0.01), z=(0.64, 0.646)), STEEL)
    for k in range(5):
        bx(f'cup_rail_{k}', dict(x=(0.53 + k * 0.05, 0.535 + k * 0.05), y=(BODY['y'][0] + 0.015, BODY['y'][1] - 0.015),
                                 z=(0.646, 0.652)), BLACKSTEEL, collide=False)
    # drip tray + grate
    bx('drip_tray', DRIP, BLACKSTEEL)
    for k in range(9):
        bx(f'grate_{k}', dict(x=(DRIP['x'][0] + 0.008, DRIP['x'][1] - 0.008),
                              y=(DRIP['y'][0] + 0.012 + k * 0.025, DRIP['y'][0] + 0.016 + k * 0.025),
                              z=(0.330, 0.332)), STEEL, collide=False)
    # brew-head housing with a 45 deg chamfer on its front-top edge (carries the mount marker)
    hx0, hx1 = HEAD['x']
    hz0, hz1 = HEAD['z']
    bx('head_rear', dict(x=(hx0 + CHAMFER, hx1), y=HEAD['y'], z=HEAD['z']), STEEL)
    bx('head_front', dict(x=(hx0, hx0 + CHAMFER), y=HEAD['y'], z=(hz0, hz1 - CHAMFER)), STEEL)
    # chamfer wedge filled by a box rotated 45 deg about y whose outer face lies on the chamfer
    L = CHAMFER * math.sqrt(2)
    n = np.array([-1, 0, 1]) / math.sqrt(2)                       # chamfer face normal (outward)
    mid = np.array([hx0 + CHAMFER / 2, (HEAD['y'][0] + HEAD['y'][1]) / 2, hz1 - CHAMFER / 2])
    th = L / 2
    lb.box('head_chamfer', T(ry(-math.pi / 4), mid - n * th / 2), (L, HEAD['y'][1] - HEAD['y'][0], th), STEEL)
    # mount marker on the chamfer: x = -y_world (image right), y = up the slope, z = n
    R_mk = np.column_stack([np.array([0, -1.0, 0]), np.cross(n, [0, -1.0, 0]), n])
    plate_t = 0.002
    mk_y = MOUNT_XY[1]
    plate_c = np.array([mid[0], mk_y, mid[2]]) + n * plate_t / 2
    lb.box('tag_plate', T(R_mk, plate_c), (MACHINE_MARKER_SIZE * 8 / 6 + 0.006,) * 2 + (plate_t,), WHITE)
    T_W_marker = T(R_mk, plate_c + n * (plate_t / 2 + 0.0004))
    lb.mesh_visual('aruco_marker', T_W_marker,
                   write_marker_mesh('espresso_machine', f'aruco_4x4_id{MACHINE_MARKER_ID}', MACHINE_MARKER_ID, MACHINE_MARKER_SIZE))

    # group head (bayonet) in the mount frame
    T_W_M = T(rz(MOUNT_YAW), (MOUNT_XY[0], MOUNT_XY[1], MOUNT_Z))
    lock = math.radians(LOCK_ANGLE_DEG)
    notch_centres = [math.pi / 2 - lock, -math.pi / 2 - lock]       # lug directions when INSERTED
    lb.ring('lip', LIP_R_IN, COLLAR_R_OUT, 0.0, LIP_H, 48, BLACKSTEEL,
            skip=[(c, math.radians(NOTCH_HALF_DEG)) for c in notch_centres], frame=T_W_M)
    lb.ring('channel', CHANNEL_R, COLLAR_R_OUT, LIP_H, COLLAR_TOP, 36, STEEL, frame=T_W_M)
    lb.cyl('shower_screen', T_W_M @ T(t=(0, 0, (CHANNEL_TOP + COLLAR_TOP) / 2)), CHANNEL_R + 0.001,
           COLLAR_TOP - CHANNEL_TOP, STEEL)
    lb.cyl('gasket', T_W_M @ T(t=(0, 0, CHANNEL_TOP - 0.0015)), CHANNEL_R - 0.001, 0.001, DARK, collide=False)
    # notch markers (visual cue where the lugs enter)
    for k, c in enumerate(notch_centres):
        lb.box(f'notch_mark_{k}', T_W_M @ T(rz(c), (COLLAR_R_OUT * math.cos(c), COLLAR_R_OUT * math.sin(c), 0.02)),
               (0.001, 0.006, 0.01), RED, collide=False)

    # front details (visual only): pressure gauge, power switch, steam knob + wand, hot-water spout
    lb.cyl('gauge', T(ry(-math.pi / 2), (0.4955, 0.17, 0.585)), 0.025, 0.006, STEEL, collide=False)
    lb.cyl('gauge_face', T(ry(-math.pi / 2), (0.4925, 0.17, 0.585)), 0.021, 0.001, WHITE, collide=False)
    lb.box('switch', T(t=(0.494, 0.17, 0.50)), (0.008, 0.018, 0.012), RED, collide=False)
    lb.cyl('steam_knob', T(ry(-math.pi / 2), (0.488, -0.015, 0.585)), 0.018, 0.02, DARK, collide=False)
    lb.cyl('steam_wand', T(ry(0.25), (0.47, -0.03, 0.47)), 0.005, 0.16, STEEL, collide=False)
    lb.cyl('feet_fl', T(t=(0.51, -0.035, 0.302)), 0.01, 0.004, DARK, collide=False)

    write(os.path.join(MODELS, 'espresso_machine', 'model.sdf'),
          model_sdf('espresso_machine', '\n'.join(lb.parts), True, '',
                    'Single-group espresso machine, 25 x 28 x 35 cm. Model origin = world origin (place at '
                    'identity). Bayonet group head: lip with two notches + lug channel + shower screen. '
                    f'ArUco {ARUCO_DICT} id {MACHINE_MARKER_ID} ({MACHINE_MARKER_SIZE * 1000:.0f} mm) on the '
                    '45 deg chamfer of the brew-head housing.'))
    write(os.path.join(MODELS, 'espresso_machine', 'model.config'),
          model_config('espresso_machine', 'Single-group espresso machine with a bayonet group head '
                       '(portafilter mount) and an ArUco marker at the mount.'))
    return T_W_M, T_W_marker


# ============================================================================ dock + table
def portafilter_dock():
    """Static; model frame = frame P of the docked portafilter."""
    lb = LinkBuilder()
    floor_top = DOCK_FLOOR_Z
    ped_h = DOCK_PF_Z + floor_top - TABLE_TOP                     # floor top above the table
    lb.cyl('pedestal', T(t=(0, 0, floor_top - ped_h / 2)), DOCK_RING_R_OUT, ped_h, WOOD)
    # holder = self-centring cup with a slot for the handle neck (+x_P):
    #   seat ring  : 1.5 mm radial clearance, DOCK_SEAT_H high
    #   lead-in    : conical funnel opening from the seat radius to DOCK_RING_R_IN at the top
    slot = [(0.0, math.radians(DOCK_SLOT_HALF_DEG))]
    seat_top = floor_top + DOCK_SEAT_H
    lb.ring('seat', DOCK_SEAT_R, DOCK_RING_R_OUT, floor_top, seat_top, 24, BLACKSTEEL,
            skip=[(0.0, math.radians(DOCK_KEY_HALF_DEG))])
    lb.cone_ring('lead_in', DOCK_SEAT_R, DOCK_RING_R_IN, seat_top, floor_top + DOCK_RING_H, 0.006, 24, BLACKSTEEL, skip=slot)
    # flat handle rest under the grip end (outside the finger zone)
    rest_top = GRIP_Z[0] - 0.001
    rest_h = rest_top - (floor_top - ped_h)
    lb.box('handle_rest', T(t=(sum(DOCK_REST_X) / 2, 0, rest_top - rest_h / 2)),
           (DOCK_REST_X[1] - DOCK_REST_X[0], 0.040, rest_h), WOOD)
    lb.box('base', T(t=(0.095, 0, floor_top - ped_h + 0.006)), (0.24, 0.10, 0.012), WOOD)
    write(os.path.join(MODELS, 'portafilter_dock', 'model.sdf'),
          model_sdf('portafilter_dock', '\n'.join(lb.parts), True, '',
                    'Portafilter stand. Model frame = frame P of the docked portafilter (rim plane).'))
    write(os.path.join(MODELS, 'portafilter_dock', 'model.config'),
          model_config('portafilter_dock', 'Wooden portafilter stand with a holder ring and a handle rest.'))
    return ped_h


def table():
    lb = LinkBuilder()
    sx, sy, sz = TABLE_SIZE
    lb.box('top', T(t=(0, 0, TABLE_TOP - sz / 2)), TABLE_SIZE, dict(rgb=(0.78, 0.72, 0.62), rough=0.7, metal=0, spec=0.15))
    leg_h = TABLE_TOP - sz
    for k, (dx, dy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        lb.box(f'leg_{k}', T(t=(dx * (sx / 2 - 0.04), dy * (sy / 2 - 0.04), leg_h / 2)), (0.04, 0.04, leg_h), DARK)
    write(os.path.join(MODELS, 'coffee_workstation_table', 'model.sdf'),
          model_sdf('coffee_workstation_table', '\n'.join(lb.parts), True, '',
                    f'Workstation table {sx} x {sy} m, top surface at z = {TABLE_TOP} m. Origin = floor below the centre.'))
    write(os.path.join(MODELS, 'coffee_workstation_table', 'model.config'),
          model_config('coffee_workstation_table', 'Coffee workstation table (top at 0.30 m).'))


# ============================================================================ world
WORLD_HEAD = open(os.path.join(PKG, 'worlds', 'openarm_cup_sorting_world.sdf')).read()


def world():
    head = WORLD_HEAD.split('<model name="ground_plane">')[0]
    head = head.replace('<world name="openarm_cup_sorting">', '<world name="openarm_coffee">')
    head = head.split('<sdf version="1.9">', 1)[1]
    head = head.replace('<camera_pose>1.35 0.9 1.15 0 0.45 -2.55</camera_pose>',
                        '<camera_pose>1.45 0.95 1.05 0 0.38 -2.55</camera_pose>')
    ground = WORLD_HEAD.split('<model name="ground_plane">', 1)[1].split('</model>', 1)[0]
    dock_pose = f'{DOCK_PF_XY[0]} {DOCK_PF_XY[1]} {DOCK_PF_Z} 0 0 {DOCK_PF_YAW:.7f}'
    return f'''<?xml version="1.0" ?>
<!--
  OpenArm 2.0 coffee workstation (portafilter installation / removal task).
  Generated by openarm_sorting_bringup/scripts/generate_coffee_models.py - edit the script.

  World frame = OpenArm root (+x forward, +y left; the LEFT arm does the task).
    table top z = {TABLE_TOP}, table spans x {TABLE_CENTER_XY[0] - TABLE_SIZE[0] / 2:.3f}..{TABLE_CENTER_XY[0] + TABLE_SIZE[0] / 2:.3f}, y {TABLE_CENTER_XY[1] - TABLE_SIZE[1] / 2:.3f}..{TABLE_CENTER_XY[1] + TABLE_SIZE[1] / 2:.3f}
    espresso machine: group-head axis ({MOUNT_XY[0]}, {MOUNT_XY[1]}), group-head bottom z = {MOUNT_Z}
    portafilter: starts in the dock, basket axis ({DOCK_PF_XY[0]}, {DOCK_PF_XY[1]}), rim z = {DOCK_PF_Z}, handle -> robot
  The task software does NOT read these numbers: it locates the portafilter and the mount with
  the camera (ArUco ids {PF_MARKER_ID} and {MACHINE_MARKER_ID}). Positions were chosen from an IK study of the left arm
  (docs/coffee/COFFEE_TASK.md).
-->
<sdf version="1.9">{head}<model name="ground_plane">{ground}</model>

    <include>
      <uri>model://coffee_workstation_table</uri>
      <name>coffee_workstation_table</name>
      <pose>{TABLE_CENTER_XY[0]} {TABLE_CENTER_XY[1]} 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://espresso_machine</uri>
      <name>espresso_machine</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://portafilter_dock</uri>
      <name>portafilter_dock</name>
      <pose>{dock_pose}</pose>
    </include>

    <include>
      <uri>model://portafilter</uri>
      <name>portafilter</name>
      <pose>{DOCK_PF_XY[0]} {DOCK_PF_XY[1]} {DOCK_PF_Z + 0.001} 0 0 {DOCK_PF_YAW:.7f}</pose>
    </include>

  </world>
</sdf>
'''


# ============================================================================ geometry YAML
def fl(values):
    """ROS 2 float array: every element written with a decimal point (homogeneous type)."""
    return '[' + ', '.join(f'{float(v):.6f}' for v in values) + ']'


def shape(name, kind, pose_xyz, dims, pose_rpy=(0, 0, 0), extra=''):
    """One collision shape as nested ROS parameters: type, pose [x y z r p y], dimensions.
    dimensions: box [sx, sy, sz] | cylinder [height, radius] | ring [height, r_in, r_out, segments, gap_deg]"""
    return (f'          {name}: {{type: {kind}, pose: {fl(list(pose_xyz) + list(pose_rpy))}, '
            f'dimensions: {fl(dims)}{extra}}}')


def geometry_yaml(T_P_marker, pf_mass, pf_com, T_W_M, T_W_mmarker):
    T_mk_P = np.linalg.inv(T_P_marker)
    T_mk_M = np.linalg.inv(T_W_mmarker) @ T_W_M
    pf_xyz, pf_rpy = xyz_rpy(T_mk_P)
    m_xyz, m_rpy = xyz_rpy(T_mk_M)
    T_M_W = np.linalg.inv(T_W_M)

    def mbox(name, d):
        """machine box (world) -> centre in frame M + size (yaw of M is pi: stays axis-aligned)."""
        c = np.array([(d[a][0] + d[a][1]) / 2 for a in 'xyz'] + [1.0])
        cm = (T_M_W @ c)[:3]
        return shape(name, 'box', cm, [d[a][1] - d[a][0] for a in 'xyz'])

    ped_h = DOCK_PF_Z + DOCK_FLOOR_Z - TABLE_TOP
    rest_top = GRIP_Z[0] - 0.001
    rest_h = rest_top - (DOCK_FLOOR_Z - ped_h)
    machine = [mbox('body', BODY), mbox('drip_tray', DRIP), mbox('brew_head', HEAD),
               shape('group_collar', 'ring', (0, 0, COLLAR_TOP / 2), [COLLAR_TOP, CHANNEL_R, COLLAR_R_OUT, 16, 0]),
               shape('shower_screen', 'cylinder', (0, 0, (CHANNEL_TOP + COLLAR_TOP) / 2), [COLLAR_TOP - CHANNEL_TOP, CHANNEL_R])]
    pf = [shape('body', 'cylinder', (0, 0, -BODY_H / 2), [BODY_H, LUG_R_OUT]),
          shape('handle', 'box', ((NECK_X[0] + GRIP_X[1]) / 2, 0, sum(GRIP_Z) / 2),
                [GRIP_X[1] - NECK_X[0], GRIP_YW, GRIP_Z[1] - GRIP_Z[0]]),
          shape('tag', 'box', T_P_marker[:3, 3] - T_P_marker[:3, 2] * 0.008,
                [TAG_PLATE['size'][0], TAG_PLATE['size'][1], 0.017], pose_rpy=rpy_of(T_P_marker[:3, :3]))]
    dock = [shape('holder', 'ring', (0, 0, DOCK_FLOOR_Z + DOCK_RING_H / 2),
                  [DOCK_RING_H, DOCK_SEAT_R, DOCK_RING_R_OUT, 12, 2 * DOCK_SLOT_HALF_DEG]),
            shape('pedestal', 'cylinder', (0, 0, DOCK_FLOOR_Z - ped_h / 2), [ped_h, DOCK_RING_R_OUT]),
            shape('handle_rest', 'box', (sum(DOCK_REST_X) / 2, 0, rest_top - rest_h / 2),
                  [DOCK_REST_X[1] - DOCK_REST_X[0], 0.040, rest_h])]
    nl = chr(10)
    return f"""# GENERATED by openarm_sorting_bringup/scripts/generate_coffee_models.py - edit the script.
#
# Object knowledge of the coffee workstation ("CAD" data a real deployment also needs):
# fixed marker -> object transforms, collision shapes in each object's OWN frame, and the
# bayonet description of the group head. It contains NO world poses: the task places these
# shapes at the poses the CAMERA measures.
#
# ROS 2 parameter file, loaded by aruco_detector, fiducial_object_tracker and the portafilter
# task node ('/**' = every node). Transforms / poses are [x, y, z, roll, pitch, yaw] (metres,
# radians, fixed-axis rpy: R = Rz(yaw) Ry(pitch) Rx(roll)), parent -> child.
#
# Frames:  portafilter    P: basket axis in the rim plane, +z up, +x along the handle
#          machine_mount  M: group-head axis in the group-head bottom plane, +z up (insertion
#                            direction), +x = handle direction when LOCKED
/**:
  ros__parameters:
    aruco_dictionary: {ARUCO_DICT}
    marker_ids: [{PF_MARKER_ID}, {MACHINE_MARKER_ID}]
    marker_sizes: {fl([PF_MARKER_SIZE, MACHINE_MARKER_SIZE])}     # black square edge [m]

    objects:
      portafilter:
        marker_id: {PF_MARKER_ID}
        marker_to_object: {fl(pf_xyz + pf_rpy)}
        mass: {pf_mass:.3f}
        shape_names: [body, handle, tag]
        shapes:      # frame P; the body radius includes the bayonet lugs
{nl.join(pf)}
      machine_mount:
        marker_id: {MACHINE_MARKER_ID}
        marker_to_object: {fl(m_xyz + m_rpy)}
        shape_names: [body, drip_tray, brew_head, group_collar, shower_screen]
        shapes:      # frame M; the collar is a ring with the lug-channel radius
{nl.join(machine)}
      portafilter_dock:
        # no marker: the dock is placed at the measured pose of the portafilter it holds
        shape_names: [holder, pedestal, handle_rest]
        shapes:      # frame P of the DOCKED portafilter
{nl.join(dock)}

    bayonet:
      locked_offset: {fl([0, 0, LOCKED_Z, 0, 0, 0])}   # frame P when locked, in frame M
      lock_angle_deg: {LOCK_ANGLE_DEG:.1f}             # inserted -> locked, about +z_M
      insertion_depth: {INSERT_DEPTH:.3f}              # rim travel along +z_M (pre-insert -> inserted)
      lip_inner_radius: {LIP_R_IN}
      channel_radius: {CHANNEL_R}

    workcell:
      # fixed cell furniture in the planning frame (like the robot itself, not perceived)
      table_center: {fl([TABLE_CENTER_XY[0], TABLE_CENTER_XY[1], TABLE_TOP - TABLE_SIZE[2] / 2])}
      table_size: {fl(TABLE_SIZE)}
"""


def main():
    T_P_marker, m, com = portafilter()
    T_W_M, T_W_mmarker = espresso_machine()
    portafilter_dock()
    table()
    write(os.path.join(PKG, 'worlds', 'openarm_coffee_world.sdf'), world())
    write(os.path.join(PKG, 'config', 'coffee', 'coffee_geometry.yaml'),
          geometry_yaml(T_P_marker, m, com, T_W_M, T_W_mmarker))
    print(f'portafilter mass {m:.3f} kg, COM {np.round(com, 4).tolist()} (frame P)')
    print('marker (P):', xyz_rpy(T_P_marker))
    print('mount marker (world):', xyz_rpy(T_W_mmarker), ' mount (world):', xyz_rpy(T_W_M))
    print('written: models/{portafilter,espresso_machine,portafilter_dock,coffee_workstation_table}, '
          'worlds/openarm_coffee_world.sdf, config/coffee/coffee_geometry.yaml')


if __name__ == '__main__':
    main()
