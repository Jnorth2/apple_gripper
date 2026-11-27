"""ROS helper for the AVF inside another node (grasp_controller).

Keeps the trellis / tree-template obstacles (from MoveIt CollisionObjects) and looks up
the AVF control points through TF, so a controller can call the pure functions in
air_functions.py / avf.py each tick:

    scene = AVFScene(node, tf_buffer)
    ctx = scene.context()          # None if TF is not ready
    ctx.obstacles, ctx.points, ctx.radii, ctx.maps, ctx.R_wedge

Same parsing as avf_node.py (which stays the standalone visualiser / constraint node).
"""

from dataclasses import dataclass
import threading

import numpy as np
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R

from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformException

try:
    from avf import Cylinder, arm_point_map, rigid_point_map
except ImportError:
    from gripper.avf import Cylinder, arm_point_map, rigid_point_map


DEFAULT_GRIPPER_POINTS = ['gripper_scups_link:0.05', 'gripper_link:0.07',
                          'tool0:0.05', 'wrist_3_link:0.05']
DEFAULT_ARM_POINTS = ['forearm_link:0.06', 'wrist_1_link:0.05', 'wrist_2_link:0.05']
DEFAULT_JOINT_FRAMES = ['shoulder_link', 'upper_arm_link', 'forearm_link',
                        'wrist_1_link', 'wrist_2_link', 'wrist_3_link']


def _pose_to_rt(pose):
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    if np.linalg.norm(q) < 1e-9:
        q = [0.0, 0.0, 0.0, 1.0]
    return R.from_quat(q), np.array([pose.position.x, pose.position.y, pose.position.z])


def _tf_to_rt(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    return R.from_quat([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])


def parse_points(specs):
    """['frame:radius', ...] -> [(frame, radius), ...]"""
    out = []
    for s in specs:
        frame, _, rad = s.partition(':')
        if frame.strip():
            out.append((frame.strip(), float(rad) if rad else 0.0))
    return out


def cylinders_from_collision_object(co, previous=()):
    """Cylinders (and spheres as zero-length cylinders) of a CollisionObject, in its
    header frame. Boxes / meshes are skipped."""
    R_obj, t_obj = _pose_to_rt(co.pose)
    cyls = list(previous)
    for i, (prim, ppose) in enumerate(zip(co.primitives, co.primitive_poses)):
        R_p, t_p = _pose_to_rt(ppose)
        Rw, tw = R_obj * R_p, R_obj.apply(t_p) + t_obj
        name = f'{co.id}/{i + len(previous)}'
        if prim.type == SolidPrimitive.CYLINDER:
            h = prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT]
            r = prim.dimensions[SolidPrimitive.CYLINDER_RADIUS]
            half = Rw.apply([0.0, 0.0, h / 2.0])
            cyls.append(Cylinder(tw - half, tw + half, r, name))
        elif prim.type == SolidPrimitive.SPHERE:
            cyls.append(Cylinder(tw, tw, prim.dimensions[SolidPrimitive.SPHERE_RADIUS], name))
    return cyls


@dataclass
class AVFContext:
    obstacles: list           # Cylinders in the planning frame, near the tool
    points: np.ndarray        # (P,3) control points in the planning frame
    radii: np.ndarray         # (P,)
    names: list
    maps: list                # P x (3x6) twist-about-tool0 -> point velocity
    R_wedge: R                # rotation gripper_scups_link -> planning frame
    origin: np.ndarray        # tool0 in the planning frame


class AVFScene:
    def __init__(self, node, tf_buffer, planning_frame='base_link', origin_frame='tool0',
                 wedge_frame='gripper_scups_link', gripper_points=DEFAULT_GRIPPER_POINTS,
                 arm_points=DEFAULT_ARM_POINTS, joint_frames=DEFAULT_JOINT_FRAMES,
                 collision_topic='collision_object', ignore_object_ids=(),
                 near_radius=1.0, jacobian_damping=0.01, callback_group=None):
        self.node, self.tf_buffer = node, tf_buffer
        self.planning_frame, self.origin_frame, self.wedge_frame = (
            planning_frame, origin_frame, wedge_frame)
        self.gripper_points = parse_points(gripper_points)
        self.arm_points = parse_points(arm_points)
        self.joint_frames = list(joint_frames)
        self.ignore = set(ignore_object_ids)
        self.near_radius = near_radius
        self.damping = jacobian_damping
        self.objects = {}                      # id -> (frame_id, [Cylinder])
        self._lock = threading.Lock()

        node.create_subscription(CollisionObject, collision_topic, self._collision_cb, 10,
                                 callback_group=callback_group)
        self._scene_client = node.create_client(GetPlanningScene, '/get_planning_scene',
                                                callback_group=callback_group)
        self._scene_requested = False
        self._scene_timer = node.create_timer(1.0, self._maybe_request_scene,
                                              callback_group=callback_group)

    # ---------------- obstacles ----------------

    def _maybe_request_scene(self):
        """Templates published before this node started are only in MoveIt's scene."""
        if self._scene_requested:
            self._scene_timer.cancel()
            return
        if not self._scene_client.service_is_ready():
            return
        self._scene_requested = True
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        self._scene_client.call_async(req).add_done_callback(self._scene_done)

    def _scene_done(self, fut):
        try:
            objs = fut.result().scene.world.collision_objects
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f'AVF: get_planning_scene failed: {exc}')
            return
        for co in objs:
            co.operation = CollisionObject.ADD
            self._collision_cb(co)

    def _collision_cb(self, co):
        with self._lock:
            if co.operation == CollisionObject.REMOVE:
                if co.id == '':
                    self.objects.clear()
                else:
                    self.objects.pop(co.id, None)
                return
            if co.operation not in (CollisionObject.ADD, CollisionObject.APPEND):
                return
            prev = () if co.operation == CollisionObject.ADD else \
                self.objects.get(co.id, (co.header.frame_id, []))[1]
            self.objects[co.id] = (co.header.frame_id,
                                   cylinders_from_collision_object(co, prev))

    def has_obstacles(self):
        with self._lock:
            return any(oid not in self.ignore and cyls
                       for oid, (_, cyls) in self.objects.items())

    def _lookup(self, target, source):
        try:
            return _tf_to_rt(self.tf_buffer.lookup_transform(target, source, Time()))
        except TransformException:
            return None

    def _obstacles(self, near):
        with self._lock:
            items = list(self.objects.items())
        out = []
        for oid, (frame, cyls) in items:
            if oid in self.ignore:
                continue
            if frame in ('', self.planning_frame):
                moved = cyls
            else:
                rt = self._lookup(self.planning_frame, frame)
                if rt is None:
                    self.node.get_logger().warn(
                        f'AVF: no TF {frame} -> {self.planning_frame}; ignoring {oid}',
                        throttle_duration_sec=5.0)
                    continue
                Rf, tf = rt
                moved = [Cylinder(Rf.apply(c.p0) + tf, Rf.apply(c.p1) + tf, c.radius, c.name)
                         for c in cyls]
            # keep only what is within near_radius of the tool (speed: templates are big)
            for c in moved:
                ab = c.p1 - c.p0
                t = np.clip((near - c.p0) @ ab / max(ab @ ab, 1e-12), 0, 1)
                if np.linalg.norm(c.p0 + t * ab - near) - c.radius < self.near_radius:
                    out.append(c)
        return out

    # ---------------- per tick ----------------

    def context(self):
        """Everything the wedge scoring needs, in the planning frame, or None if TF
        for the tool is not available yet."""
        pf = self.planning_frame
        o = self._lookup(pf, self.origin_frame)
        w = self._lookup(pf, self.wedge_frame)
        if o is None or w is None:
            return None
        origin = o[1]

        P, radii, names, maps = [], [], [], []
        for f, rad in self.gripper_points:
            rt = self._lookup(pf, f)
            if rt is None:
                continue
            P.append(rt[1]); radii.append(rad); names.append(f)
            maps.append(rigid_point_map(rt[1], origin))

        joints = [self._lookup(pf, f) for f in self.joint_frames]
        if all(j is not None for j in joints):
            O = np.array([j[1] for j in joints])
            Z = np.array([j[0].apply([0.0, 0.0, 1.0]) for j in joints])
            for f, rad in self.arm_points:
                if f not in self.joint_frames:
                    continue
                rt = self._lookup(pf, f)
                if rt is None:
                    continue
                P.append(rt[1]); radii.append(rad); names.append(f)
                maps.append(arm_point_map(rt[1], self.joint_frames.index(f) + 1, origin,
                                          O, Z, self.damping))

        return AVFContext(self._obstacles(origin), np.array(P).reshape(-1, 3),
                          np.array(radii), names, maps, w[0], origin)
