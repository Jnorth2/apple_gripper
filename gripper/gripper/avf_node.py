#!/usr/bin/env python3
"""Artificial vector field node: constrains the pressure-servoing twist away from the trellis.
Inputs
  input_topic (geometry_msgs/TwistStamped)       servo command, default
                                                 /servo_node/delta_twist_cmds (read only)
  collision_object (moveit_msgs/CollisionObject) trellis / tree templates
  /get_planning_scene (service, optional)        catches templates added before startup
  TF                                             control points, joint frames, twist origin

Outputs
  output_topic (geometry_msgs/TwistStamped)      constrained twist, same frame as the input,
                                                 default /avf/constrained_twist
  /avf/markers (visualization_msgs/MarkerArray)
        obstacles       parsed cylinders
        control_points  clearance spheres: grey = clear, orange = inside d_influence,
                        red = this point's constraint changed the command
        point_normals   at active points, the allowed approach velocity (toward the obstacle)
        twist           desired (grey) vs constrained (blue) linear velocity at the twist origin
        field_grid      at each grid point, the fastest allowed velocity toward the nearest
                        obstacle: long far out, zero at d_safe, flipped (must retreat) inside it

Tune live, e.g.:  ros2 param set /avf_node d_influence 0.15
Test without the servo:
  ros2 param set /avf_node use_test_twist true
  ros2 param set /avf_node test_twist "[0.0, 0.0, 0.05, 0.0, 0.0, 0.0]"
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Point, TwistStamped, Vector3
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:
    from gripper.avf import (AVFParams, Cylinder, arm_point_map, constrain_twist,
                             field_constraints, nearest_field, rigid_point_map)
except ImportError:  # running straight from the source folder
    from avf import (AVFParams, Cylinder, arm_point_map, constrain_twist,
                     field_constraints, nearest_field, rigid_point_map)


def _pose_to_rt(pose):
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    if np.linalg.norm(q) < 1e-9:  # unset quaternion -> identity
        q = [0.0, 0.0, 0.0, 1.0]
    return R.from_quat(q), np.array([pose.position.x, pose.position.y, pose.position.z])


def _tf_to_rt(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return R.from_quat([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])


def _parse_points(specs):
    """['frame:radius', ...] -> [(frame, radius), ...]"""
    out = []
    for s in specs:
        frame, _, rad = s.partition(':')
        if frame.strip():
            out.append((frame.strip(), float(rad) if rad else 0.0))
    return out


class AVFNode(Node):
    def __init__(self):
        super().__init__('avf_node')

        # Topics and frames
        self.declare_parameter('input_topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('output_topic', '/avf/constrained_twist')
        self.declare_parameter('planning_frame', 'world')
        # Point the servo twist acts about. MoveIt Servo applies it at its ee_frame_name
        # (tool0 in harvest_hardware_moveit_config/config/ur_servo.yaml).
        self.declare_parameter('twist_origin_frame', 'tool0')
        # Rigid with the tool. 'frame:clearance_radius_m'
        self.declare_parameter('gripper_points', [
            'gripper_scups_link:0.05',
            'gripper_link:0.07',
            'tool0:0.05',
            'wrist_3_link:0.05',
        ])
        # On the arm; each frame must appear in joint_frames (moved by joints up to its own)
        self.declare_parameter('arm_points', ['forearm_link:0.06', 'wrist_1_link:0.05',
                                              'wrist_2_link:0.05'])
        # Child link of each revolute joint, base to tip. ur_description rotates every
        # joint about its child link's z axis.
        self.declare_parameter('joint_frames', ['shoulder_link', 'upper_arm_link',
                                                'forearm_link', 'wrist_1_link',
                                                'wrist_2_link', 'wrist_3_link'])

        # Field (see avf.py)
        self.declare_parameter('enabled', True)
        self.declare_parameter('d_influence', 0.12)
        self.declare_parameter('d_safe', 0.02)
        self.declare_parameter('approach_speed', 0.05)
        self.declare_parameter('angular_weight', 0.01)
        self.declare_parameter('jacobian_damping', 0.01)
        self.declare_parameter('ignore_object_ids', [''])

        # Input handling
        self.declare_parameter('input_timeout', 0.5)     # s; older commands are ignored
        self.declare_parameter('use_test_twist', False)
        self.declare_parameter('test_twist', [0.0] * 6)  # [vx vy vz wx wy wz]
        self.declare_parameter('test_twist_frame', 'gripper_scups_link')

        # Loop + viz
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('arrow_scale', 2.0)       # m of arrow per m/s
        self.declare_parameter('grid_enabled', True)
        self.declare_parameter('grid_half_size', 0.25)
        self.declare_parameter('grid_spacing', 0.08)
        self.declare_parameter('grid_every_n', 5)

        self.objects = {}        # id -> (frame_id, [Cylinder in object frame])
        self.last_cmd = None     # TwistStamped
        self.last_cmd_time = None
        self.tick = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CollisionObject, 'collision_object', self.collision_cb, 10)
        self.create_subscription(TwistStamped, self.get_parameter('input_topic').value,
                                 self.cmd_cb, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, self.get_parameter('output_topic').value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/avf/markers', 10)

        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self._scene_requested = False
        self.create_timer(1.0, self._maybe_request_scene)

        self.period = 1.0 / self.get_parameter('rate').value
        self.create_timer(self.period, self.step)
        self.get_logger().info(
            f"AVF constraint up: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}")

    # ---------------- inputs ----------------

    def _maybe_request_scene(self):
        if self._scene_requested or not self.scene_client.service_is_ready():
            return
        self._scene_requested = True
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        self.scene_client.call_async(req).add_done_callback(self._scene_done)

    def _scene_done(self, fut):
        try:
            objs = fut.result().scene.world.collision_objects
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'get_planning_scene failed: {exc}')
            return
        for co in objs:
            co.operation = CollisionObject.ADD
            self.collision_cb(co)
        self.get_logger().info(f'Loaded {len(objs)} objects from the planning scene')

    def collision_cb(self, co: CollisionObject):
        if co.operation == CollisionObject.REMOVE:
            if co.id == '':
                self.objects.clear()
            else:
                self.objects.pop(co.id, None)
            return
        if co.operation not in (CollisionObject.ADD, CollisionObject.APPEND):
            return  # MOVE: ignore (trellis is static)

        R_obj, t_obj = _pose_to_rt(co.pose)
        cyls = [] if co.operation == CollisionObject.ADD else list(
            self.objects.get(co.id, (co.header.frame_id, []))[1])
        for i, (prim, ppose) in enumerate(zip(co.primitives, co.primitive_poses)):
            R_p, t_p = _pose_to_rt(ppose)
            Rw, tw = R_obj * R_p, R_obj.apply(t_p) + t_obj   # primitive in header frame
            name = f'{co.id}/{i}'
            if prim.type == SolidPrimitive.CYLINDER:
                h = prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT]
                r = prim.dimensions[SolidPrimitive.CYLINDER_RADIUS]
                half = Rw.apply([0.0, 0.0, h / 2.0])           # cylinder axis is local z
                cyls.append(Cylinder(tw - half, tw + half, r, name))
            elif prim.type == SolidPrimitive.SPHERE:
                r = prim.dimensions[SolidPrimitive.SPHERE_RADIUS]
                cyls.append(Cylinder(tw, tw, r, name))
            else:
                self.get_logger().debug(f'{name}: primitive type {prim.type} not supported, skipped')
        self.objects[co.id] = (co.header.frame_id, cyls)
        self.get_logger().info(f'Obstacle {co.id}: {len(cyls)} primitives in {co.header.frame_id}')

    def cmd_cb(self, msg: TwistStamped):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _lookup(self, target, source):
        try:
            return _tf_to_rt(self.tf_buffer.lookup_transform(target, source, Time()))
        except TransformException:
            return None

    def _desired(self):
        """(twist (6,) in its own frame, frame_id) or None."""
        g = self.get_parameter
        if g('use_test_twist').value:
            tw = np.array(g('test_twist').value, dtype=float)
            return (tw, g('test_twist_frame').value) if tw.size == 6 else None
        if self.last_cmd is None:
            return None
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > g('input_timeout').value:
            return None
        t = self.last_cmd.twist
        tw = np.array([t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z])
        return tw, self.last_cmd.header.frame_id or g('planning_frame').value

    # ---------------- main loop ----------------

    def _params(self):
        g = self.get_parameter
        ignore = [s for s in g('ignore_object_ids').value if s]
        return AVFParams(d_influence=g('d_influence').value, d_safe=g('d_safe').value,
                         approach_speed=g('approach_speed').value,
                         angular_weight=g('angular_weight').value), ignore

    def _obstacles(self, planning_frame, ignore):
        out = []
        for oid, (frame, cyls) in self.objects.items():
            if oid in ignore:
                continue
            if frame in ('', planning_frame):
                out.extend(cyls)
                continue
            rt = self._lookup(planning_frame, frame)
            if rt is None:
                self.get_logger().warn(
                    f'No TF {frame} -> {planning_frame} for {oid}; skipping '
                    f'(for sim, run tree_template with -p trellis_frame:={planning_frame})',
                    throttle_duration_sec=5.0)
                continue
            Rf, tf = rt
            out.extend(Cylinder(Rf.apply(c.p0) + tf, Rf.apply(c.p1) + tf, c.radius, c.name)
                       for c in cyls)
        return out

    def _control_points(self, frame, origin, joints):
        """Positions, radii, names and 3x6 twist->velocity maps for every control point
        whose TF is available. Arm points are skipped if the joint frames are missing."""
        g = self.get_parameter
        P, radii, names, maps, is_arm = [], [], [], [], []
        for f, rad in _parse_points(g('gripper_points').value):
            rt = self._lookup(frame, f)
            if rt is None:
                continue
            P.append(rt[1]); radii.append(rad); names.append(f); is_arm.append(False)
            maps.append(rigid_point_map(rt[1], origin))
        joint_names = list(g('joint_frames').value)
        for f, rad in _parse_points(g('arm_points').value):
            if joints is None:
                break
            if f not in joint_names:
                self.get_logger().warn(f'arm point {f} is not in joint_frames; skipped',
                                       throttle_duration_sec=10.0)
                continue
            rt = self._lookup(frame, f)
            if rt is None:
                continue
            O, Z = joints
            P.append(rt[1]); radii.append(rad); names.append(f); is_arm.append(True)
            maps.append(arm_point_map(rt[1], joint_names.index(f) + 1, origin, O, Z,
                                      g('jacobian_damping').value))
        return np.array(P).reshape(-1, 3), np.array(radii), names, maps, is_arm

    def _joints(self, frame):
        O, Z = [], []
        for f in self.get_parameter('joint_frames').value:
            rt = self._lookup(frame, f)
            if rt is None:
                return None
            O.append(rt[1]); Z.append(rt[0].apply([0.0, 0.0, 1.0]))
        return np.array(O), np.array(Z)

    def step(self):
        self.tick += 1
        g = self.get_parameter
        frame = g('planning_frame').value
        params, ignore = self._params()
        obstacles = self._obstacles(frame, ignore)
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        self._add_obstacle_markers(markers, obstacles, frame, now)

        origin_rt = self._lookup(frame, g('twist_origin_frame').value)
        if origin_rt is None:
            self.get_logger().warn(f"No TF for twist_origin_frame {g('twist_origin_frame').value}",
                                   throttle_duration_sec=5.0)
            self.marker_pub.publish(markers)
            return
        origin = origin_rt[1]
        P, radii, names, maps, is_arm = self._control_points(frame, origin, self._joints(frame))
        rows = field_constraints(P, obstacles, params, radii) if len(P) else []

        desired = self._desired()
        result = None
        if desired is not None:
            tw_in, cmd_frame = desired
            cmd_rt = self._lookup(frame, cmd_frame)
            if cmd_rt is None:
                self.get_logger().warn(f'No TF for command frame {cmd_frame}',
                                       throttle_duration_sec=5.0)
            else:
                Rc = cmd_rt[0]          # twist is rotated, not moved: it still acts at origin
                x_des = np.concatenate([Rc.apply(tw_in[:3]), Rc.apply(tw_in[3:])])
                if g('enabled').value and rows:
                    result = constrain_twist(x_des, rows, maps, params)
                    x = result.twist
                else:
                    x = x_des
                out = TwistStamped()
                out.header.stamp = now
                out.header.frame_id = cmd_frame
                v, w = Rc.inv().apply(x[:3]), Rc.inv().apply(x[3:])
                out.twist.linear = Vector3(x=v[0], y=v[1], z=v[2])
                out.twist.angular = Vector3(x=w[0], y=w[1], z=w[2])
                self.twist_pub.publish(out)
                self._add_twist_markers(markers, origin, x_des[:3], x[:3], frame, now)
                if result is not None and result.violation > 1e-4:
                    self.get_logger().warn(
                        f'Constraints conflict (gap narrower than the clearance?); '
                        f'worst violation {result.violation:.3f} m/s',
                        throttle_duration_sec=2.0)

        self._add_point_markers(markers, P, radii, rows, result, maps, frame, now, params)

        if g('grid_enabled').value and obstacles and self.tick % max(1, g('grid_every_n').value) == 0:
            self._add_grid_markers(markers, origin, obstacles, params, frame, now)

        if markers.markers:
            self.marker_pub.publish(markers)

    # ---------------- markers ----------------

    def _marker(self, ns, mid, mtype, frame, now, life_s):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = now
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(seconds=life_s).to_msg()
        return m

    def _arrow(self, ns, mid, start, vec, frame, now, color, life_s, shaft=0.006):
        m = self._marker(ns, mid, Marker.ARROW, frame, now, life_s)
        end = start + vec * self.get_parameter('arrow_scale').value
        m.points = [Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
                    Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
        m.scale = Vector3(x=shaft, y=shaft * 2.0, z=shaft * 3.0)
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        return m

    def _life(self, n=3):
        return self.period * n

    def _add_obstacle_markers(self, arr, obstacles, frame, now):
        for i, c in enumerate(obstacles):
            m = self._marker('obstacles', i, Marker.CYLINDER, frame, now, self._life())
            axis = c.p1 - c.p0
            L = np.linalg.norm(axis)
            if L < 1e-9:
                m.type = Marker.SPHERE
                m.scale = Vector3(x=2 * c.radius, y=2 * c.radius, z=2 * c.radius)
            else:
                a = axis / L
                cross = np.cross([0.0, 0.0, 1.0], a)      # rotate marker z onto the axis
                s = np.linalg.norm(cross)
                ang = np.arctan2(s, a[2])
                rv = cross / s * ang if s > 1e-9 else (
                    np.zeros(3) if a[2] > 0 else np.array([np.pi, 0.0, 0.0]))
                q = R.from_rotvec(rv).as_quat()
                m.pose.orientation.x, m.pose.orientation.y = q[0], q[1]
                m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
                m.scale = Vector3(x=2 * c.radius, y=2 * c.radius, z=float(L))
            mid = (c.p0 + c.p1) / 2.0
            m.pose.position = Point(x=mid[0], y=mid[1], z=mid[2])
            m.color = ColorRGBA(r=0.9, g=0.8, b=0.1, a=0.35)
            arr.markers.append(m)

    def _add_point_markers(self, arr, P, radii, rows, result, maps, frame, now, params):
        state = np.zeros(len(P), int)          # 0 clear, 1 inside d_influence, 2 binding
        for k, r in enumerate(rows):
            state[r.point] = max(state[r.point], 1)
            if result is not None and result.active[k]:
                state[r.point] = 2
        colors = {0: (0.6, 0.6, 0.6, 0.25), 1: (1.0, 0.6, 0.1, 0.35), 2: (1.0, 0.1, 0.1, 0.5)}
        for i, (p, r) in enumerate(zip(P, radii)):
            s = self._marker('control_points', i, Marker.SPHERE, frame, now, self._life())
            s.pose.position = Point(x=p[0], y=p[1], z=p[2])
            d = max(2 * r, 0.01)
            s.scale = Vector3(x=d, y=d, z=d)
            c = colors[int(state[i])]
            s.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])
            arr.markers.append(s)
        # allowed approach velocity for each active row: -n * max approach speed
        for k, r in enumerate(rows):
            vec = r.normal * r.min_normal_speed      # min_normal_speed = -max approach
            col = (1.0, 0.1, 0.1, 1.0) if (result is not None and result.active[k]) \
                else (1.0, 0.6, 0.1, 0.9)
            arr.markers.append(self._arrow('point_normals', k, P[r.point], vec, frame, now,
                                           col, self._life(), shaft=0.006))

    def _add_twist_markers(self, arr, origin, v_des, v_out, frame, now):
        arr.markers.append(self._arrow('twist', 0, origin, v_des, frame, now,
                                       (0.6, 0.6, 0.6, 0.8 if np.linalg.norm(v_des) > 1e-6 else 0.0),
                                       self._life(), shaft=0.010))
        arr.markers.append(self._arrow('twist', 1, origin, v_out, frame, now,
                                       (0.1, 0.3, 1.0, 1.0 if np.linalg.norm(v_out) > 1e-6 else 0.0),
                                       self._life(), shaft=0.012))

    def _add_grid_markers(self, arr, center, obstacles, params, frame, now):
        g = self.get_parameter
        h, s = g('grid_half_size').value, g('grid_spacing').value
        ax = np.arange(-h, h + 1e-9, s)
        G = np.array(np.meshgrid(ax, ax, ax, indexing='ij')).reshape(3, -1).T + center
        gap, n, vmax = nearest_field(G, obstacles, params)
        life = self.period * g('grid_every_n').value * 2.0
        for i, (p, d, ni, vm) in enumerate(zip(G, gap, n, vmax)):
            inside = d < params.d_influence
            vec = -ni * vm if inside else np.zeros(3)   # toward the obstacle; flips inside d_safe
            frac = float(np.clip(vm / max(params.approach_speed, 1e-9), 0.0, 1.0))
            color = (1.0 - frac, frac, 0.0, 0.8 if inside else 0.0)   # red = no approach left
            arr.markers.append(self._arrow('field_grid', i, p, vec, frame, now, color, life,
                                           shaft=0.003))


def main(args=None):
    rclpy.init(args=args)
    node = AVFNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
