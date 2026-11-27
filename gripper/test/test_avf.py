import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'gripper'))
from avf import (AVFParams, Cylinder, arm_point_map, constrain_twist,  # noqa: E402
                 field_constraints, max_approach_speed, nearest_field, rigid_point_map,
                 solve_soft_qp)

# Vertical trellis post along z at the origin, radius 4 cm
POST = Cylinder([0, 0, 0], [0, 0, 2], 0.04, "post")
P = AVFParams(d_influence=0.12, d_safe=0.02, approach_speed=0.05)


def _one_point(p, twist, obstacles=(POST,), params=P, radius=0.0):
    p = np.asarray(p, float)
    rows = field_constraints([p], obstacles, params, [radius])
    return constrain_twist(twist, rows, [rigid_point_map(p, p)], params), rows


# ---------------- the field ----------------

def test_bound_profile():
    assert np.isclose(max_approach_speed(P.d_influence, P), P.approach_speed)
    assert np.isclose(max_approach_speed(P.d_safe, P), 0.0)
    assert max_approach_speed(0.0, P) < 0          # inside d_s: must retreat


def test_no_rows_outside_influence():
    assert field_constraints([[0.5, 0, 1]], [POST], P) == []


def test_normal_points_away_from_post():
    (row,) = field_constraints([[0.1, 0, 1]], [POST], P)
    assert np.allclose(row.normal, [1, 0, 0])
    assert np.isclose(row.gap, 0.06)


def test_point_radius_counts_toward_gap():
    # gap 0.2 - 0.04 = 0.16 > d_i without radius; 0.11 < d_i with a 5 cm radius
    assert field_constraints([[0.2, 0, 1]], [POST], P) == []
    assert len(field_constraints([[0.2, 0, 1]], [POST], P, [0.05])) == 1


def test_ignore_names_and_sphere():
    params = AVFParams(ignore_names=["post"])
    assert field_constraints([[0.1, 0, 1]], [POST], params) == []
    ball = Cylinder([0, 0, 0], [0, 0, 0], 0.04, "ball")
    (row,) = field_constraints([[0, 0.1, 0]], [ball], P)
    assert np.allclose(row.normal, [0, 1, 0])


def test_nearest_field_for_viz():
    gap, n, vmax = nearest_field([[0.1, 0, 1], [5, 0, 1]], [POST], P)
    assert np.isclose(gap[0], 0.06) and np.allclose(n[0], [1, 0, 0])
    assert np.isclose(vmax[0], max_approach_speed(0.06, P))
    gap, _, _ = nearest_field([[0.1, 0, 1]], [], P)
    assert np.isinf(gap[0])


# ---------------- constraining the servo command ----------------

def test_command_unchanged_when_clear():
    twist = [-0.03, 0.01, 0, 0, 0, 0.2]
    res, _ = _one_point([0.5, 0, 1], twist)
    assert np.allclose(res.twist, twist) and not res.modified


def test_moving_away_is_never_limited():
    twist = [0.08, 0, 0, 0, 0, 0]
    res, _ = _one_point([0.07, 0, 1], twist)
    assert np.allclose(res.twist, twist) and not res.modified


def test_approach_speed_capped_and_sideways_motion_kept():
    p = [0.1, 0, 1]                                   # gap 6 cm
    allowed = float(max_approach_speed(0.06, P))      # 0.02 m/s
    res, _ = _one_point(p, [-0.05, 0.03, 0.01, 0, 0, 0])
    v = res.twist[:3]
    assert res.modified
    assert np.isclose(v[0], -allowed, atol=1e-5)      # normal part limited to the bound
    assert np.allclose(v[1:], [0.03, 0.01], atol=1e-6)  # motion along the post untouched


def test_slow_approach_allowed():
    res, _ = _one_point([0.1, 0, 1], [-0.01, 0, 0, 0, 0, 0])
    assert not res.modified and np.isclose(res.twist[0], -0.01)


def test_inside_safe_distance_retreats():
    p = [0.05, 0, 1]                                  # gap 1 cm < d_s
    res, _ = _one_point(p, np.zeros(6))
    need = -float(max_approach_speed(0.01, P))
    assert np.isclose(res.twist[0], need, atol=1e-5) and res.twist[0] > 0


def test_rotation_that_swings_a_point_into_the_post_is_limited():
    # twist about the origin far from the post; a point 0.1 m out on x near the post.
    post = Cylinder([0.1, 0.1, -1], [0.1, 0.1, 1], 0.02, "post")
    p = np.array([0.1, 0.0, 0.0])                     # gap to post: 0.1 - 0.02 = 0.08
    origin = np.zeros(3)
    rows = field_constraints([p], [post], P)
    twist = np.array([0, 0, 0, 0, 0, 0.5])            # w_z swings p toward +y at 0.05 m/s
    res = constrain_twist(twist, rows, [rigid_point_map(p, origin)], P)
    p_dot = rigid_point_map(p, origin) @ res.twist
    assert res.modified
    assert np.isclose(p_dot[1], float(max_approach_speed(0.08, P)), atol=1e-5)


def test_conflicting_constraints_stay_finite():
    # point squeezed between two posts closer than d_s on both sides
    left = Cylinder([-0.05, 0, -1], [-0.05, 0, 1], 0.04, "L")
    right = Cylinder([0.05, 0, -1], [0.05, 0, 1], 0.04, "R")
    p = np.zeros(3)
    rows = field_constraints([p], [left, right], P)
    res = constrain_twist(np.zeros(6), rows, [rigid_point_map(p, p)], P)
    assert np.all(np.isfinite(res.twist))
    assert abs(res.twist[0]) < 1e-6                  # symmetric: no net push sideways
    assert res.violation > 0                         # reports that it could not satisfy both


def test_qp_matches_projection():
    # single half-space x0 >= 1 with identity weights -> plain projection
    x, lam = solve_soft_qp(np.zeros(6), np.ones(6), np.array([[1.0, 0, 0, 0, 0, 0]]),
                           np.array([1.0]), rho=1e9)
    assert np.allclose(x, [1, 0, 0, 0, 0, 0], atol=1e-6) and lam[0] > 0


# ---------------- arm points through the Jacobian ----------------

def _random_arm(seed=0):
    rng = np.random.default_rng(seed)
    O = rng.normal(size=(6, 3)) * 0.3
    Z = rng.normal(size=(6, 3))
    Z /= np.linalg.norm(Z, axis=1, keepdims=True)
    return O, Z


def test_arm_map_matches_rigid_map_on_last_link():
    O, Z = _random_arm()
    origin = np.array([0.2, -0.1, 0.4])
    p = np.array([0.25, 0.0, 0.35])
    M_arm = arm_point_map(p, 6, origin, O, Z, damping=1e-6)
    assert np.allclose(M_arm, rigid_point_map(p, origin), atol=1e-5)


def test_arm_map_for_upstream_link_is_consistent_with_joints():
    O, Z = _random_arm(1)
    origin = np.array([0.2, -0.1, 0.4])
    p = np.array([0.0, 0.1, 0.2])                     # rides on the first 3 joints
    twist = np.array([0.01, -0.02, 0.03, 0.1, 0.0, -0.1])
    from avf import damped_pinv, point_jacobian, twist_jacobian
    qdot = damped_pinv(twist_jacobian(origin, O, Z), 1e-6) @ twist
    expected = point_jacobian(p, O, Z, 3) @ qdot
    assert np.allclose(arm_point_map(p, 3, origin, O, Z, 1e-6) @ twist, expected)


# ---------------- scoring the wedge of candidate twists ----------------

from avf import rotate_twists, score_wedge_collision  # noqa: E402

TIP = np.array([0.0, 0.0, 0.2])          # control point 20 cm from tool0 (origin)
TIP_MAP = [rigid_point_map(TIP, np.zeros(3))]
ROT_PX = np.array([0, 0, 0, 1.0, 0, 0])  # +x rotation swings TIP toward -y
ROT_NX = -ROT_PX


def test_wedge_clear_scores_one():
    s = score_wedge_collision([ROT_PX, ROT_NX], [TIP], TIP_MAP, [], P)
    assert np.allclose(s.weights, 1) and s.feasible.all() and np.isinf(s.min_gap).all()


def test_wedge_prefers_swinging_away_from_post():
    post = Cylinder([0, -0.1, -1], [0, -0.1, 1], 0.02, "post")   # 8 cm gap on the -y side
    s = score_wedge_collision([ROT_PX, ROT_NX], [TIP], TIP_MAP, [post], P)
    assert s.feasible.all()
    assert s.min_gap[0] < s.min_gap[1] and s.weights[0] < s.weights[1]
    assert s.cost[0] > s.cost[1]                                # cost grows toward the post


def test_wedge_weight_profile_is_exponential():
    post = Cylinder([0, -0.1, -1], [0, -0.1, 1], 0.02, "post")
    ls = 0.03
    s = score_wedge_collision([ROT_NX], [TIP], TIP_MAP, [post], P, length_scale=ls)
    assert np.isclose(s.weights[0], 1 - np.exp(-(s.min_gap[0] - P.d_safe) / ls))


def test_wedge_step_into_safe_zone_is_infeasible():
    post = Cylinder([0, -0.05, -1], [0, -0.05, 1], 0.02, "post")  # 3 cm gap; 5 deg step = 1.7 cm
    s = score_wedge_collision([ROT_PX, ROT_NX], [TIP], TIP_MAP, [post], P)
    assert not s.feasible[0] and s.weights[0] == 0 and s.feasible[1]
    assert s.closest_point[0] == 0


def test_wedge_substeps_catch_passing_through():
    ball = Cylinder([0, -0.1, 0.2], [0, -0.1, 0.2], 0.01, "ball")  # on TIP's path, mid-step
    one = score_wedge_collision([ROT_PX], [TIP], TIP_MAP, [ball], P, step=1.0, n_substeps=1)
    many = score_wedge_collision([ROT_PX], [TIP], TIP_MAP, [ball], P, step=1.0, n_substeps=4)
    assert one.feasible[0] and not many.feasible[0]


def test_wedge_respects_ignore_names():
    post = Cylinder([0, -0.05, -1], [0, -0.05, 1], 0.02, "post")
    s = score_wedge_collision([ROT_PX], [TIP], TIP_MAP, [post], AVFParams(ignore_names=["post"]))
    assert s.feasible[0] and np.isclose(s.weights[0], 1)


def test_wedge_empty_inputs():
    s = score_wedge_collision(np.empty((0, 6)), [TIP], TIP_MAP, [], P)
    assert s.weights.shape == (0,)


def test_rotate_twists():
    Rz = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])          # 90 deg about z
    out = rotate_twists([[1, 0, 0, 1, 0, 0]], Rz)
    assert np.allclose(out, [[0, 1, 0, 0, 1, 0]])


def test_with_jareds_wedge_of_axes():
    from air_functions import weight_wedge_collision, wedge_of_axes
    pressures = [300, 1000, 1000]                       # cup A engaged
    wedge = wedge_of_axes(pressures, d_theta=5, gripper_height=0.19)
    # planning frame = gripper_scups_link axes, tool0 at the origin, cups 19 cm out
    cups = np.array([0.0, 0.0, 0.19])
    maps = [rigid_point_map(cups, np.zeros(3))]
    W = rotate_twists(wedge, np.eye(3))
    mid = len(wedge) // 2

    clear = weight_wedge_collision(wedge, np.eye(3), [cups], maps, [])
    assert clear.weights.shape == (len(wedge),) and np.allclose(clear.weights, 1)

    # The cups sit ~3 cm from the pivot and barely move; tool0 (19 cm lever) swings.
    # Put a post 3 cm from tool0, right where the best axis would swing it.
    tool0 = np.zeros(3)
    pts, maps = [cups, tool0], maps + [rigid_point_map(tool0, np.zeros(3))]
    dirn = maps[1] @ W[mid]
    dirn /= np.linalg.norm(dirn)
    c = tool0 + 0.04 * dirn
    axis = W[mid, 3:] / np.linalg.norm(W[mid, 3:])      # post along the axis, across the swing
    post = Cylinder(c - axis, c + axis, 0.01, "post")
    s = score_wedge_collision(W, pts, maps, [post], P)
    assert not s.feasible[mid] and s.closest_point[mid] == 1
    # axes further from the one heading at the post get more room, so higher weight
    assert s.feasible[0] and s.feasible[-1]
    assert s.weights[0] > s.weights[mid] and s.weights[-1] > s.weights[mid]
