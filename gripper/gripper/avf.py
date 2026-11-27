"""Artificial vector field (AVF) as a constraint on the pressure-servoing direction.

Follows the ME531 proposal (Manipulation Aware Pressure Servoing):

    "Modern trellised orchards have predictable posts and wires, so they can be
    represented as templates in the collision scene [...]. Artificial vector fields
    could be used to add a constraint to the pressure servoing direction to keep the
    arm away from dangerous collisions."

So the AVF here does NOT generate motion of its own. The pressure servo decides the
twist; the AVF only limits it. numpy + scipy, no ROS, so it can be unit tested.

The field
---------
Obstacles are the trellis / tree-template cylinders (axis segment p0->p1 + radius;
a sphere is a cylinder with p0 == p1). At any point p with clearance radius r_p:

    c = closest point on the cylinder axis segment
    d = |p - c| - r_cyl - r_p                  gap between surfaces
    n = (p - c) / |p - c|                      unit vector pointing away from the obstacle

Inside the influence distance d_i, the field assigns each point a bound on how fast it
may move along -n (toward the obstacle). This is the velocity damper used by
NEO (Haviland & Corke 2021), the reactive controller the proposal builds on:

    n . p_dot  >=  -xi * (d - d_s) / (d_i - d_s)        for d < d_i

    d = d_i : may approach at up to xi
    d = d_s : may not approach at all
    d < d_s : must move away

The constraint
--------------
Control points are on the gripper (rigid with the tool) and on the arm. Each point's
velocity is a linear function of the commanded twist xi_cmd = (v, w):

    gripper / tool point : p_dot = v + w x (p - o)                     (rigid body)
    arm point            : p_dot = J_p(q) J_ee(q)^+ xi_cmd            (through the joints)

so every active (point, obstacle) pair gives one linear inequality a . xi >= b.
`constrain_twist` returns the twist closest to the servo's command that satisfies them:

    min  (xi - xi_servo)^T W (xi - xi_servo) + rho |s|^2
    s.t. A xi + s >= b

The slack s (heavily penalised) keeps the problem solvable when constraints conflict,
e.g. a gap narrower than the gripper. The result reports how much it was needed.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np
from scipy.optimize import nnls


# ---------------------------------------------------------------------------
# Obstacles and distances
# ---------------------------------------------------------------------------

@dataclass
class Cylinder:
    p0: np.ndarray
    p1: np.ndarray
    radius: float
    name: str = ""

    def __post_init__(self):
        self.p0 = np.asarray(self.p0, dtype=float).reshape(3)
        self.p1 = np.asarray(self.p1, dtype=float).reshape(3)


@dataclass
class AVFParams:
    d_influence: float = 0.12     # m, d_i: gap at which the constraint switches on
    d_safe: float = 0.02          # m, d_s: gap at which approaching is no longer allowed
    approach_speed: float = 0.05  # m/s, xi: allowed approach speed at d_i
    angular_weight: float = 0.01  # cost on changing w relative to v; ~ (lever arm in m)^2
    slack_weight: float = 1e6     # rho: how hard the constraints are
    ignore_names: List[str] = field(default_factory=list)


def closest_points_on_segment(P: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Closest point on segment a-b for each row of P (N,3). Returns (N,3)."""
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return np.broadcast_to(a, P.shape).copy()
    t = np.clip(((P - a) @ ab) / denom, 0.0, 1.0)
    return a + t[:, None] * ab


def _perpendicular(axis: np.ndarray) -> np.ndarray:
    """Any unit vector perpendicular to axis (used when a point sits exactly on the axis)."""
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    trial = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    v = np.cross(axis, trial)
    return v / np.linalg.norm(v)


def surface_distances(P: np.ndarray, cyl: Cylinder, point_radii: np.ndarray):
    """Return (gap d (N,), unit normals n (N,3)) from the cylinder surface to each point."""
    C = closest_points_on_segment(P, cyl.p0, cyl.p1)
    diff = P - C
    dist = np.linalg.norm(diff, axis=1)
    n = np.empty_like(diff)
    ok = dist > 1e-9
    n[ok] = diff[ok] / dist[ok, None]
    if np.any(~ok):
        axis = cyl.p1 - cyl.p0
        n[~ok] = _perpendicular(axis if np.linalg.norm(axis) > 1e-9 else np.array([0, 0, 1.0]))
    d = dist - cyl.radius - point_radii
    return d, n


def max_approach_speed(d, params: AVFParams):
    """Fastest allowed speed toward the obstacle at gap d (negative = must retreat)."""
    span = max(params.d_influence - params.d_safe, 1e-9)
    return params.approach_speed * (np.asarray(d, dtype=float) - params.d_safe) / span


@dataclass
class ConstraintRow:
    point: int          # index into the control points
    obstacle: str       # cylinder name
    normal: np.ndarray  # (3,) unit, away from the obstacle
    gap: float          # m
    min_normal_speed: float  # n . p_dot must be >= this (m/s)


def field_constraints(points: np.ndarray,
                      obstacles: Sequence[Cylinder],
                      params: AVFParams,
                      point_radii: Optional[np.ndarray] = None) -> List[ConstraintRow]:
    """One row per (point, obstacle) pair inside the influence distance."""
    P = np.atleast_2d(np.asarray(points, dtype=float))
    N = P.shape[0]
    radii = np.zeros(N) if point_radii is None else np.broadcast_to(
        np.asarray(point_radii, dtype=float), (N,))
    rows = []
    for cyl in obstacles:
        if cyl.name in params.ignore_names:
            continue
        d, n = surface_distances(P, cyl, radii)
        for i in np.flatnonzero(d < params.d_influence):
            rows.append(ConstraintRow(int(i), cyl.name, n[i].copy(), float(d[i]),
                                      -float(max_approach_speed(d[i], params))))
    return rows


def nearest_field(points: np.ndarray,
                  obstacles: Sequence[Cylinder],
                  params: AVFParams,
                  point_radii: Optional[np.ndarray] = None):
    """For visualization: (gap (N,), normal (N,3), max approach speed (N,)) to the nearest
    obstacle. Gap is +inf where there is no obstacle."""
    P = np.atleast_2d(np.asarray(points, dtype=float))
    N = P.shape[0]
    radii = np.zeros(N) if point_radii is None else np.broadcast_to(
        np.asarray(point_radii, dtype=float), (N,))
    gap = np.full(N, np.inf)
    nrm = np.zeros((N, 3))
    for cyl in obstacles:
        if cyl.name in params.ignore_names:
            continue
        d, n = surface_distances(P, cyl, radii)
        closer = d < gap
        gap[closer] = d[closer]
        nrm[closer] = n[closer]
    return gap, nrm, max_approach_speed(gap, params)


# ---------------------------------------------------------------------------
# Mapping the commanded twist to control-point velocities
# ---------------------------------------------------------------------------

def _skew(r: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -r[2], r[1]],
                     [r[2], 0.0, -r[0]],
                     [-r[1], r[0], 0.0]])


def rigid_point_map(p: np.ndarray, origin: np.ndarray) -> np.ndarray:
    """3x6 M with p_dot = M @ [v; w] for a point rigidly attached to the tool,
    where the twist (v, w) is taken about `origin`."""
    r = np.asarray(p, dtype=float) - np.asarray(origin, dtype=float)
    return np.hstack([np.eye(3), -_skew(r)])


def point_jacobian(p, joint_origins, joint_axes, n_upstream: int) -> np.ndarray:
    """3xJ linear-velocity Jacobian of point p for a serial arm of revolute joints.
    Only the first `n_upstream` joints move the point."""
    O = np.atleast_2d(np.asarray(joint_origins, dtype=float))
    Z = np.atleast_2d(np.asarray(joint_axes, dtype=float))
    J = np.zeros((3, O.shape[0]))
    for k in range(min(n_upstream, O.shape[0])):
        J[:, k] = np.cross(Z[k], np.asarray(p, dtype=float) - O[k])
    return J


def twist_jacobian(origin, joint_origins, joint_axes) -> np.ndarray:
    """6xJ geometric Jacobian of the twist (v, w) about `origin` on the last link."""
    O = np.atleast_2d(np.asarray(joint_origins, dtype=float))
    Z = np.atleast_2d(np.asarray(joint_axes, dtype=float))
    J = np.zeros((6, O.shape[0]))
    for k in range(O.shape[0]):
        J[:3, k] = np.cross(Z[k], np.asarray(origin, dtype=float) - O[k])
        J[3:, k] = Z[k]
    return J


def damped_pinv(J: np.ndarray, damping: float) -> np.ndarray:
    """J^T (J J^T + damping^2 I)^-1, well behaved near singularities."""
    return J.T @ np.linalg.inv(J @ J.T + damping ** 2 * np.eye(J.shape[0]))


def arm_point_map(p, n_upstream: int, origin, joint_origins, joint_axes,
                  damping: float = 0.01) -> np.ndarray:
    """3x6 M with p_dot = M @ [v; w] for a point on an arm link: the joint velocities
    that produce the tool twist (v, w), pushed through the point's own Jacobian."""
    Jp = point_jacobian(p, joint_origins, joint_axes, n_upstream)
    Je = twist_jacobian(origin, joint_origins, joint_axes)
    return Jp @ damped_pinv(Je, damping)


# ---------------------------------------------------------------------------
# The constraint solve
# ---------------------------------------------------------------------------

@dataclass
class ConstrainResult:
    twist: np.ndarray          # (6,) [v; w] after the constraint
    active: np.ndarray         # (m,) bool, rows that changed the command
    violation: float           # m/s, worst remaining violation (0 if all satisfied)
    modified: bool             # True if the servo command was changed


def solve_soft_qp(x_des: np.ndarray, weights: np.ndarray, A: np.ndarray, b: np.ndarray,
                  rho: float):
    """min 1/2 (x - x_des)^T W (x - x_des) + rho/2 |s|^2  s.t.  A x + s >= b.

    KKT gives x = x_des + W^-1 A^T lam and s = lam / rho, with lam >= 0 minimising the dual
        1/2 lam^T P lam + K^T lam,   P = A W^-1 A^T + I/rho,   K = A x_des - b.
    Since P = B^T B with B = [W^-1/2 A^T; I/sqrt(rho)], the dual is the non-negative
    least-squares problem  min |B lam + c|^2,  c = [0; sqrt(rho) K],  solved exactly by
    scipy's active-set NNLS. This stays exact when constraints conflict and P is nearly
    singular. Returns (x, lam)."""
    x_des = np.asarray(x_des, dtype=float)
    m = A.shape[0]
    if m == 0:
        return x_des.copy(), np.zeros(0)
    Winv = 1.0 / np.asarray(weights, dtype=float)
    K = A @ x_des - b                # >= 0 means the servo command already satisfies row i
    B = np.vstack([(A * np.sqrt(Winv)).T, np.eye(m) / np.sqrt(rho)])
    c = np.concatenate([np.zeros(A.shape[1]), np.sqrt(rho) * K])
    lam, _ = nnls(B, -c, maxiter=50 * (m + 6))
    x = x_des + Winv * (A.T @ lam)
    return x, lam


def constrain_twist(twist_des,
                    rows: Sequence[ConstraintRow],
                    point_maps: Sequence[np.ndarray],
                    params: AVFParams) -> ConstrainResult:
    """Closest twist to the servo command that keeps every control point inside the field.

    twist_des   (6,) [v; w] from the pressure servo, in the planning frame
    rows        from field_constraints()
    point_maps  3x6 map for each control point (rigid_point_map / arm_point_map)
    """
    x_des = np.asarray(twist_des, dtype=float).reshape(6)
    A, b = [], []
    for r in rows:
        a = r.normal @ point_maps[r.point]
        if np.linalg.norm(a) < 1e-9:
            continue          # the command cannot move this point; nothing to constrain
        A.append(a)
        b.append(r.min_normal_speed)
    if not A:
        return ConstrainResult(x_des.copy(), np.zeros(len(rows), bool), 0.0, False)
    A, b = np.array(A), np.array(b)
    weights = np.array([1.0, 1.0, 1.0] + [params.angular_weight] * 3)
    x, lam = solve_soft_qp(x_des, weights, A, b, params.slack_weight)
    active_kept = lam > 1e-12
    # map back to the full row list (rows with a ~ 0 are never active)
    active = np.zeros(len(rows), bool)
    j = 0
    for k, r in enumerate(rows):
        if np.linalg.norm(r.normal @ point_maps[r.point]) < 1e-9:
            continue
        active[k] = active_kept[j]
        j += 1
    violation = float(max(0.0, np.max(b - A @ x)))
    return ConstrainResult(x, active, violation, bool(np.any(active_kept)))


# ---------------------------------------------------------------------------
# Scoring a discrete set of candidate twists (the "wedge" / cone)
# ---------------------------------------------------------------------------
#
# air_functions.wedge_of_axes() gives N candidate unit twists (one per rotation axis
# about the apple). weight_wedge_pressure() scores them for suction-cup engagement;
# the manipulability term will score them for the arm. This section only scores the
# same rows for collision; choosing the move from the weights is the control policy's
# job, not the AVF's:
#
#   for each candidate twist xi_k:
#       step every control point forward:  p_i(s) = p_i + s * (M_i @ xi_k),  s in (0, step]
#       g_k = smallest surface gap over all points, obstacles and sub-steps
#       weight_k = 1 - exp(-(g_k - d_safe) / length_scale)     (0 if g_k <= d_safe)
#
# The weight is 1 far from obstacles and falls off exponentially to 0 at d_safe, so
# it only matters close to the trellis. Candidates that end inside d_safe are marked
# infeasible (weight 0). The wedge twists have unit angular speed, so `step` is the
# rotation angle [rad] of the look-ahead.
#
# constrain_twist() should still run on whichever twist is finally commanded: the
# look-ahead is first order and only checks the sampled path.

@dataclass
class WedgeCollisionScore:
    weights: np.ndarray       # (N,) in [0, 1], 1 = clear, 0 = infeasible
    cost: np.ndarray          # (N,) exp(-(gap - d_safe) / length_scale), grows near obstacles
    min_gap: np.ndarray       # (N,) m, smallest gap along the look-ahead (+inf if no obstacles)
    feasible: np.ndarray      # (N,) bool, min_gap > d_safe
    closest_point: np.ndarray  # (N,) int, control point that set min_gap (-1 if none)


def rotate_twists(twists: np.ndarray, R) -> np.ndarray:
    """Rotate (N,6) twists [v; w] into another frame. R is a 3x3 matrix (or scipy
    Rotation) taking vectors from the twists' frame to the target frame.
    wedge_of_axes() returns components in 'gripper_scups_link'; rotate them into the
    planning frame before scoring. The reference point (tool0) is unchanged."""
    T = np.atleast_2d(np.asarray(twists, dtype=float))
    if T.shape[0] == 0:
        return T.reshape(0, 6)
    Rm = R.as_matrix() if hasattr(R, 'as_matrix') else np.asarray(R, dtype=float)
    return np.hstack([T[:, :3] @ Rm.T, T[:, 3:] @ Rm.T])


def score_wedge_collision(wedge: np.ndarray,
                          points: np.ndarray,
                          point_maps: Sequence[np.ndarray],
                          obstacles: Sequence[Cylinder],
                          params: AVFParams,
                          point_radii: Optional[np.ndarray] = None,
                          step: float = np.radians(5.0),
                          n_substeps: int = 3,
                          length_scale: float = 0.03) -> WedgeCollisionScore:
    """Collision weight for each candidate twist in `wedge`.

    wedge        (N,6) candidate twists [v; w] in the planning frame, about the same origin
                 the point maps use (tool0). Use rotate_twists() on wedge_of_axes() output.
    points       (P,3) control points in the planning frame
    point_maps   P maps 3x6 (rigid_point_map / arm_point_map), p_dot = M @ twist
    step         look-ahead along each twist; rad for unit-angular twists
    n_substeps   gaps are checked at step/n, 2 step/n, ..., step (catches passing through)
    length_scale m, how fast the weight recovers past d_safe (63% at d_safe + length_scale)
    """
    W = np.atleast_2d(np.asarray(wedge, dtype=float))
    N = W.shape[0] if W.size else 0
    P = np.atleast_2d(np.asarray(points, dtype=float))
    n_pts = P.shape[0] if P.size else 0
    radii = np.zeros(n_pts) if point_radii is None else np.broadcast_to(
        np.asarray(point_radii, dtype=float), (n_pts,))

    min_gap = np.full(N, np.inf)
    closest = np.full(N, -1, dtype=int)
    if N and n_pts and len(obstacles):
        M = np.stack([np.asarray(m, dtype=float) for m in point_maps])   # (P,3,6)
        V = np.einsum('pij,nj->npi', M, W)                               # (N,P,3) point velocities
        for s in step * np.arange(1, max(1, n_substeps) + 1) / max(1, n_substeps):
            Q = (P[None] + s * V).reshape(-1, 3)                         # (N*P,3)
            gap, _, _ = nearest_field(Q, obstacles, params, np.tile(radii, N))
            gap = gap.reshape(N, n_pts)
            k = np.argmin(gap, axis=1)
            g = gap[np.arange(N), k]
            closer = g < min_gap
            min_gap[closer] = g[closer]
            closest[closer] = k[closer]

    margin = min_gap - params.d_safe
    with np.errstate(over='ignore'):
        cost = np.exp(-margin / max(length_scale, 1e-9))
    feasible = margin > 0
    weights = np.where(feasible, 1.0 - np.exp(-np.maximum(margin, 0) / max(length_scale, 1e-9)),
                       0.0)
    return WedgeCollisionScore(weights, cost, min_gap, feasible, closest)
