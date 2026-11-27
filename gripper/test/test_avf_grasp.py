"""grasp_controller.avf_collision_weights with a fake node: the AVF hook, without ROS running."""
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'gripper'))
gc = pytest.importorskip('grasp_controller')       # needs the ROS env sourced
from air_functions import wedge_of_axes  # noqa: E402
from avf import Cylinder, rigid_point_map  # noqa: E402
from avf_scene import AVFContext  # noqa: E402

GRIPPER_HEIGHT = 0.19
PARAMS = {'avf_step_deg': 5.0, 'avf_d_safe': 0.02, 'avf_length_scale': 0.03}
ONE_CUP = [300.0, 1000.0, 1000.0]
WEDGE = wedge_of_axes(ONE_CUP, d_theta=2, gripper_height=GRIPPER_HEIGHT)
MID = len(WEDGE) // 2


class _Log:
    def __init__(self):
        self.msgs = []

    def info(self, m, **_):
        self.msgs.append(m)

    warn = info


class _Scene:
    def __init__(self, obstacles, tf_ready=True):
        self.obstacles, self.tf_ready = obstacles, tf_ready

    def has_obstacles(self):
        return bool(self.obstacles)

    def context(self):
        if not self.tf_ready:
            return None
        # planning frame = gripper_scups_link axes with tool0 at the origin
        tool0 = np.zeros(3)
        pts = np.array([[0.0, 0.0, GRIPPER_HEIGHT], tool0])
        return AVFContext(self.obstacles, pts, np.zeros(2), ['gripper_scups_link', 'tool0'],
                          [rigid_point_map(p, tool0) for p in pts], np.eye(3), tool0)


def _node(scene):
    log = _Log()
    return SimpleNamespace(avf_scene=scene, get_logger=lambda: log,
                           get_parameter=lambda n: SimpleNamespace(value=PARAMS[n])), log


def _weights(node, wedge=WEDGE):
    return gc.GraspController.avf_collision_weights(node, wedge)


def _post_in_the_way():
    """Post 3 cm from tool0, right where the pressure-best axis would swing it."""
    dirn = WEDGE[MID, :3] / np.linalg.norm(WEDGE[MID, :3])
    axis = WEDGE[MID, 3:]
    c = 0.04 * dirn
    return Cylinder(c - axis, c + axis, 0.01, 'post')


def test_all_ones_when_avf_off_or_no_trellis():
    assert np.allclose(_weights(_node(None)[0]), 1)
    assert np.allclose(_weights(_node(_Scene([]))[0]), 1)


def test_all_ones_when_tf_not_ready():
    node, log = _node(_Scene([_post_in_the_way()], tf_ready=False))
    assert np.allclose(_weights(node), 1)
    assert any('TF not ready' in m for m in log.msgs)


def test_empty_wedge():
    assert _weights(_node(_Scene([_post_in_the_way()]))[0], np.empty((0, 6))).shape == (0,)


def test_far_trellis_all_clear():
    far = Cylinder([0.5, 0.5, -1], [0.5, 0.5, 1], 0.03, 'post')
    w = _weights(_node(_Scene([far]))[0])
    assert w.shape == (len(WEDGE),) and np.all(w > 0.99)


def test_post_in_the_way_blocks_those_axes_only():
    node, log = _node(_Scene([_post_in_the_way()]))
    w = _weights(node)
    assert w.shape == (len(WEDGE),) and np.all((w >= 0) & (w <= 1))
    assert w[MID] == 0                                    # heads straight at the post
    assert w[0] > 0 and w[-1] > 0                         # wedge edges swing clear
    assert any('axes blocked by the trellis' in m and 'tool0' in m for m in log.msgs)


def test_grasp_controller_has_no_move_selection():
    # The AVF only provides weights; picking the move is the control policy's job
    assert not hasattr(gc.GraspController, 'avf_select_twist')
    import air_functions
    assert not hasattr(air_functions, 'select_wedge_twist')
