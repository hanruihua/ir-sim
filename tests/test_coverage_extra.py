import types

# Force non-GUI backend for matplotlib usage inside plotting code
import matplotlib
import numpy as np
import pytest

import irsim

matplotlib.use("Agg")

from irsim.config import env_param
from irsim.env.env_plot import EnvPlot, draw_patch
from irsim.env.env_plot3d import EnvPlot3D
from irsim.lib.behavior.behavior import Behavior
from irsim.lib.handler.geometry_handler import GeometryFactory
from irsim.lib.path_planners.rrt_star import RRTStar
from irsim.world.obstacles.obstacle_acker import ObstacleAcker


class _DummyWorld2D:
    def __init__(self):
        self.grid_map = np.zeros((10, 10))
        self.x_range = (0, 10)
        self.y_range = (0, 10)
        self.time = 0.0
        self.status = "ok"
        self.plot_parse = {}


class _DummyWorld3D(_DummyWorld2D):
    def __init__(self):
        super().__init__()
        self.z_range = (0, 5)


def _install_dummy_logger():
    class _Logger:
        def info(self, *_args, **_kwargs):
            pass

        def warning(self, *_args, **_kwargs):
            pass

        def error(self, *_args, **_kwargs):
            pass

    env_param.logger = _Logger()


def test_env_plot_basic_branches():
    _install_dummy_logger()
    w = _DummyWorld2D()
    plot = EnvPlot(
        w, objects=[], saved_figure={}, figure_pixels=[200, 150], show_title=True
    )
    # init_plot branches
    plot._init_plot(w, [], no_axis=True, tight=True)

    # draw_trajectory with list and show_direction True
    traj_list = [np.array([[1.0], [2.0], [0.0]]), np.array([[2.0], [3.0], [0.5]])]
    plot.draw_trajectory(traj_list, show_direction=True, refresh=True)

    # draw_points with list and ndarray shapes
    plot.draw_points([[1.0, 2.0], [3.0, 4.0]], s=5, c="r", refresh=True)
    pts = np.array([[1.0, 2.0], [3.0, 4.0]])
    plot.draw_points(pts, s=5, c="b", refresh=True)
    col = np.array([[1.0], [2.0]])
    plot.draw_points(col, s=5, c="g", refresh=True)

    # quiver guards and drawing
    plot.draw_quiver(None)
    plot.draw_quiver(np.array([1.0, 2.0, 0.5, 0.25]), refresh=True)
    plot.draw_quivers([np.array([1.0, 2.0, 0.3, 0.1])], refresh=True)

    # clear all dynamic elements
    plot.clear_components("all", [])


def test_env_plot3d_branches():
    _install_dummy_logger()
    w3 = _DummyWorld3D()
    plot3d = EnvPlot3D(
        w3, objects=[], saved_figure={}, figure_pixels=[200, 150], show_title=True
    )

    # points list and ndarray
    plot3d.draw_points([[1.0, 2.0, 1.0], [2.0, 3.0, 1.5]], s=5, c="m", refresh=True)
    pts3 = np.array([[1.0, 2.0], [2.0, 3.0], [0.5, 0.7]])
    plot3d.draw_points(pts3, s=5, c="c", refresh=True)
    col3 = np.array([[1.0], [2.0], [0.5]])
    plot3d.draw_points(col3, s=5, c="y", refresh=True)

    # quiver guard and list iteration path
    plot3d.draw_quiver(None)
    plot3d.draw_quiver(np.array([1.0, 2.0, 0.5, 0.1, 0.2, 0.3]), refresh=True)
    plot3d.draw_quivers([np.array([1.0, 2.0, 0.5, 0.1, 0.2, 0.3])], refresh=True)


def test_geometry_factory_and_Gh_edges():
    # Circle G/h
    circle = GeometryFactory.create_geometry("circle", center=[0.0, 0.0], radius=1.0)
    G, h, cone, convex = circle.get_circle_Gh(np.array([[0.0], [0.0]]), 1.0)
    assert G.shape == (3, 2)
    assert h.shape == (3, 1)
    assert cone == "norm2"
    assert convex is True

    # Polygon get_polygon_Gh with None
    polygon = GeometryFactory.create_geometry(
        "polygon", vertices=[(0, 0), (1, 0), (0, 1)]
    )
    G2, h2, cone2, convex2 = polygon.get_polygon_Gh(None)
    assert G2 is None
    assert h2 is None
    assert cone2 is None
    assert convex2 is False

    # Invalid name raises
    with pytest.raises(ValueError, match="Invalid geometry name"):
        GeometryFactory.create_geometry("not-exists")


def test_behavior_edges():
    _install_dummy_logger()
    # Empty behavior dict early return
    b = Behavior(object_info=types.SimpleNamespace(id=1), behavior_dict={})
    vel = b.gen_vel(ego_object=None, external_objects=[])
    assert np.allclose(vel, np.zeros((2, 1)))

    # invoke invalid behavior path
    with pytest.raises(ValueError, match="No method found"):
        b.invoke_behavior(
            "diff", "unknown_action", ego_object=None, external_objects=[]
        )


def test_rrt_star_edge_methods():
    # Minimal instance without running full planning
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    env_map = env.get_map()
    r = RRTStar(env_map=env_map, robot_radius=0.5)
    # Monkeypatch required attributes/methods
    r.node_list = []
    r.robot_radius = 0.5
    r.expand_dis = 1.0

    # choose_parent: empty near indices
    nn = r.Node(0.0, 0.0)
    assert r.choose_parent(nn, []) is None

    # choose_parent: all costs inf (steer returns Falsey)
    r.node_list = [r.Node(0.0, 0.0)]
    r.steer = lambda *_args, **_kwargs: None  # type: ignore[assignment]
    r.check_collision = lambda *_args, **_kwargs: False  # type: ignore[assignment]
    assert r.choose_parent(nn, [0]) is None

    # rewire: edge_node is None path and no improved cost path
    r.node_list = [r.Node(0.0, 0.0), r.Node(1.0, 0.0)]
    new_node = r.Node(0.5, 0.0)
    # First, edge_node None -> continue
    r.steer = lambda *_args, **_kwargs: None  # type: ignore[assignment]
    r.rewire(new_node, [0])
    # Now, edge_node with no improvement
    tmp_edge = r.Node(0.6, 0.0)
    tmp_edge.cost = 10.0
    r.steer = lambda *_args, **_kwargs: tmp_edge  # type: ignore[assignment]
    r.check_collision = lambda *_args, **_kwargs: True  # type: ignore[assignment]
    r.node_list[0].cost = 0.0

    def _calc_new_cost_stub(_a, _b):
        return 100.0

    r.calc_new_cost = _calc_new_cost_stub  # type: ignore[assignment]
    r.rewire(new_node, [0])

    # search_best_goal_node: no safe goals
    r.calc_dist_to_goal = lambda *_args, **_kwargs: 999.0  # type: ignore[assignment]
    r.steer = lambda *_args, **_kwargs: r.Node(0.0, 0.0)  # type: ignore[assignment]
    assert r.search_best_goal_node() is None


def test_obstacle_acker_instantiation():
    # Covers its __init__ path
    _ = ObstacleAcker()


def test_draw_patch_variants():
    _install_dummy_logger()

    # Set up a simple 2D plot axes
    w = _DummyWorld2D()
    plot = EnvPlot(
        w, objects=[], saved_figure={}, figure_pixels=[200, 150], show_title=False
    )
    ax = plot.ax

    # Base state at origin with zero angle
    state = np.array([[0.0], [0.0], [0.0]])

    # circle
    circ = draw_patch(
        ax, "circle", state=state, radius=1.0, color="k", alpha=0.5, zorder=1
    )
    assert circ is not None

    # rectangle by vertices
    rect_vertices = np.array([[0.0, 1.0, 1.0, 0.0], [0.0, 0.0, 0.5, 0.5]])
    rect1 = draw_patch(ax, "rectangle", state=state, vertices=rect_vertices, color="r")
    assert rect1 is not None

    # rectangle by width/height (place at vertices[0] which is (0,0))
    rect2 = draw_patch(
        ax,
        "rectangle",
        state=state,
        vertices=np.array([[0.0, 0.0]]).T,
        width=1.0,
        height=0.5,
        color="g",
    )
    assert rect2 is not None

    # polygon
    poly_vertices = np.array([[0.0, 0.5, 0.0], [0.0, 0.5, 0.5]])
    poly = draw_patch(ax, "polygon", state=state, vertices=poly_vertices, color="b")
    assert poly is not None

    # ellipse
    ell = draw_patch(ax, "ellipse", state=state, width=0.5, height=0.2, color="c")
    assert ell is not None

    # wedge via theta1/theta2
    wedge1 = draw_patch(
        ax, "wedge", state=state, radius=1.0, theta1=-45.0, theta2=45.0, color="m"
    )
    assert wedge1 is not None

    # wedge via fov (radians)
    wedge2 = draw_patch(ax, "wedge", state=state, radius=1.0, fov=np.pi / 2, color="y")
    assert wedge2 is not None

    # arrow (uses state orientation)
    arr = draw_patch(
        ax, "arrow", state=state, arrow_length=0.3, arrow_width=0.4, color="k"
    )
    assert arr is not None

    # line (2D)
    line_vertices = np.array([[0.0, 1.0], [0.0, 1.0]])
    line = draw_patch(ax, "line", vertices=line_vertices, color="k")
    assert line is not None

    # linestring (alias)
    ls = draw_patch(ax, "linestring", vertices=line_vertices, color="k")
    assert ls is not None

    # invalid shape
    with pytest.raises(ValueError, match="Unsupported shape type"):
        draw_patch(ax, "unknown-shape", state=state)
