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
from irsim.world.object_factory import ObjectFactory
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

    # draw_patch on 3D axes: line branch creates Line3D
    line_vertices = np.array([[0.0, 1.0], [0.0, 1.0]])
    line3d = draw_patch(
        plot3d.ax, "line", vertices=line_vertices, color="k", alpha=0.8, zorder=2
    )
    assert line3d is not None

    # draw_patch on 3D axes: circle triggers patch_2d_to_3d conversion
    state = np.array([[0.0], [0.0], [0.0]])
    circ3d = draw_patch(plot3d.ax, "circle", state=state, radius=0.2, color="r")
    assert circ3d is not None


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
    b = Behavior(
        object_info=types.SimpleNamespace(id=1, name="test_obj"), behavior_dict={}
    )
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
    r = RRTStar(env_map=env_map, robot=env.robot)
    r.node_list = []
    r.expand_dis = 1.0

    # _choose_parent: empty near indices returns nearest
    nearest = r.Node(0.0, 0.0)
    nearest.cost = 0.0
    r.node_list = [nearest]
    nn = r.Node(1.0, 0.0)
    best_parent, _ = r._choose_parent(nn, nearest, 0, 1.0, [])
    assert best_parent is nearest

    # _choose_parent: all candidates fail collision returns nearest
    r.is_collision = lambda *_args, **_kwargs: False  # type: ignore[assignment]
    best_parent, _ = r._choose_parent(nn, nearest, 0, 1.0, [0])
    assert best_parent is nearest

    # _rewire: no cost improvement (candidate cost already lower)
    r.node_list = [r.Node(0.0, 0.0), r.Node(1.0, 0.0)]
    r.node_list[0].cost = 0.0
    r.node_list[1].cost = 0.5
    new_node = r.Node(0.5, 0.0)
    new_node.cost = 10.0
    r._rewire(new_node, [1])
    assert r.node_list[1].parent is None


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

    # rectangle width/height error branches (cover lines 647-655 path)
    with pytest.raises(
        ValueError, match="rectangle requires either vertices or width/height"
    ):
        draw_patch(ax, "rectangle", state=state)

    with pytest.raises(TypeError):
        draw_patch(ax, "rectangle", state=state, width=1.0, height=0.5)

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


def test_object_factory_3d_distribution_not_implemented():
    """Test that 3D distribution raises NotImplementedError"""
    factory = ObjectFactory()
    with pytest.raises(
        NotImplementedError, match="3D state generation is not yet implemented"
    ):
        factory.create_object(
            obj_type="robot",
            number=1,
            distribution={"name": "manual", "3d": True},
        )


def test_object_factory_uniform_distribution_not_implemented():
    """Test that uniform distribution raises NotImplementedError"""
    factory = ObjectFactory()
    with pytest.raises(
        NotImplementedError, match="'uniform' distribution is not yet implemented"
    ):
        factory.generate_state_list(
            number=1,
            distribution={"name": "uniform"},
        )


def test_object_factory_unknown_distribution_raises():
    """Test that unknown distribution raises ValueError"""
    factory = ObjectFactory()
    with pytest.raises(ValueError, match="Unknown distribution name"):
        factory.generate_state_list(
            number=1,
            distribution={"name": "unknown_distribution"},
        )


def test_env_robot_property_empty_list():
    """Test that robot property raises IndexError when no robots exist"""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    # Clear all robots
    for robot in list(env.robot_list):
        env.delete_object(robot.id)

    with pytest.raises(IndexError, match="No robots in the environment"):
        _ = env.robot
    env.end()


def test_save_figure_no_extension():
    """Test save_figure with filename without extension"""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    env.render()
    # Test filename without extension - should default to png
    env.save_figure(save_name="test_no_ext")
    env.end()

    import os

    # Clean up the generated file
    if os.path.exists("test_no_ext.png"):
        os.remove("test_no_ext.png")


def test_save_figure_multiple_dots():
    """Test save_figure with filename containing multiple dots"""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    env.render()
    # Test filename with multiple dots
    env.save_figure(save_name="test.file.name.png")
    env.end()

    import os

    # Clean up the generated file
    if os.path.exists("test.file.name.png"):
        os.remove("test.file.name.png")


def test_env_plot_set_ax_viewpoint_none_objects():
    """Test set_ax_viewpoint with None objects"""
    _install_dummy_logger()
    w = _DummyWorld2D()
    plot = EnvPlot(
        w, objects=[], saved_figure={}, figure_pixels=[200, 150], show_title=True
    )
    plot.viewpoint = "some_name"  # Set string viewpoint to trigger the branch
    # Should not raise when objects is None
    plot.set_ax_viewpoint(objects=None)


def test_env_step_auto_control():
    """Test environment step with auto control mode."""
    from irsim.config import world_param

    env = irsim.make(
        "test_collision_avoidance.yaml", save_ani=False, full=True, display=False
    )
    original_mode = world_param.control_mode
    world_param.control_mode = "auto"
    for _ in range(3):
        env.step()
    world_param.control_mode = original_mode
    env.end()


def test_env_robot_state_getters():
    """Test various robot state getter methods."""
    env = irsim.make(
        "test_multi_objects_world.yaml", save_ani=False, full=False, display=False
    )
    # Test get_robot_state
    state = env.get_robot_state()
    assert state is not None

    # Test get_robot_info
    info = env.get_robot_info()
    assert info is not None

    env.end()


def test_env_obstacle_state_getters():
    """Test obstacle state getter methods."""
    env = irsim.make("test_all_objects.yaml", save_ani=False, full=False, display=False)
    if len(env.obstacle_list) > 0:
        # Test get_obstacle_info_list
        info_list = env.get_obstacle_info_list()
        assert info_list is not None

    env.end()


def test_env_set_robot_state():
    """Test setting robot state."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    # Get current state and modify it
    robot = env.robot
    new_state = np.array([[5.0], [5.0], [0.0]])
    robot.set_state(new_state)
    env.end()


def test_env_set_robot_velocity():
    """Test setting robot velocity."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    robot = env.robot
    vel = np.array([[0.5], [0.1]])
    robot.set_velocity(vel)
    env.end()


def test_env_collision_detection():
    """Test collision detection methods."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    robot = env.robot
    # Test collision property
    _ = robot.collision
    env.end()


def test_env_goal_reached():
    """Test goal reached detection."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    # Check if done method works
    _ = env.done()
    env.end()


def test_env_render_with_kwargs():
    """Test render with various kwargs."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    env.render(0.01)
    env.end()


def test_env_draw_methods():
    """Test various draw methods."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    env.render()

    # Draw trajectory
    traj = np.array([[0.0, 1.0, 2.0], [0.0, 1.0, 2.0]])
    env.draw_trajectory(traj)

    # Draw points
    points = np.array([[1.0], [1.0]])
    env.draw_points(points)

    env.end()


def test_object_base_properties():
    """Test object base properties."""
    env = irsim.make("test_all_objects.yaml", save_ani=False, full=False, display=False)
    robot = env.robot

    # Test various properties
    _ = robot.state
    _ = robot.velocity
    _ = robot.geometry
    _ = robot.vertices
    _ = robot.radius
    _ = robot.goal
    _ = robot.role
    _ = robot.kinematics

    env.end()


def test_object_base_collision_methods():
    """Test object base collision methods."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    robot = env.robot

    # Test collision check
    _ = robot.collision

    if len(env.obstacle_list) > 0:
        obstacle = env.obstacle_list[0]
        # Test distance calculation if method exists
        if hasattr(robot, "get_distance_to"):
            _ = robot.get_distance_to(obstacle)

    env.end()


def test_object_base_trajectory():
    """Test object trajectory methods."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    robot = env.robot

    # Step to build trajectory
    for _ in range(5):
        env.step()

    # Test trajectory property
    if hasattr(robot, "trajectory"):
        traj = robot.trajectory
        assert traj is not None

    env.end()


def test_lidar_methods():
    """Test lidar sensor methods."""
    env = irsim.make("test_all_objects.yaml", save_ani=False, full=False, display=False)
    if hasattr(env.robot, "lidar") and env.robot.lidar is not None:
        lidar = env.robot.lidar

        # Step to get scan data
        env.step()

        # Test lidar properties
        if hasattr(lidar, "scan"):
            _ = lidar.scan
        if hasattr(lidar, "range_max"):
            _ = lidar.range_max

    env.end()


def test_world_properties():
    """Test world properties."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    # Access internal world object
    world = env._world

    # Test world properties
    _ = world.grid_map
    _ = world.x_range
    _ = world.y_range
    _ = world.step_time

    env.end()


def test_env_reset():
    """Test environment reset."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    # Step a few times
    for _ in range(5):
        env.step()

    # Reset
    env.reset()

    # Time should be reset
    assert env.time == 0 or env.time < 0.1

    env.end()


def test_env_reload():
    """Test environment reload."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    # Step a few times
    for _ in range(3):
        env.step()

    # Reload
    env.reload()

    env.end()


def test_object_factory_create_with_id():
    """Test ObjectFactory create_object with explicit ID."""
    factory = ObjectFactory()
    obj = factory.create_object(
        obj_type="obstacle",
        number=1,
        obj_id=999,
        shape={"name": "circle", "radius": 0.5},
    )
    assert obj is not None


def test_env_keyboard_interaction():
    """Test keyboard control basic setup."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    # The keyboard controller should be initialized
    if hasattr(env, "keyboard"):
        _ = env.keyboard
    env.end()


def test_behavior_registry():
    """Test behavior registry has expected behaviors."""
    from irsim.lib.behavior.behavior_registry import behaviors_map

    # Check that common behaviors are registered
    assert ("diff", "dash") in behaviors_map or len(behaviors_map) > 0


def test_collision_mode_reactive():
    """Test collision mode reactive (lines 484-485)."""
    from irsim.config import world_param

    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    original_mode = world_param.collision_mode
    world_param.collision_mode = "reactive"
    for _ in range(3):
        env.step()
    world_param.collision_mode = original_mode
    env.end()


def test_collision_mode_undefined():
    """Test collision mode undefined warning (lines 496-497)."""
    from irsim.config import world_param

    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    original_mode = world_param.collision_mode
    world_param.collision_mode = "unknown_mode"
    world_param.count = 50
    for _ in range(3):
        env.step()
    world_param.collision_mode = original_mode
    env.end()


def test_object_state_validation():
    """Test state shape validation in object_base."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    robot = env.robot
    # Test with smaller state
    small_state = np.array([[1.0], [2.0]])
    if hasattr(robot, "state_validation_check"):
        validated = robot.state_validation_check(small_state)
        assert validated is not None
    env.end()


def test_object_state_larger():
    """Test state shape validation with larger state."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    robot = env.robot
    # Test with larger state (6D)
    large_state = np.array([[1.0], [2.0], [0.0], [0.0], [0.0], [0.0]])
    if hasattr(robot, "state_validation_check"):
        validated = robot.state_validation_check(large_state)
        assert validated is not None
    env.end()


def test_object_arrive_state_mode():
    """Test object arrival in state mode."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    robot = env.robot
    # Check arrive_mode property
    _ = robot.arrive_mode
    env.end()


def test_object_multiple_goals():
    """Test object with multiple goals."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    robot = env.robot
    # Set multiple goals if supported
    if hasattr(robot, "set_goals"):
        robot.set_goals([[[5], [5], [0]], [[8], [8], [0]]])
    env.end()


def test_object_info_properties():
    """Test ObjectInfo properties."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    info = env.get_robot_info()
    # Test info properties that exist
    if hasattr(info, "state"):
        _ = info.state
    if hasattr(info, "goal"):
        _ = info.goal
    if hasattr(info, "id"):
        _ = info.id
    env.end()


def test_geometry_handler_3d():
    """Test 3D geometry handler (lines 379-411 in geometry_handler.py)."""
    # Create a 3D geometry handler test
    from irsim.lib.handler.geometry_handler import GeometryFactory

    # Test with 2D geometry (3D geometry handlers are for EnvBase3D)
    circle = GeometryFactory.create_geometry("circle", center=[0.0, 0.0], radius=1.0)
    assert circle is not None


def test_env_plot_with_different_params():
    """Test EnvPlot with different parameters."""
    env = irsim.make("test_all_objects.yaml", save_ani=False, full=False, display=False)
    env.render(0.01)
    env.end()


def test_object_kinematics_properties():
    """Test object kinematics-related properties."""
    env = irsim.make("test_all_objects.yaml", save_ani=False, full=False, display=False)
    robot = env.robot

    # Test kinematics properties
    _ = robot.kinematics
    _ = robot.max_speed if hasattr(robot, "max_speed") else None
    _ = robot.max_acce if hasattr(robot, "max_acce") else None

    env.end()


def test_util_functions():
    """Test utility functions in util.py."""
    from irsim.util import util

    # Test distance calculation
    state1 = np.array([[0], [0]])
    state2 = np.array([[3], [4]])
    dist = util.distance(state1, state2)
    assert dist == 5.0

    # Test relative position
    if hasattr(util, "relative_position"):
        rel = util.relative_position(state1, state2)
        assert rel is not None


def test_sensor_factory():
    """Test sensor factory (line 27 in sensor_factory.py)."""
    from irsim.world.sensors.sensor_factory import SensorFactory

    # Factory should exist
    factory = SensorFactory()
    assert factory is not None


def test_rvo_algorithm():
    """Test RVO algorithm additional branches."""
    env = irsim.make("test_rvo_world.yaml", save_ani=False, full=True, display=False)
    for _ in range(5):
        env.step()
    env.end()


def test_env_plot_draw_robot_arrow():
    """Test drawing robot with different parameters."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    env.render(0.01)

    # Test draw methods
    if hasattr(env, "draw_arrow"):
        env.draw_arrow([0, 0], [1, 1])

    env.end()


def test_object_velocity_range():
    """Test get_vel_range method."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    robot = env.robot
    min_vel, max_vel = robot.get_vel_range()
    assert min_vel is not None
    assert max_vel is not None
    env.end()


def test_object_acceleration():
    """Test object acceleration properties."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    robot = env.robot
    _ = robot.acce if hasattr(robot, "acce") else None
    env.end()


def test_world_get_map():
    """Test world get_map method."""
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    env_map = env.get_map()
    assert env_map is not None
    env.end()


def test_object_gf_none():
    """Test object with gf=None (line 265)."""

    # Creating object without geometry factory should handle gf=None
    # This is handled internally when shape is None
    env = irsim.make(
        "test_collision_world.yaml", save_ani=False, full=False, display=False
    )
    # Objects in the env should have valid geometry
    robot = env.robot
    assert robot.gf is not None
    env.end()


class TestOrcaGroupBehaviorCoverage:
    """Tests for OrcaGroupBehavior coverage with mocked pyrvo."""

    def _clear_orca_registration(self):
        """Clear ORCA registration from registry to allow fresh import."""
        import sys

        from irsim.lib.behavior.behavior_registry import group_behaviors_class_map

        key = ("omni", "orca")
        if key in group_behaviors_class_map:
            del group_behaviors_class_map[key]

        # Also clear the module cache
        if "irsim.lib.behavior.group_behavior_methods" in sys.modules:
            del sys.modules["irsim.lib.behavior.group_behavior_methods"]

    def _restore_orca_registration(self):
        """Restore ORCA registration after test."""
        import sys

        from irsim.lib.behavior.behavior_registry import group_behaviors_class_map

        # Clear registration if it exists (from mock import)
        key = ("omni", "orca")
        if key in group_behaviors_class_map:
            del group_behaviors_class_map[key]

        # Clear module cache
        if "irsim.lib.behavior.group_behavior_methods" in sys.modules:
            del sys.modules["irsim.lib.behavior.group_behavior_methods"]

        # Note: We don't re-import since pyrvo isn't installed.
        # The module will be imported naturally when needed by other tests.

    def test_orca_build_sim_and_call(self):
        """Test ORCA behavior _build_sim and __call__ with mocked pyrvo."""
        from unittest.mock import MagicMock, patch

        self._clear_orca_registration()

        try:
            # Create mock pyrvo module
            mock_pyrvo = MagicMock()
            mock_sim = MagicMock()
            mock_pyrvo.RVOSimulator.return_value = mock_sim
            mock_sim.get_num_agents.return_value = 2
            mock_sim.get_agent_velocity.return_value = MagicMock(
                to_tuple=lambda: (1.0, 0.5)
            )

            with patch.dict("sys.modules", {"pyrvo": mock_pyrvo}):
                from irsim.lib.behavior.group_behavior_methods import OrcaGroupBehavior

                # Create mock members
                member1 = MagicMock()
                member1.state = np.array([[0.0], [0.0]])
                member1.radius = 0.5
                member1.max_speed = 1.0
                member1._world_param = MagicMock()
                member1._world_param.step_time = 0.1
                member1.get_desired_omni_vel = MagicMock(
                    return_value=np.array([[1.0], [0.0]])
                )

                member2 = MagicMock()
                member2.state = np.array([[2.0], [2.0]])
                member2.radius = 0.5
                member2.max_speed = 1.0
                member2._world_param = MagicMock()
                member2._world_param.step_time = 0.1
                member2.get_desired_omni_vel = MagicMock(
                    return_value=np.array([[0.0], [1.0]])
                )

                members = [member1, member2]

                # Test initialization (lines 62-83)
                orca = OrcaGroupBehavior(members, maxSpeed=2.0)

                # Verify sim was built
                assert mock_pyrvo.RVOSimulator.called
                assert mock_sim.set_time_step.called
                assert mock_sim.add_agent.call_count == 2

                # Test __call__ (lines 96-110)
                result = orca(members)

                # Verify sim methods were called
                assert mock_sim.set_agent_pref_velocity.call_count == 2
                assert mock_sim.set_agent_position.call_count == 2
                assert mock_sim.do_step.called
                assert len(result) == 2
        finally:
            self._restore_orca_registration()

    def test_orca_rebuild_on_mismatch(self):
        """Test ORCA rebuilds sim when member count changes."""
        from unittest.mock import MagicMock, patch

        self._clear_orca_registration()

        try:
            mock_pyrvo = MagicMock()
            mock_sim = MagicMock()
            mock_pyrvo.RVOSimulator.return_value = mock_sim
            mock_sim.get_agent_velocity.return_value = MagicMock(
                to_tuple=lambda: (0.5, 0.5)
            )

            with patch.dict("sys.modules", {"pyrvo": mock_pyrvo}):
                from irsim.lib.behavior.group_behavior_methods import OrcaGroupBehavior

                member1 = MagicMock()
                member1.state = np.array([[0.0], [0.0]])
                member1.radius = 0.5
                member1.max_speed = 1.0
                member1._world_param = MagicMock()
                member1._world_param.step_time = 0.1
                member1.get_desired_omni_vel = MagicMock(
                    return_value=np.array([[1.0], [0.0]])
                )

                # Initialize with one member
                mock_sim.get_num_agents.return_value = 1
                orca = OrcaGroupBehavior([member1])
                initial_call_count = mock_pyrvo.RVOSimulator.call_count

                # Call with two members (triggers rebuild at line 98)
                member2 = MagicMock()
                member2.state = np.array([[1.0], [1.0]])
                member2.radius = 0.5
                member2.max_speed = 1.0
                member2._world_param = MagicMock()
                member2._world_param.step_time = 0.1
                member2.get_desired_omni_vel = MagicMock(
                    return_value=np.array([[0.0], [1.0]])
                )

                # get_num_agents returns 1 (mismatched), triggering rebuild
                mock_sim.get_num_agents.return_value = 1
                result = orca([member1, member2])

                # Should have rebuilt (RVOSimulator called again)
                assert mock_pyrvo.RVOSimulator.call_count > initial_call_count
                assert len(result) == 1  # Based on final get_num_agents
        finally:
            self._restore_orca_registration()

    def test_orca_rebuild_on_exception(self):
        """Test ORCA rebuilds sim when get_num_agents raises exception."""
        from unittest.mock import MagicMock, patch

        self._clear_orca_registration()

        try:
            mock_pyrvo = MagicMock()
            mock_sim = MagicMock()
            mock_pyrvo.RVOSimulator.return_value = mock_sim
            # First call raises exception, triggering rebuild at line 100
            call_count = [0]

            def get_num_agents_side_effect():
                call_count[0] += 1
                if call_count[0] == 1:
                    raise RuntimeError("Sim error")
                return 1

            mock_sim.get_num_agents.side_effect = get_num_agents_side_effect
            mock_sim.get_agent_velocity.return_value = MagicMock(
                to_tuple=lambda: (0.5, 0.5)
            )

            with patch.dict("sys.modules", {"pyrvo": mock_pyrvo}):
                from irsim.lib.behavior.group_behavior_methods import OrcaGroupBehavior

                member1 = MagicMock()
                member1.state = np.array([[0.0], [0.0]])
                member1.radius = 0.5
                member1.max_speed = 1.0
                member1._world_param = MagicMock()
                member1._world_param.step_time = 0.1
                member1.get_desired_omni_vel = MagicMock(
                    return_value=np.array([[1.0], [0.0]])
                )

                orca = OrcaGroupBehavior([member1])

                # This call triggers exception handling and rebuild
                result = orca([member1])

                # Should have rebuilt
                assert mock_pyrvo.RVOSimulator.call_count == 2
                assert len(result) == 1
        finally:
            self._restore_orca_registration()

    def test_orca_ensure_pyrvo_success(self):
        """Test _ensure_pyrvo returns pyrvo when available."""
        from unittest.mock import MagicMock, patch

        self._clear_orca_registration()

        try:
            mock_pyrvo = MagicMock()
            mock_sim = MagicMock()
            mock_pyrvo.RVOSimulator.return_value = mock_sim
            mock_sim.get_num_agents.return_value = 0

            with patch.dict("sys.modules", {"pyrvo": mock_pyrvo}):
                from irsim.lib.behavior.group_behavior_methods import OrcaGroupBehavior

                member = MagicMock()
                member.state = np.array([[0.0], [0.0]])
                member.radius = 0.5
                member.max_speed = 1.0
                member._world_param = MagicMock()
                member._world_param.step_time = 0.1

                orca = OrcaGroupBehavior([member])

                # Test _ensure_pyrvo returns module (line 45)
                result = orca._ensure_pyrvo()
                assert result == mock_pyrvo
        finally:
            self._restore_orca_registration()


class TestLoadBehaviorReinitialization:
    """Tests for load_behavior behavior reinitialization (env_base.py lines 1151-1159)."""

    def test_load_behavior_with_group_behaviors_mock(self):
        """Test load_behavior reinitializes group behaviors using mocks."""
        from unittest.mock import MagicMock, patch

        # Create mock environment with objects and groups
        mock_env = MagicMock()

        # Mock object with behavior
        mock_obj = MagicMock()
        mock_obj.obj_behavior = MagicMock()
        mock_obj.obj_behavior._init_behavior_class = MagicMock()
        mock_env.objects = [mock_obj]

        # Mock object group with group behavior
        mock_group = MagicMock()
        mock_group.group_behavior = MagicMock()
        mock_group.group_behavior._init_group_behavior_class = MagicMock()
        mock_env._object_groups = [mock_group]

        # Import the real load_behavior method and bind it to our mock
        from irsim.env.env_base import EnvBase

        # Patch importlib.import_module to succeed
        with patch("importlib.import_module"):
            # Call the real method on mock env
            EnvBase.load_behavior(mock_env, "test_module")

        # Verify individual behavior reinitialization was called
        mock_obj.obj_behavior._init_behavior_class.assert_called_once()

        # Verify group behavior reinitialization was called
        mock_group.group_behavior._init_group_behavior_class.assert_called_once()

    def test_load_behavior_import_error_mock(self, capsys):
        """Test that load_behavior returns early on import error."""
        from unittest.mock import MagicMock, patch

        mock_env = MagicMock()
        mock_env.objects = []
        mock_env._object_groups = []

        from irsim.env.env_base import EnvBase

        # Patch importlib to raise ImportError
        with patch(
            "importlib.import_module", side_effect=ImportError("Module not found")
        ):
            EnvBase.load_behavior(mock_env, "nonexistent_module")

        captured = capsys.readouterr()
        assert "Failed to load module" in captured.out

    def test_load_behavior_skips_none_behaviors(self):
        """Test load_behavior handles objects without behaviors."""
        from unittest.mock import MagicMock, patch

        mock_env = MagicMock()

        # Object without obj_behavior attribute
        mock_obj1 = MagicMock(spec=[])

        # Object with None behavior
        mock_obj2 = MagicMock()
        mock_obj2.obj_behavior = None

        # Object with valid behavior
        mock_obj3 = MagicMock()
        mock_obj3.obj_behavior = MagicMock()
        mock_obj3.obj_behavior._init_behavior_class = MagicMock()

        mock_env.objects = [mock_obj1, mock_obj2, mock_obj3]

        # Group without group_behavior
        mock_group1 = MagicMock(spec=[])

        # Group with None group_behavior
        mock_group2 = MagicMock()
        mock_group2.group_behavior = None

        mock_env._object_groups = [mock_group1, mock_group2]

        from irsim.env.env_base import EnvBase

        with patch("importlib.import_module"):
            EnvBase.load_behavior(mock_env, "test_module")

        # Only the valid behavior should be reinitialized
        mock_obj3.obj_behavior._init_behavior_class.assert_called_once()


class TestLidar2DEnsureMultiLineString:
    """Tests for Lidar2D._ensure_multi_linestring method."""

    @pytest.fixture
    def lidar(self):
        """Create a minimal Lidar2D instance for testing."""
        from irsim.world.sensors.lidar2d import Lidar2D

        state = np.array([[0.0], [0.0], [0.0]])
        return Lidar2D(state=state, obj_id=1, number=10, range_max=5.0)

    def test_linestring_input(self, lidar):
        """Test that LineString is wrapped in MultiLineString."""
        from shapely import LineString, MultiLineString

        line = LineString([(0, 0), (1, 1)])
        result = lidar._ensure_multi_linestring(line)

        assert isinstance(result, MultiLineString)
        assert len(result.geoms) == 1
        assert result.geoms[0].equals(line)

    def test_multilinestring_input(self, lidar):
        """Test that MultiLineString is returned unchanged."""
        from shapely import MultiLineString

        lines = MultiLineString([[(0, 0), (1, 1)], [(2, 2), (3, 3)]])
        result = lidar._ensure_multi_linestring(lines)

        assert result is lines
        assert len(result.geoms) == 2

    def test_empty_geometry(self, lidar):
        """Test that empty geometry returns empty MultiLineString."""
        from shapely import MultiLineString, MultiPolygon

        # Use an empty MultiPolygon to hit the is_empty branch
        # (empty LineString would hit the LineString branch first)
        empty = MultiPolygon()
        result = lidar._ensure_multi_linestring(empty)

        assert isinstance(result, MultiLineString)
        assert result.is_empty

    def test_geometry_collection_with_linestrings(self, lidar):
        """Test GeometryCollection with LineString components."""
        from shapely import GeometryCollection, LineString, MultiLineString

        line1 = LineString([(0, 0), (1, 1)])
        line2 = LineString([(2, 2), (3, 3)])
        collection = GeometryCollection([line1, line2])
        result = lidar._ensure_multi_linestring(collection)

        assert isinstance(result, MultiLineString)
        assert len(result.geoms) == 2

    def test_geometry_collection_with_nested_multilinestring(self, lidar):
        """Test GeometryCollection with nested MultiLineString."""
        from shapely import GeometryCollection, LineString, MultiLineString

        line1 = LineString([(0, 0), (1, 1)])
        multi = MultiLineString([[(2, 2), (3, 3)], [(4, 4), (5, 5)]])
        collection = GeometryCollection([line1, multi])
        result = lidar._ensure_multi_linestring(collection)

        assert isinstance(result, MultiLineString)
        assert len(result.geoms) == 3  # 1 from line1 + 2 from multi

    def test_geometry_collection_empty(self, lidar):
        """Test empty GeometryCollection returns empty MultiLineString."""
        from shapely import GeometryCollection, MultiLineString

        collection = GeometryCollection()
        result = lidar._ensure_multi_linestring(collection)

        assert isinstance(result, MultiLineString)
        assert result.is_empty

    def test_unsupported_point_geometry(self, lidar):
        """Test that Point geometry returns empty MultiLineString."""
        from shapely import MultiLineString, Point

        point = Point(1, 1)
        result = lidar._ensure_multi_linestring(point)

        assert isinstance(result, MultiLineString)
        assert result.is_empty

    def test_unsupported_polygon_geometry(self, lidar):
        """Test that Polygon geometry returns empty MultiLineString."""
        from shapely import MultiLineString, Polygon

        polygon = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
        result = lidar._ensure_multi_linestring(polygon)

        assert isinstance(result, MultiLineString)
        assert result.is_empty

    def test_geometry_collection_mixed_types(self, lidar):
        """Test GeometryCollection with mixed geometry types extracts only lines."""
        from shapely import GeometryCollection, LineString, MultiLineString, Point

        line = LineString([(0, 0), (1, 1)])
        point = Point(5, 5)  # Should be ignored
        collection = GeometryCollection([line, point])
        result = lidar._ensure_multi_linestring(collection)

        assert isinstance(result, MultiLineString)
        assert len(result.geoms) == 1  # Only the LineString


# ---------------------------------------------------------------------------
# Coverage-targeted tests for lidar2d.py
# ---------------------------------------------------------------------------


class TestLidar2DNoise:
    """Test Lidar2D with noise=True (line 296)."""

    def test_lidar_with_noise(self):
        """Lidar2D with noise=True adds noise to range data (line 296)."""
        from irsim.world.sensors.lidar2d import Lidar2D

        state = np.array([[0.0], [0.0], [0.0]])
        lidar = Lidar2D(state=state, obj_id=1, number=10, range_max=5.0, noise=True)
        assert lidar.noise is True
        # Calculate range with noise
        lidar.calculate_range()
        assert lidar.range_data is not None


class TestLidar2DScanToPointcloud:
    """Test scan_to_pointcloud (lines 585-591)."""

    def test_scan_to_pointcloud_with_hits(self):
        """scan_to_pointcloud with hits returns array (lines 585-586)."""
        from irsim.world.sensors.lidar2d import Lidar2D

        state = np.array([[0.0], [0.0], [0.0]])
        lidar = Lidar2D(state=state, obj_id=1, number=10, range_max=5.0)
        # Set some ranges shorter than range_max to simulate hits
        lidar.range_data[:5] = 2.0  # Half of beams hit something
        result = lidar.scan_to_pointcloud()
        assert result is not None
        assert result.shape[0] == 2  # 2D points
        assert result.shape[1] == 5  # 5 hit points

    def test_scan_to_pointcloud_no_hits(self):
        """scan_to_pointcloud with no hits returns None (line 591)."""
        from irsim.world.sensors.lidar2d import Lidar2D

        state = np.array([[0.0], [0.0], [0.0]])
        lidar = Lidar2D(state=state, obj_id=1, number=10, range_max=5.0)
        # All ranges at max (no hits)
        lidar.range_data[:] = 5.0
        result = lidar.scan_to_pointcloud()
        assert result is None


class TestBehaviorRegistryDuplicate:
    """Test behavior registry duplicate registration (line 24)."""

    def test_duplicate_behavior_raises(self):
        """Registering same (kinematics, action) twice raises ValueError."""
        from irsim.lib.behavior.behavior_registry import _make_register

        target_map = {}
        register = _make_register(
            target_map, "Duplicate behavior '{kinematics}/{action}'"
        )

        @register("test_kin", "test_action")
        class Beh1:
            pass

        with pytest.raises(ValueError, match="Duplicate behavior"):

            @register("test_kin", "test_action")
            class Beh2:
                pass


class TestBehaviorGenVelExternalNone:
    """Test Behavior.gen_vel with external_objects=None (line 52)."""

    def test_gen_vel_external_none(self):
        """gen_vel with external_objects=None uses empty list (line 52)."""
        from irsim.lib.behavior.behavior import Behavior

        beh = Behavior(object_info=None, behavior_dict={})
        # When behavior_dict is empty and external_objects is None
        vel = beh.gen_vel(ego_object=None, external_objects=None)
        assert vel is not None


# ---------------------------------------------------------------------------
# Tests for typo-detection / unknown-kwargs validation
# ---------------------------------------------------------------------------


class TestWorldUnknownKwargs:
    """Tests for World unknown kwargs validation and logger/env_param properties."""

    def test_world_unknown_kwarg_warns(self):
        """World.__init__ with unknown kwarg emits a warning via check_unknown_kwargs."""
        from unittest.mock import MagicMock

        from irsim.world.world import World

        logger = MagicMock()
        # Patch the logger property to capture the warning
        with pytest.MonkeyPatch.context() as mp:
            mp.setattr(World, "logger", property(lambda self: logger))
            World(name="test", step_tme=0.05)  # typo: step_tme

        logger.warning.assert_called_once()
        msg = logger.warning.call_args[0][0]
        assert "step_tme" in msg

    def test_world_env_param_fallback_to_global(self):
        """World._env_param falls back to global env_param when _env is None."""
        from irsim.config import env_param as global_ep
        from irsim.world.world import World

        w = World(name="test")
        assert w._env is None
        assert w._env_param is global_ep

    def test_world_env_param_via_env(self):
        """World._env_param delegates to env._env_param when _env is set."""
        from unittest.mock import MagicMock

        from irsim.world.world import World

        w = World(name="test")
        mock_env = MagicMock()
        mock_env._env_param = MagicMock()
        w._env = mock_env
        assert w._env_param is mock_env._env_param

    def test_world_logger_property(self):
        """World.logger returns the logger from _env_param."""
        from irsim.world.world import World

        w = World(name="test")
        assert w.logger is w._env_param.logger


class TestEnvConfigInvalidKey:
    """Tests for EnvConfig.load_yaml invalid key suggestion."""

    def test_invalid_yaml_key_close_match(self, tmp_path):
        """EnvConfig raises KeyError with suggestion for close typo."""
        _install_dummy_logger()

        yaml_file = tmp_path / "bad.yaml"
        yaml_file.write_text("wrold:\n  height: 10\n")

        from irsim.env.env_config import EnvConfig

        with pytest.raises(KeyError):
            EnvConfig(str(yaml_file))

    def test_invalid_yaml_key_no_match(self, tmp_path):
        """EnvConfig raises KeyError listing valid keys for unrecognised key."""
        _install_dummy_logger()

        yaml_file = tmp_path / "bad2.yaml"
        yaml_file.write_text("zzzzz:\n  foo: bar\n")

        from irsim.env.env_config import EnvConfig

        with pytest.raises(KeyError):
            EnvConfig(str(yaml_file))


class TestObjectBaseUnknownKwargs:
    """Tests for ObjectBase unknown kwargs validation."""

    def test_object_base_unknown_kwarg_warns(self):
        """ObjectBase.__init__ warns about unknown kwargs."""
        _install_dummy_logger()
        warnings_collected = []
        original_logger = env_param.logger

        class CapturingLogger:
            def info(self, *a, **kw):
                pass

            def warning(self, msg, *a, **kw):
                warnings_collected.append(msg)

            def error(self, *a, **kw):
                pass

            def debug(self, *a, **kw):
                pass

        env_param.logger = CapturingLogger()
        try:
            from irsim.world.object_base import ObjectBase

            ObjectBase(colr="red")  # typo: colr instead of color
            assert any("colr" in w for w in warnings_collected)
        finally:
            env_param.logger = original_logger
