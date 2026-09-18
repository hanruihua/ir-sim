"""
Tests for the ``contact`` collision mode: per-object ``mass``, the SAT contact
solver, and mass-shared pushing in the environment.
"""

from pathlib import Path

import numpy as np
import pytest
import shapely
from shapely.geometry import LineString, Polygon

from irsim.lib.algorithm.contact import (
    CONTACT_SLOP,
    geometry_pieces,
    object_mtv,
    object_pieces,
    piece_mtv,
    resolve_contacts,
)
from irsim.lib.handler.kinematics_handler import DifferentialKinematics
from irsim.world.object_base import ObjectBase
from irsim.world.object_factory import DYNAMIC_BODY_COLOR

CAVE_PNG = Path(__file__).parent / "cave.png"


@pytest.fixture(autouse=True)
def _quiet_logger(dummy_logger):
    """Objects built outside an environment log through the shared params."""
    return dummy_logger


def _yaml(tmp_path, text, name="contact_world.yaml"):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


def _push_world(box_mass=1.0, wall=True, mode="contact", robot_mass=1.0):
    wall_block = (
        "  - shape: {name: 'rectangle', length: 0.4, width: 6}\n    state: [8, 5, 0]\n"
        if wall
        else ""
    )
    return (
        "world:\n"
        "  height: 10\n"
        "  width: 10\n"
        "  step_time: 0.1\n"
        f"  collision_mode: '{mode}'\n"
        "robot:\n"
        "  - kinematics: {name: 'diff'}\n"
        "    shape: {name: 'circle', radius: 0.3}\n"
        "    state: [1, 5, 0]\n"
        "    goal: [9.5, 5, 0]\n"
        "    vel_max: [1.0, 1.0]\n"
        f"    mass: {robot_mass}\n"
        "obstacle:\n"
        "  - shape: {name: 'rectangle', length: 0.8, width: 0.8}\n"
        "    state: [3, 5, 0]\n"
        f"    mass: {box_mass}\n" + wall_block
    )


def _circle(cx, cy, r):
    return ("circle", np.array([cx, cy], dtype=float), float(r))


def _square(x0, y0, side):
    return (
        "polygon",
        np.array(
            [[x0, y0], [x0 + side, y0], [x0 + side, y0 + side], [x0, y0 + side]],
            dtype=float,
        ),
    )


class TestPieceMTV:
    """Separating-axis minimum translation between convex pieces."""

    def test_circle_circle(self):
        normal, depth = piece_mtv(_circle(0, 0, 1), _circle(1.5, 0, 1))
        assert np.allclose(normal, [-1, 0])
        assert depth == pytest.approx(0.5)

    def test_circle_polygon(self):
        normal, depth = piece_mtv(_circle(1.2, 0.5, 0.3), _square(0, 0, 1))
        assert np.allclose(normal, [1, 0])
        assert depth == pytest.approx(0.1)

    def test_polygon_polygon(self):
        normal, depth = piece_mtv(_square(0.9, 0.2, 1), _square(0, 0, 1))
        assert np.allclose(normal, [1, 0])
        assert depth == pytest.approx(0.1)

    def test_circle_straddling_segment_pushes_out_the_short_way(self):
        wall = ("polygon", np.array([[-5.0, 0.5], [5.0, 0.5]]))
        normal, depth = piece_mtv(_circle(0, 0.25, 0.3), wall)
        assert np.allclose(normal, [0, -1])
        assert depth == pytest.approx(0.05)

    def test_containment_exits_through_nearest_side(self):
        normal, depth = piece_mtv(_square(0.7, 0.4, 0.2), _square(0, 0, 1))
        assert np.allclose(normal, [1, 0])
        assert depth == pytest.approx(0.3)

    def test_separated_is_none(self):
        assert piece_mtv(_circle(0, 0, 1), _circle(3, 0, 1)) is None
        assert piece_mtv(_square(2, 2, 1), _square(0, 0, 1)) is None

    def test_touching_is_zero_depth_contact(self):
        _, depth = piece_mtv(_square(1, 0, 1), _square(0, 0, 1))
        assert depth == pytest.approx(0.0)

    def test_degenerate_axes_give_none(self):
        point = ("polygon", np.array([[0.0, 0.0], [0.0, 0.0]]))
        assert piece_mtv(_circle(0, 0, 1), point) is None


class TestPieces:
    """Convex decomposition of shapely geometries and objects."""

    def test_convex_polygon_is_one_piece(self):
        pieces = geometry_pieces(shapely.box(0, 0, 1, 1))
        assert len(pieces) == 1
        assert pieces[0][1].shape == (4, 2)

    def test_non_convex_polygon_is_triangulated(self):
        arrow = Polygon([(0, 0), (1, 0), (1, 1), (0.5, 0.3), (0, 1)])
        pieces = geometry_pieces(arrow)
        assert len(pieces) >= 2
        assert all(p[1].shape == (3, 2) for p in pieces)
        assert sum(Polygon(p[1]).area for p in pieces) == pytest.approx(arrow.area)

    def test_polygon_with_hole_is_triangulated(self):
        ring = shapely.box(0, 0, 3, 3).difference(shapely.box(1, 1, 2, 2))
        pieces = geometry_pieces(ring)
        assert len(pieces) >= 4
        assert sum(Polygon(p[1]).area for p in pieces) == pytest.approx(ring.area)

    def test_line_becomes_segments_filtered_by_bounds(self):
        line = LineString([(0, 0), (1, 0), (2, 0), (3, 0)])
        assert len(geometry_pieces(line)) == 3
        assert len(geometry_pieces(line, bounds=(0.2, -1, 0.8, 1))) == 1
        assert geometry_pieces(LineString()) == []
        assert geometry_pieces(shapely.Point(0, 0)) == []
        assert geometry_pieces(shapely.GeometryCollection([])) == []

    def test_zero_length_segment_has_no_axes_but_still_contacts(self):
        pieces = geometry_pieces(LineString([(0, 0), (0, 0), (1, 0)]))
        assert len(pieces) == 2
        assert pieces[0][2].shape == (0, 2)
        assert pieces[1][2].shape == (1, 2)
        # the degenerate piece acts as a point: a circle over it is pushed off it
        normal, depth = piece_mtv(_circle(0.1, 0, 0.5), pieces[0])
        assert np.allclose(normal, [1, 0])
        assert depth == pytest.approx(0.4)
        assert piece_mtv(pieces[0], pieces[0]) is None

    def test_object_pieces_by_shape(self):
        circle = ObjectBase(shape={"name": "circle", "radius": 0.5}, state=[2, 3, 0])
        pieces = object_pieces(circle)
        assert len(pieces) == 1
        kind, center, radius = pieces[0]
        assert kind == "circle"
        assert np.allclose(center, [2, 3])
        assert radius == pytest.approx(0.5)

        compound = ObjectBase(
            shape={
                "name": "compound",
                "parts": [
                    {"name": "rectangle", "length": 0.6, "width": 0.6},
                    {"name": "circle", "radius": 0.3, "pose": [0.4, 0.3, 0]},
                ],
            }
        )
        assert len(object_pieces(compound)) == 2

        wall = ObjectBase(
            shape={"name": "linestring", "vertices": [[0, 0], [1, 0], [1, 1]]},
            state=[0, 0, 0],
        )
        assert len(object_pieces(wall)) == 2

    def test_object_mtv_uses_deepest_piece_and_cache(self):
        a = ObjectBase(shape={"name": "circle", "radius": 0.5}, state=[0.9, 0, 0])
        b = ObjectBase(
            shape={"name": "rectangle", "length": 1, "width": 1}, state=[0, 0, 0]
        )
        cache = {}
        normal, depth = object_mtv(a, b, cache)
        assert np.allclose(normal, [1, 0])
        assert depth == pytest.approx(0.1)
        assert id(a) in cache
        assert id(b) in cache
        assert object_mtv(a, b, cache)[1] == pytest.approx(depth)


class TestMass:
    """The ``mass`` parameter and what it makes of an object."""

    def test_defaults(self):
        robot = ObjectBase(kinematics={"name": "diff"}, role="robot")
        assert robot.mass == 1.0
        assert robot.inv_mass == 1.0
        assert robot.info.mass == 1.0

        rock = ObjectBase()
        assert rock.mass == float("inf")
        assert rock.inv_mass == 0.0
        assert rock.static

    def test_finite_mass_without_kinematics_is_a_dynamic_body(self):
        box = ObjectBase(mass=2)
        assert not box.static
        assert box.kinematics is None
        assert box.inv_mass == pytest.approx(0.5)
        # stays put on its own and reports no velocity
        state = box.state.copy()
        box.step()
        assert np.allclose(box.state, state)
        assert np.allclose(box.velocity_xy, 0)

    def test_static_flag_makes_any_mass_immovable(self):
        frozen = ObjectBase(kinematics={"name": "diff"}, static=True, mass=2)
        assert frozen.inv_mass == 0.0
        assert frozen.mass == 2.0
        rock = ObjectBase(static=True, mass=5)
        assert rock.static
        assert rock.inv_mass == 0.0

    @pytest.mark.parametrize("mass", [0, -1, "abc", float("nan"), [1, 2]])
    def test_invalid_mass_raises(self, mass):
        with pytest.raises(ValueError, match="mass"):
            ObjectBase(kinematics={"name": "diff"}, mass=mass)

    def test_mass_accepts_inf_and_numeric_strings(self):
        assert ObjectBase(kinematics={"name": "diff"}, mass="inf").inv_mass == 0.0
        assert ObjectBase(kinematics={"name": "diff"}, mass="2.5").mass == 2.5

    def test_factory_expands_mass_per_object(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 10, width: 10, collision_mode: 'contact'}\n"
                "obstacle:\n"
                "  - number: 3\n"
                "    distribution: {name: 'manual'}\n"
                "    shape: {name: 'circle', radius: 0.2}\n"
                "    state: [[1, 1, 0], [3, 3, 0], [5, 5, 0]]\n"
                "    mass: [0.5, 2]\n"
                "  - number: 2\n"
                "    distribution: {name: 'manual'}\n"
                "    shape: {name: 'circle', radius: 0.2}\n"
                "    state: [[7, 7, 0], [8, 8, 0]]\n"
                "    mass: 3\n"
                "  - shape: {name: 'circle', radius: 0.2}\n"
                "    state: [9, 9, 0]\n",
            )
        )
        masses = [obj.mass for obj in env.obstacle_list]
        assert masses == [0.5, 2, 2, 3, 3, float("inf")]
        assert [obj.static for obj in env.obstacle_list] == [False] * 5 + [True]

    def test_pushable_obstacles_get_the_movable_color(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 10, width: 10}\n"
                "robot:\n"
                "  - kinematics: {name: 'diff'}\n"
                "    state: [1, 1, 0]\n"
                "    mass: 2\n"
                "obstacle:\n"
                "  - {shape: {name: 'circle', radius: 0.2}, state: [2, 2, 0], mass: 2}\n"
                "  - {shape: {name: 'circle', radius: 0.2}, state: [3, 3, 0], mass: .inf}\n"
                "  - {shape: {name: 'circle', radius: 0.2}, state: [4, 4, 0], mass: 2, color: 'red'}\n"
                "  - {shape: {name: 'circle', radius: 0.2}, state: [5, 5, 0], kinematics: {name: 'diff'}, mass: 2}\n"
                "  - {shape: {name: 'circle', radius: 0.2}, state: [6, 6, 0], kinematics: {name: 'diff'}}\n"
                "  - {shape: {name: 'circle', radius: 0.2}, state: [7, 7, 0], kinematics: {name: 'diff'}, mass: 2, static: true}\n"
                "  - {shape: {name: 'circle', radius: 0.2}, state: [8, 8, 0]}\n",
            )
        )
        assert env.robot.color == DifferentialKinematics.color
        assert [obj.color for obj in env.obstacle_list] == [
            DYNAMIC_BODY_COLOR,
            "k",
            "red",
            DYNAMIC_BODY_COLOR,
            "k",
            "k",
            "k",
        ]

    @pytest.mark.parametrize(
        ("kinematics", "theta", "expected"),
        [
            (None, 0.0, [1.0, 2.0]),
            ({"name": "diff"}, np.pi / 2, [2.0, 0.0]),
            ({"name": "omni"}, np.pi / 2, [2.0, -1.0]),
            ({"name": "omni_angular"}, np.pi / 2, [2.0, -1.0, 0.0]),
            ({"name": "acker"}, np.pi / 2, [2.0, 0.0]),
        ],
    )
    def test_contact_velocity_in_command_frame(self, kinematics, theta, expected):
        obj = ObjectBase(
            kinematics=kinematics,
            mass=1,
            state=[0, 0, theta, 0]
            if kinematics == {"name": "acker"}
            else [0, 0, theta],
            state_dim=4 if kinematics == {"name": "acker"} else None,
        )
        out = obj._contact_velocity(np.array([[1.0], [2.0]]))
        assert np.allclose(out.ravel(), expected)

    def test_apply_contact_displacement_updates_everything(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        robot, box = env.robot, env.obstacle_list[0]
        env.step([1.0, 0.0])
        box.apply_contact_displacement([0.2, 0.0])
        assert box.state[0, 0] == pytest.approx(3.2)
        assert box.geometry.centroid.x == pytest.approx(3.2)
        assert np.allclose(box.velocity_xy.ravel(), [2.0, 0.0])
        assert np.allclose(box.trajectory[-1], box.state)
        robot.apply_contact_displacement([-0.1, 0.0])
        assert robot.velocity[0, 0] == pytest.approx(0.0)
        assert np.allclose(robot.trajectory[-1], robot.state)


class TestContactMode:
    """Environment-level behavior of ``collision_mode: contact``."""

    def _drive(self, env, steps, action=(1.0, 0.0)):
        for _ in range(steps):
            env.step(list(action))

    def test_equal_masses_share_the_motion(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        robot, box = env.robot, env.obstacle_list[0]
        self._drive(env, 13)  # robot front reaches the box at x = 2.6
        assert box.state[0, 0] == pytest.approx(3.0, abs=1e-5)
        self._drive(env, 40)
        assert box.state[0, 0] == pytest.approx(3.0 + 40 * 0.05, abs=1e-3)
        assert robot.state[0, 0] == pytest.approx(2.3 + 40 * 0.05, abs=1e-3)
        assert robot.velocity[0, 0] == pytest.approx(0.5, abs=1e-3)
        assert np.allclose(box.velocity_xy.ravel(), [0.5, 0.0], atol=1e-3)
        assert robot.contact
        assert box.contact
        assert box in robot.contact_obj
        assert robot in box.contact_obj
        assert not robot.collision
        assert not box.collision
        assert not env.done()
        assert env.status == "Running"
        assert not robot.geometry.intersects(box.geometry)
        assert robot.geometry.distance(box.geometry) < 10 * CONTACT_SLOP

    @pytest.mark.parametrize(
        ("box_mass", "share"), [(0.1, 1 / 1.1), (9.0, 0.1), ("inf", 0.0)]
    )
    def test_lighter_boxes_move_further(self, env_factory, tmp_path, box_mass, share):
        env = env_factory(_yaml(tmp_path, _push_world(box_mass=box_mass, wall=False)))
        robot, box = env.robot, env.obstacle_list[0]
        self._drive(env, 13 + 20)
        assert box.state[0, 0] == pytest.approx(3.0 + 20 * 0.1 * share, abs=1e-3)
        assert robot.state[0, 0] == pytest.approx(2.3 + 20 * 0.1 * share, abs=1e-3)

    def test_wall_stops_the_chain(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world()))
        robot, box, wall = env.robot, env.obstacle_list[0], env.obstacle_list[1]
        assert wall.static
        assert wall.inv_mass == 0.0
        self._drive(env, 120)
        first = (robot.state[0, 0], box.state[0, 0])
        self._drive(env, 5)
        assert robot.state[0, 0] == pytest.approx(first[0], abs=1e-4)
        assert box.state[0, 0] == pytest.approx(first[1], abs=1e-4)
        # box (0.8 wide) rests against the wall face at x = 7.8, robot behind it
        assert box.state[0, 0] == pytest.approx(7.4, abs=1e-3)
        assert robot.state[0, 0] == pytest.approx(6.7, abs=1e-3)
        assert not box.geometry.intersects(wall.geometry)
        assert not robot.geometry.intersects(box.geometry)
        assert robot.velocity[0, 0] == pytest.approx(0.0, abs=1e-3)
        assert not robot.collision
        assert not env.done()
        assert wall.contact
        assert not wall.trajectory

    def test_oblique_push_slides_the_box_along_the_wall(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 12, width: 12, step_time: 0.1, collision_mode: 'contact'}\n"
                "robot:\n"
                "  - kinematics: {name: 'omni'}\n"
                "    shape: {name: 'circle', radius: 0.3}\n"
                "    state: [2, 2, 0]\n"
                "obstacle:\n"
                "  - shape: {name: 'rectangle', length: 0.8, width: 0.8}\n"
                "    state: [3.2, 3.2, 0]\n"
                "    mass: 1.0\n"
                "  - shape: {name: 'linestring', vertices: [[0, 4.5], [12, 4.5]]}\n"
                "    state: [0, 0, 0]\n",
            )
        )
        robot, box, wall = env.robot, env.obstacle_list[0], env.obstacle_list[1]
        self._drive(env, 80, action=(0.7, 0.7))
        assert box.state[1, 0] == pytest.approx(4.1, abs=1e-3)  # pressed on the wall
        assert box.state[0, 0] > 5.0  # and still travelling along it
        assert robot.state[0, 0] > 4.0
        assert robot.velocity[0, 0] > 0.3
        assert not box.geometry.intersects(wall.geometry)
        assert not robot.geometry.intersects(box.geometry)
        assert not any(obj.collision for obj in env.objects)

    def test_light_box_between_robot_and_wall_settles(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world(box_mass=0.01)))
        robot, box, wall = env.robot, env.obstacle_list[0], env.obstacle_list[1]
        self._drive(env, 90)
        assert box.state[0, 0] == pytest.approx(7.4, abs=1e-3)
        assert robot.state[0, 0] == pytest.approx(6.7, abs=1e-3)
        assert not robot.collision
        assert not box.collision
        assert not robot.geometry.intersects(box.geometry)
        assert not box.geometry.intersects(wall.geometry)

    def test_box_stops_when_the_push_stops(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        box = env.obstacle_list[0]
        self._drive(env, 20)
        assert box.velocity_xy[0, 0] == pytest.approx(0.5, abs=1e-3)
        x = box.state[0, 0]
        self._drive(env, 3, action=(0.0, 0.0))
        assert box.state[0, 0] == pytest.approx(x)
        assert np.allclose(box.velocity_xy, 0)
        assert not box.contact

    def test_unobstructed_objects_take_no_part(self, env_factory, tmp_path):
        text = _push_world(wall=False).replace(
            "    mass: 1.0\nobstacle", "    mass: 1.0\n    unobstructed: true\nobstacle"
        )
        env = env_factory(_yaml(tmp_path, text))
        robot, box = env.robot, env.obstacle_list[0]
        self._drive(env, 20)
        assert box.state[0, 0] == pytest.approx(3.0)
        assert robot.state[0, 0] == pytest.approx(3.0)
        assert not robot.contact
        assert not box.contact

    def test_reset_clears_contact(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        robot, box = env.robot, env.obstacle_list[0]
        self._drive(env, 20)
        assert robot.contact
        env.reset()
        assert not robot.contact
        assert not box.contact
        assert robot.contact_obj == []
        assert box.contact_obj == []
        assert box.state[0, 0] == pytest.approx(3.0)

    def test_omni_robot_slides_along_a_linestring_wall(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 10, width: 10, step_time: 0.1, collision_mode: 'contact'}\n"
                "robot:\n"
                "  - kinematics: {name: 'omni'}\n"
                "    shape: {name: 'circle', radius: 0.3}\n"
                "    state: [2, 2, 0]\n"
                "obstacle:\n"
                "  - shape: {name: 'linestring', vertices: [[0, 4.2], [8, 4.2]]}\n"
                "    state: [0, 0, 0]\n",
            )
        )
        robot, wall = env.robot, env.obstacle_list[0]
        self._drive(env, 60, action=(0.7, 0.7))
        assert robot.state[1, 0] == pytest.approx(3.9, abs=1e-3)
        assert robot.state[0, 0] == pytest.approx(2 + 60 * 0.07, abs=1e-6)
        assert robot.velocity[0, 0] == pytest.approx(0.7, abs=1e-3)
        assert robot.velocity[1, 0] == pytest.approx(0.0, abs=1e-3)
        assert robot.contact
        assert not robot.collision
        assert not robot.geometry.intersects(wall.geometry)

    def test_grid_map_blocks_the_robot(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(
                tmp_path,
                "world:\n"
                "  height: 50\n  width: 50\n  step_time: 0.1\n"
                "  collision_mode: 'contact'\n"
                f"  obstacle_map: '{CAVE_PNG}'\n"
                "  mdownsample: 2\n"
                "robot:\n"
                "  - kinematics: {name: 'diff'}\n"
                "    shape: {name: 'circle', radius: 1.0}\n"
                "    state: [5, 5, 0]\n"
                "    vel_max: [4, 1]\n",
            )
        )
        robot = env.robot
        grid = next(obj for obj in env.objects if obj.shape == "map")
        assert grid.inv_mass == 0.0
        assert object_pieces(grid, robot) == []  # nothing near the start
        self._drive(env, 200, action=(3.0, 0.0))
        assert robot.contact
        assert grid in robot.contact_obj
        assert robot.state[0, 0] < 40
        assert robot.velocity[0, 0] == pytest.approx(0.0, abs=1e-3)
        assert not grid.is_collision(robot.geometry)
        assert not robot.collision
        robot.set_state([robot.state[0, 0] + 0.1, 5, 0])  # into the wall
        assert object_pieces(grid, robot)
        assert object_mtv(robot, grid)[1] == pytest.approx(0.1, abs=1e-3)

    def test_compound_and_non_convex_bodies_are_pushed(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 12, width: 12, step_time: 0.1, collision_mode: 'contact'}\n"
                "robot:\n"
                "  - kinematics: {name: 'diff'}\n"
                "    shape: {name: 'rectangle', length: 0.6, width: 0.4}\n"
                "    state: [1, 8, 0]\n"
                "    mass: 2.0\n"
                "obstacle:\n"
                "  - shape: {name: 'compound', parts: [{name: 'rectangle', length: 0.6, width: 0.6}, {name: 'circle', radius: 0.3, pose: [0.4, 0.3, 0]}]}\n"
                "    state: [3, 8, 0]\n"
                "    mass: 0.5\n"
                "  - shape: {name: 'polygon', vertices: [[0, 0], [1, 0], [1, 1], [0.5, 0.3], [0, 1]]}\n"
                "    state: [5, 7.6, 0]\n"
                "    mass: 3.0\n",
            )
        )
        robot, compound, arrow = env.robot, env.obstacle_list[0], env.obstacle_list[1]
        self._drive(env, 60)
        assert compound.state[0, 0] > 3.5
        assert arrow.state[0, 0] > 5.1
        assert not robot.geometry.intersects(compound.geometry)
        assert not compound.geometry.intersects(arrow.geometry)
        assert not any(obj.collision for obj in env.objects)

    def test_stop_mode_ignores_mass(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world(mode="stop")))
        robot, box = env.robot, env.obstacle_list[0]
        self._drive(env, 40)
        assert robot.stop_flag
        assert env.done()
        assert robot.state[0, 0] == pytest.approx(2.3)
        assert box.state[0, 0] == pytest.approx(3.0)
        assert not robot.contact

    def test_external_step_mode_leaves_states_alone(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(tmp_path, _push_world(wall=False)), step_mode="external"
        )
        robot, box = env.robot, env.obstacle_list[0]
        robot.set_state([2.5, 5, 0])
        env.step()
        assert robot.state[0, 0] == pytest.approx(2.5)
        assert box.state[0, 0] == pytest.approx(3.0)
        assert robot.collision
        assert not robot.contact

    @pytest.mark.parametrize("projection", ["2d", "3d"])
    def test_projection_and_messages(self, env_factory, tmp_path, projection):
        env = env_factory(_yaml(tmp_path, _push_world()), projection=projection)
        self._drive(env, 20)
        env.render(0.01)
        msg = env.get_msg()
        box = msg.obstacles[0]
        assert box.static is False
        assert box.odom.twist.twist.linear.x == pytest.approx(0.5, abs=1e-3)
        assert env.robot.contact

    def test_resolve_contacts_without_movable_bodies(self):
        a = ObjectBase(shape={"name": "circle", "radius": 0.5}, state=[0, 0, 0])
        b = ObjectBase(shape={"name": "circle", "radius": 0.5}, state=[0.5, 0, 0])
        assert resolve_contacts([(a, b)]) == []
        assert not a.contact

    def test_resolve_contacts_records_normal_and_depth(self):
        a = ObjectBase(
            kinematics={"name": "omni"},
            shape={"name": "circle", "radius": 0.5},
            state=[0, 0, 0],
        )
        b = ObjectBase(shape={"name": "circle", "radius": 0.5}, state=[0.8, 0, 0])
        contacts = resolve_contacts([(a, b)])
        assert len(contacts) == 1
        assert contacts[0].a is a
        assert contacts[0].b is b
        assert np.allclose(contacts[0].normal, [-1, 0])
        assert contacts[0].depth == pytest.approx(0.2 + CONTACT_SLOP)
        assert a.state[0, 0] == pytest.approx(-0.2 - CONTACT_SLOP)
        assert b.state[0, 0] == pytest.approx(0.8)
        assert a.contact
        assert b.contact
