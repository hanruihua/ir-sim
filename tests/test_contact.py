"""
Tests for the ``contact`` collision mode: per-object ``mass``, the SAT contact
solver, and mass-shared pushing in the environment.
"""

from pathlib import Path

import numpy as np
import pytest
import shapely
from shapely.geometry import LineString, Polygon

from irsim.config import palette_param
from irsim.lib.algorithm.contact import (
    CONTACT_SLOP,
    Contact,
    _axes,
    _transform,
    contact_step,
    geometry_pieces,
    object_mtv,
    object_pieces,
    piece_mtv,
)
from irsim.lib.handler.kinematics_handler import (
    DifferentialKinematics,
    PassiveKinematics,
)
from irsim.world.object_base import ObjectBase
from irsim.world.sensors import Contact2D, SensorFactory

CAVE_PNG = Path(__file__).parent / "cave.png"
# In contact mode a robot's wheels change its speed by at most friction * g per
# second (the default grip), so a drive launches and brakes over a few steps.
GRIP = 0.5 * 9.81


def _ramp(command, steps, dt=0.1, grip=GRIP):
    """Distance a drive covers along each axis from rest in ``steps``, with its
    speed change per step capped at ``grip * dt``."""
    command = np.asarray(command, dtype=float)
    velocity = np.zeros_like(command)
    distance = np.zeros_like(command)
    for _ in range(steps):
        change = command - velocity
        wanted = float(np.linalg.norm(change))
        if wanted > grip * dt:
            change = change * (grip * dt / wanted)
        velocity = velocity + change
        distance = distance + velocity * dt
    return distance


# how far a robot commanded 1 m/s from rest falls behind an instant start
LAUNCH_SHORTFALL = 3 * 0.1 - float(_ramp([1.0], 3)[0])


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
        normal, depth, _ = piece_mtv(_circle(0, 0, 1), _circle(1.5, 0, 1))
        assert np.allclose(normal, [-1, 0])
        assert depth == pytest.approx(0.5)

    def test_circle_polygon(self):
        normal, depth, _ = piece_mtv(_circle(1.2, 0.5, 0.3), _square(0, 0, 1))
        assert np.allclose(normal, [1, 0])
        assert depth == pytest.approx(0.1)

    def test_polygon_polygon(self):
        normal, depth, _ = piece_mtv(_square(0.9, 0.2, 1), _square(0, 0, 1))
        assert np.allclose(normal, [1, 0])
        assert depth == pytest.approx(0.1)

    def test_circle_straddling_segment_pushes_out_the_short_way(self):
        wall = ("polygon", np.array([[-5.0, 0.5], [5.0, 0.5]]))
        normal, depth, _ = piece_mtv(_circle(0, 0.25, 0.3), wall)
        assert np.allclose(normal, [0, -1])
        assert depth == pytest.approx(0.05)

    def test_containment_exits_through_nearest_side(self):
        normal, depth, _ = piece_mtv(_square(0.7, 0.4, 0.2), _square(0, 0, 1))
        assert np.allclose(normal, [1, 0])
        assert depth == pytest.approx(0.3)

    def test_contact_point_lies_midway_through_the_overlap(self):
        # a circle acts at its rim, midway through the 0.1 overlap
        assert np.allclose(
            piece_mtv(_circle(1.2, 0.5, 0.3), _square(0, 0, 1))[2], [0.95, 0.5]
        )
        # two parallel faces act at the middle of their overlapping stretch
        assert np.allclose(
            piece_mtv(_square(0.9, 0.2, 1), _square(0, 0, 1))[2], [0.95, 0.6]
        )
        # two circles act on the line through both centers
        assert np.allclose(
            piece_mtv(_circle(0, 0, 1), _circle(1.5, 0, 1))[2], [0.75, 0]
        )

    def test_separated_is_none(self):
        assert piece_mtv(_circle(0, 0, 1), _circle(3, 0, 1)) is None
        assert piece_mtv(_square(2, 2, 1), _square(0, 0, 1)) is None

    def test_touching_is_zero_depth_contact(self):
        _, depth, _ = piece_mtv(_square(1, 0, 1), _square(0, 0, 1))
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
        normal, depth, _ = piece_mtv(_circle(0.1, 0, 0.5), pieces[0])
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
        normal, depth, _ = object_mtv(a, b, cache)
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
        assert robot.pushable
        assert robot.info.mass == 1.0

        rock = ObjectBase()
        assert rock.mass == float("inf")
        assert rock.inv_mass == 0.0
        assert rock.static
        assert not rock.pushable
        assert rock.info.static
        assert rock.color == palette_param.obstacle

    def test_finite_mass_without_kinematics_is_pushable(self):
        box = ObjectBase(mass=2)
        assert not box.static
        assert box.pushable
        assert not box.info.static
        assert box.kinematics is None
        assert isinstance(box.kf, PassiveKinematics)
        assert box.inv_mass == pytest.approx(0.5)
        # the object picks the pushable color itself, unless one is given
        assert box.color == palette_param.pushable
        assert box.info.color == palette_param.pushable
        assert ObjectBase(mass=2, color="red").color == "red"
        assert ObjectBase(mass=2, role="robot").color == palette_param.pushable
        # stays put on its own and reports no velocity
        state = box.state.copy()
        box.step()
        assert np.allclose(box.state, state)
        assert np.allclose(box.velocity_xy, 0)

    def test_static_flag_makes_any_mass_immovable(self):
        frozen = ObjectBase(kinematics={"name": "diff"}, static=True, mass=2)
        assert frozen.inv_mass == 0.0
        assert not frozen.pushable
        assert frozen.mass == 2.0
        rock = ObjectBase(static=True, mass=5)
        assert rock.static
        assert rock.inv_mass == 0.0

    def test_friction_defaults_and_force(self):
        robot = ObjectBase(kinematics={"name": "diff"}, role="robot")
        assert robot.friction == 0.5
        assert robot.info.friction == 0.5
        assert not robot.passive
        assert robot.friction_force == pytest.approx(0.5 * 1.0 * 9.81)
        box = ObjectBase(mass=2, friction=0.1)
        assert box.passive
        assert box.friction_force == pytest.approx(0.1 * 2 * 9.81)
        assert ObjectBase().friction_force == float("inf")

    def test_inertia_defaults_and_override(self):
        disc = ObjectBase(shape={"name": "circle", "radius": 0.5}, mass=2)
        assert disc.inertia == pytest.approx(0.5 * 2 * 0.5**2)
        assert disc.inv_inertia == pytest.approx(1 / disc.inertia)
        assert disc.gyration == pytest.approx(0.5 / np.sqrt(2))
        slab = ObjectBase(
            shape={"name": "rectangle", "length": 0.8, "width": 0.4}, mass=3
        )
        assert slab.inertia == pytest.approx(3 * (0.8**2 + 0.4**2) / 12)
        # a polygon turns about its centroid wherever its origin is: this unit
        # square has its origin at a corner and still gets m (1 + 1) / 12
        corner = ObjectBase(
            shape={"name": "polygon", "vertices": [[0, 0], [1, 0], [1, 1], [0, 1]]},
            mass=1,
        )
        assert corner.inertia == pytest.approx(1 / 6)
        assert corner.info.inertia == pytest.approx(1 / 6)
        assert (
            ObjectBase(
                shape={"name": "circle", "radius": 0.5}, mass=2, inertia=5
            ).inertia
            == 5
        )
        # a robot turns only when a contact makes it yield; walls never move
        robot = ObjectBase(
            kinematics={"name": "diff"}, shape={"name": "circle", "radius": 0.3}
        )
        assert robot.inertia == pytest.approx(0.5 * 1 * 0.3**2)
        assert robot.inv_inertia == pytest.approx(1 / robot.inertia)
        assert robot.restitution == 0.0
        assert robot.info.restitution == 0.0
        assert ObjectBase(mass=1, restitution=0.3).restitution == 0.3
        wall = ObjectBase(shape={"name": "circle", "radius": 0.3})
        assert wall.inertia == float("inf")
        assert wall.inv_inertia == 0.0
        assert wall.gyration == 0.0

    @pytest.mark.parametrize("restitution", [-0.1, 1.5, "abc", float("nan")])
    def test_invalid_restitution_raises(self, restitution):
        with pytest.raises(ValueError, match="restitution"):
            ObjectBase(mass=1, restitution=restitution)

    @pytest.mark.parametrize("inertia", [0, -1, "abc", float("nan")])
    def test_invalid_inertia_raises(self, inertia):
        with pytest.raises(ValueError, match="inertia"):
            ObjectBase(mass=1, inertia=inertia)

    @pytest.mark.parametrize("friction", [-0.1, "abc", float("inf"), float("nan")])
    def test_invalid_friction_raises(self, friction):
        with pytest.raises(ValueError, match="friction"):
            ObjectBase(mass=1, friction=friction)

    @pytest.mark.parametrize("mass", [0, -1, "abc", float("nan"), [1, 2]])
    def test_invalid_mass_raises(self, mass):
        with pytest.raises(ValueError, match="mass"):
            ObjectBase(kinematics={"name": "diff"}, mass=mass)

    def test_mass_accepts_inf_and_numeric_strings(self):
        heavy = ObjectBase(kinematics={"name": "diff"}, mass="inf")
        assert heavy.inv_mass == 0.0
        assert not heavy.pushable
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
                "    friction: [0.2, 0.3, 0.4]\n"
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
        assert [obj.friction for obj in env.obstacle_list] == [
            0.2,
            0.3,
            0.4,
            0.5,
            0.5,
            0.5,
        ]
        assert [obj.static for obj in env.obstacle_list] == [False] * 5 + [True]
        # objects without kinematics carry no handler; the object itself
        # decides from its mass whether it is static
        assert all(obj.kinematics is None for obj in env.obstacle_list)

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
        assert env.robot.color == DifferentialKinematics.default_color("robot")
        # orange marks a body without kinematics that only moves when pushed;
        # an obstacle with kinematics keeps its handler color, mass or not
        assert [obj.color for obj in env.obstacle_list] == [
            palette_param.pushable,
            "k",
            "red",
            "k",
            "k",
            "k",
            "k",
        ]

    @pytest.mark.parametrize(
        ("kinematics", "theta", "expected"),
        [
            (None, 0.0, [1.0, 2.0, 0.0]),
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

    def test_bodies_turn_about_their_center_of_mass(self, env_factory, tmp_path):
        """A square whose origin is a corner still turns about its centroid:
        pushed through the centroid it does not turn at all, and pushed
        off-center its centroid only translates while the corner swings."""

        def scene(robot_y):
            return env_factory(
                _yaml(
                    tmp_path,
                    "world: {height: 12, width: 12, step_time: 0.1, collision_mode: 'contact'}\n"
                    "robot:\n"
                    "  - kinematics: {name: 'omni'}\n"
                    "    shape: {name: 'circle', radius: 0.2}\n"
                    f"    state: [2.5, {robot_y}, 0]\n"
                    "obstacle:\n"
                    "  - shape: {name: 'polygon', vertices: [[0, 0], [1, 0], [1, 1], [0, 1]]}\n"
                    "    state: [3, 5, 0]\n"
                    "    mass: 1.0\n",
                )
            )

        env = scene(5.5)  # aimed at the centroid (3.5, 5.5)
        square = env.obstacle_list[0]
        for _ in range(15):
            env.step([1.0, 0.0])
        assert square.state[2, 0] == pytest.approx(0.0, abs=1e-9)
        # one contact near the top edge: the centroid moves only along the
        # push normal (+x) while the corner origin swings around it
        square = ObjectBase(
            shape={"name": "polygon", "vertices": [[0, 0], [1, 0], [1, 1], [0, 1]]},
            state=[3, 5, 0],
            mass=1.0,
        )
        robot = ObjectBase(
            kinematics={"name": "omni"},
            shape={"name": "circle", "radius": 0.2},
            state=[2.85, 5.9, 0],  # overlaps the left face by 5 cm, near the top
        )
        before = np.array(square.geometry.centroid.coords[0])
        contact_step([(robot, square)])
        after = np.array(square.geometry.centroid.coords[0])
        assert square.state[2, 0] < -0.01  # turned clockwise
        assert after[0] > before[0]
        # a second sweep pushes on the slightly turned face, so allow a hair;
        # turning about the corner would have moved the centroid by ~1.4 cm
        assert after[1] == pytest.approx(before[1], abs=1e-3)
        assert square.state[1, 0] != pytest.approx(5.0, abs=1e-4)  # origin swung

    def test_apply_contact_displacement_updates_everything(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        robot, box = env.robot, env.obstacle_list[0]
        env.step([1.0, 0.0])
        box.apply_contact_displacement([0.2, 0.0])
        assert box.state[0, 0] == pytest.approx(3.2)
        assert box.geometry.centroid.x == pytest.approx(3.2)
        assert np.allclose(box.velocity_xy.ravel(), [2.0, 0.0])
        assert np.allclose(box.trajectory[-1], box.state)
        box.apply_contact_displacement([0.0, 0.0], 0.1)
        assert box.state[2, 0] == pytest.approx(0.1)
        assert box.velocity[2, 0] == pytest.approx(1.0)  # 0.1 rad in 0.1 s
        assert box.geometry.bounds[2] - box.geometry.bounds[0] > 0.8  # turned box
        robot.apply_contact_displacement([-0.1, 0.0])
        # one step from rest the drive reached grip * dt, the push takes 1 m/s off it
        assert robot.velocity[0, 0] == pytest.approx(GRIP * 0.1 - 1.0)
        assert np.allclose(robot.trajectory[-1], robot.state)


class TestContactMode:
    """Environment-level behavior of ``collision_mode: contact``."""

    def _drive(self, env, steps, action=(1.0, 0.0)):
        for _ in range(steps):
            env.step(list(action))

    def test_pushed_box_moves_with_the_robot(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        robot, box = env.robot, env.obstacle_list[0]
        self._drive(env, 13)  # robot front reaches the box at x = 2.6
        assert box.state[0, 0] == pytest.approx(3.0, abs=1e-5)
        self._drive(env, 40)
        # equal materials and masses: the robot's traction matches the box's
        # ground friction, so it pushes the box at its own speed
        assert box.state[0, 0] == pytest.approx(
            3.0 + 40 * 0.1 - LAUNCH_SHORTFALL, abs=1e-3
        )
        assert robot.state[0, 0] == pytest.approx(
            2.3 + 40 * 0.1 - LAUNCH_SHORTFALL, abs=1e-3
        )
        assert robot.velocity[0, 0] == pytest.approx(1.0, abs=1e-3)
        assert np.allclose(box.velocity_xy.ravel(), [1.0, 0.0], atol=1e-3)
        assert robot.contact.in_contact
        assert box.contact.in_contact
        assert box in robot.contact.partners
        assert robot in box.contact.partners
        assert not robot.collision
        assert not box.collision
        assert not env.done()
        assert env.status == "Running"
        assert not robot.geometry.intersects(box.geometry)
        assert robot.geometry.distance(box.geometry) < 10 * CONTACT_SLOP

    @pytest.mark.parametrize(
        ("box_mass", "pushed"),
        [(0.1, True), (1.0, True), (1.5, False), (9.0, False), ("inf", False)],
    )
    def test_traction_decides_what_the_robot_can_push(
        self, env_factory, tmp_path, box_mass, pushed
    ):
        """A robot pushes a body while friction * mass of the robot is at
        least the body's; a 1 kg robot moves boxes up to 1 kg and stalls
        against heavier ones."""
        env = env_factory(_yaml(tmp_path, _push_world(box_mass=box_mass, wall=False)))
        robot, box = env.robot, env.obstacle_list[0]
        self._drive(env, 13 + 20)
        travel = 20 * 0.1 - LAUNCH_SHORTFALL if pushed else 0.0
        assert box.state[0, 0] == pytest.approx(3.0 + travel, abs=1e-3)
        assert robot.state[0, 0] == pytest.approx(2.3 + travel, abs=1e-3)
        assert robot.velocity[0, 0] == pytest.approx(1.0 if pushed else 0.0, abs=1e-3)
        assert robot.contact.in_contact
        assert not robot.collision
        assert not env.done()

    @pytest.mark.parametrize(("robot_friction", "pushed"), [(0.5, False), (1.5, True)])
    def test_load_is_summed_along_the_chain(
        self, env_factory, tmp_path, robot_friction, pushed
    ):
        """A 1 kg robot that cannot push a 2 kg box cannot push it through a
        0.3 kg box either: the whole chain ahead of it counts as load."""
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 12, width: 12, step_time: 0.1, collision_mode: 'contact'}\n"
                "robot:\n"
                "  - kinematics: {name: 'diff'}\n"
                "    shape: {name: 'circle', radius: 0.3}\n"
                "    state: [1, 5, 0]\n"
                "    vel_max: [1.0, 1.0]\n"
                f"    friction: {robot_friction}\n"
                "obstacle:\n"
                "  - {shape: {name: 'rectangle', length: 0.4, width: 0.8}, state: [2.0, 5, 0], mass: 0.3}\n"
                "  - {shape: {name: 'rectangle', length: 0.8, width: 0.8}, state: [2.6, 5, 0], mass: 2.0}\n",
            )
        )
        robot, light, heavy = env.robot, env.obstacle_list[0], env.obstacle_list[1]
        assert robot.friction_force == pytest.approx(robot_friction * 9.81)
        self._drive(env, 40)
        if pushed:
            assert heavy.state[0, 0] > 3.5
            assert robot.velocity[0, 0] == pytest.approx(1.0, abs=1e-3)
        else:
            assert light.state[0, 0] == pytest.approx(2.0, abs=1e-3)
            assert heavy.state[0, 0] == pytest.approx(2.6, abs=1e-3)
            assert robot.velocity[0, 0] == pytest.approx(0.0, abs=1e-3)
            assert robot.contact.in_contact
            assert not robot.collision

    def test_contact_force_reports_the_load_being_overcome(self, env_factory, tmp_path):
        """The XPBD force estimate, impulse over the squared step, gives the
        ground friction a steadily pushed box overcomes."""
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        robot, box = env.robot, env.obstacle_list[0]
        assert np.allclose(box.contact.force, 0)
        self._drive(env, 30)
        assert np.allclose(
            box.contact.force.ravel(), [0.5 * 1.0 * 9.81, 0.0], atol=1e-6
        )
        assert np.allclose(
            robot.contact.force.ravel(), [-0.5 * 1.0 * 9.81, 0.0], atol=1e-6
        )
        self._drive(env, 3, action=(0.0, 0.0))
        assert np.allclose(box.contact.force, 0)  # cleared once nothing touches
        # a stalled robot reports its traction, the most its wheels can push
        # with, whatever the step size (the raw estimate would be m v / dt)
        for step_time in (0.1, 0.05):
            text = _push_world(box_mass=4.0, wall=False).replace(
                "  step_time: 0.1\n", f"  step_time: {step_time}\n"
            )
            env = env_factory(_yaml(tmp_path, text))
            robot = env.robot
            self._drive(env, int(3 / step_time))
            assert robot.velocity[0, 0] == pytest.approx(0.0, abs=1e-3)
            assert robot.contact.force[0, 0] == pytest.approx(-0.5 * 1.0 * 9.81)
            assert env.obstacle_list[0].contact.force[0, 0] == pytest.approx(
                0.5 * 1.0 * 9.81
            )

    @pytest.mark.parametrize("restitution", [0.0, 0.5, 1.0])
    def test_restitution_makes_passive_bodies_bounce(
        self, env_factory, tmp_path, restitution
    ):
        """A frictionless disc launched by a robot of the same material leaves
        it at (1 + e) times the robot's speed and comes back off a wall of
        that material at e times its own; a pair's restitution is the mean of
        the two materials, so an e = 0 robot would halve the bounce."""
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 12, width: 12, step_time: 0.1, collision_mode: 'contact'}\n"
                "robot:\n"
                "  - kinematics: {name: 'omni'}\n"
                "    shape: {name: 'circle', radius: 0.3}\n"
                "    state: [1, 5, 0]\n"
                f"    restitution: {restitution}\n"
                "obstacle:\n"
                "  - {shape: {name: 'circle', radius: 0.4}, state: [2, 5, 0], mass: 1.0, friction: 0, "
                f"restitution: {restitution}}}\n"
                "  - {shape: {name: 'rectangle', length: 0.4, width: 4}, state: [6, 5, 0], "
                f"restitution: {restitution}}}\n",
            )
        )
        disc = env.obstacle_list[0]
        self._drive(env, 10)
        launched = disc.velocity_xy[0, 0]
        assert launched == pytest.approx((1 + restitution) * 1.0, abs=1e-6)
        env.robot.set_state([1, 9, 0])  # out of the disc's way back
        self._drive(env, 60, action=(0.0, 0.0))
        assert disc.velocity_xy[0, 0] == pytest.approx(
            -restitution * launched, abs=1e-6
        )
        assert not disc.geometry.intersects(env.obstacle_list[1].geometry)

    def test_bounce_cannot_hammer_a_box_the_robot_cannot_push(
        self, env_factory, tmp_path
    ):
        """A bounce only goes to a body the contact could move: a stalled
        robot with restitution does not kick a too-heavy box forward."""
        text = _push_world(box_mass=2.0, wall=False).replace(
            "  collision_mode: 'contact'\n",
            "  collision_mode: 'contact'\n  restitution: 1.0\n",
        )
        env = env_factory(_yaml(tmp_path, text))
        robot, box = env.robot, env.obstacle_list[0]
        self._drive(env, 40)
        assert box.state[0, 0] == pytest.approx(3.0, abs=1e-6)
        assert np.allclose(box.velocity_xy, 0)
        assert robot.velocity[0, 0] == pytest.approx(0.0, abs=1e-3)

    def test_elastic_discs_exchange_their_velocities(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 12, width: 12, step_time: 0.1, collision_mode: 'contact'}\n"
                "robot:\n"
                "  - kinematics: {name: 'omni'}\n"
                "    shape: {name: 'circle', radius: 0.3}\n"
                "    state: [1, 5, 0]\n"
                "obstacle:\n"
                "  - {shape: {name: 'circle', radius: 0.4}, state: [2, 5, 0], mass: 1.0, friction: 0, restitution: 1}\n"
                "  - {shape: {name: 'circle', radius: 0.4}, state: [6, 5, 0], mass: 1.0, friction: 0, restitution: 1}\n",
            )
        )
        first, second = env.obstacle_list
        self._drive(env, 10)
        launched = first.velocity_xy[0, 0]
        self._drive(env, 60, action=(0.0, 0.0))
        assert first.velocity_xy[0, 0] == pytest.approx(0.0, abs=1e-6)
        assert second.velocity_xy[0, 0] == pytest.approx(launched, abs=1e-6)

    @pytest.mark.parametrize(
        ("box_mass", "offset", "deflected"),
        [(4.0, 0.25, True), (4.0, 0.0, False), (0.5, 0.25, False)],
    )
    def test_stalled_robot_is_deflected_by_an_off_center_contact(
        self, env_factory, tmp_path, box_mass, offset, deflected
    ):
        """A rectangle robot stopped by a box it cannot push turns away from
        an off-center contact as slipping wheels would; centered it stays
        straight, and while it pushes a box it can move it keeps its heading."""
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 12, width: 12, step_time: 0.1, collision_mode: 'contact'}\n"
                "robot:\n"
                "  - kinematics: {name: 'diff'}\n"
                "    shape: {name: 'rectangle', length: 0.6, width: 0.4}\n"
                f"    state: [1.5, {5 + offset}, 0]\n"
                "    vel_max: [1, 1]\n"
                "obstacle:\n"
                "  - shape: {name: 'rectangle', length: 0.8, width: 0.8}\n"
                "    state: [3, 5, 0]\n"
                f"    mass: {box_mass}\n",
            )
        )
        robot, box = env.robot, env.obstacle_list[0]
        swing, yaw_rates = 0.0, []
        for _ in range(30):
            env.step([1.0, 0.0])
            swing = max(swing, abs(float(robot.state[2, 0])))
            yaw_rates.append(abs(float(robot.velocity[1, 0])))  # diff yaw rate row
        if deflected:
            # jammed against the box, the robot is knocked a few degrees either way
            assert swing > 0.05
            assert max(yaw_rates) > 0.1
            assert box.state[0, 0] == pytest.approx(3.0, abs=1e-6)  # stalled
        else:
            assert swing == 0.0
            assert max(yaw_rates) == 0.0
        if box_mass < 1:
            assert abs(box.state[2, 0]) > 0.3  # the light box turned instead

    def test_more_grip_lets_the_robot_push_a_heavier_box(self, env_factory, tmp_path):
        text = _push_world(box_mass=3.0, wall=False).replace(
            "    mass: 1.0\nobstacle", "    mass: 1.0\n    friction: 2.0\nobstacle"
        )
        env = env_factory(_yaml(tmp_path, text))
        robot, box = env.robot, env.obstacle_list[0]
        assert robot.friction_force == pytest.approx(2.0 * 1.0 * 9.81)
        assert box.friction_force == pytest.approx(0.5 * 3.0 * 9.81)
        self._drive(env, 33)
        assert box.state[0, 0] == pytest.approx(5.0, abs=1e-3)
        assert robot.velocity[0, 0] == pytest.approx(1.0, abs=1e-3)

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
        assert wall.contact.in_contact
        assert not wall.trajectory

    def test_oblique_push_moves_then_pivots_the_box_on_the_wall(
        self, env_factory, tmp_path
    ):
        """A robot driving diagonally through a box's corner region pushes it
        without turning it (the contact acts on the line through both
        centers); once the box reaches the wall it pivots around the pressed
        corner while the robot slides on along the wall."""
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
        self._drive(env, 21, action=(0.7, 0.7))
        assert box.state[0, 0] == pytest.approx(box.state[1, 0], abs=1e-6)  # diagonal
        assert box.state[2, 0] == pytest.approx(0.0, abs=1e-9)  # centered push
        self._drive(env, 1, action=(0.7, 0.7))
        assert box.geometry.distance(wall.geometry) < 0.05  # reached the wall
        self._drive(env, 58, action=(0.7, 0.7))
        assert abs(box.state[2, 0]) > 0.5  # pivoted around the pressed corner
        assert np.allclose(robot.velocity.ravel(), [0.7, 0.0], atol=1e-3)  # slides on
        assert robot.state[0, 0] > box.state[0, 0]  # and left the box behind
        assert not box.geometry.intersects(wall.geometry)
        assert not robot.geometry.intersects(box.geometry)
        assert not any(obj.collision for obj in env.objects)

    @pytest.mark.parametrize(("offset", "sign"), [(0.3, -1), (-0.3, 1), (0.0, 0)])
    def test_off_center_push_turns_the_box(self, env_factory, tmp_path, offset, sign):
        """A push above the box's center turns it clockwise, below it
        counter-clockwise, through the center not at all; the robot keeps
        its heading and a released spin dies under friction."""
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 12, width: 12, step_time: 0.1, collision_mode: 'contact'}\n"
                "robot:\n"
                "  - kinematics: {name: 'omni'}\n"
                "    shape: {name: 'circle', radius: 0.3}\n"
                f"    state: [2, {5 + offset}, 0]\n"
                "obstacle:\n"
                "  - shape: {name: 'rectangle', length: 0.8, width: 0.8}\n"
                "    state: [3, 5, 0]\n"
                "    mass: 1.0\n",
            )
        )
        robot, box = env.robot, env.obstacle_list[0]
        assert box.inertia == pytest.approx(1.0 * (0.8**2 + 0.8**2) / 12)
        self._drive(env, 15)
        if sign == 0:
            assert box.state[2, 0] == pytest.approx(0.0, abs=1e-9)
            assert box.velocity[2, 0] == pytest.approx(0.0, abs=1e-9)
        else:
            assert sign * box.state[2, 0] > 0.3
            assert sign * box.velocity[2, 0] > 0.1
        assert robot.state[2, 0] == 0.0
        heading = box.state[2, 0]
        self._drive(env, 30, action=(0.0, 0.0))
        assert box.velocity[2, 0] == pytest.approx(0.0, abs=1e-9)  # spin braked
        assert abs(box.state[2, 0] - heading) < 0.2  # and it stayed put

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

    def test_released_box_coasts_to_a_stop(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        box = env.obstacle_list[0]
        self._drive(env, 20)
        assert box.velocity_xy[0, 0] == pytest.approx(1.0, abs=1e-3)
        x = box.state[0, 0]
        env.robot.set_state([1, 9, 0])  # away: a braking robot would still push
        self._drive(env, 4, action=(0.0, 0.0))
        # ground friction 0.5 stops it within about v^2 / (2 * 0.5 * g) = 5 cm
        assert 0.04 < box.state[0, 0] - x < 0.07
        assert np.allclose(box.velocity_xy, 0)
        assert not box.contact.in_contact

    def test_frictionless_box_keeps_its_velocity(self, env_factory, tmp_path):
        text = _push_world(wall=False) + "    friction: 0\n"
        env = env_factory(_yaml(tmp_path, text))
        box = env.obstacle_list[0]
        self._drive(env, 20)
        x = box.state[0, 0]
        self._drive(env, 5, action=(0.0, 0.0))
        assert box.state[0, 0] - x == pytest.approx(0.5, abs=1e-6)
        assert box.velocity_xy[0, 0] == pytest.approx(1.0)

    def test_coasting_boxes_exchange_momentum_by_mass(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 12, width: 12, step_time: 0.1, collision_mode: 'contact'}\n"
                "robot:\n"
                "  - kinematics: {name: 'omni'}\n"
                "    shape: {name: 'circle', radius: 0.3}\n"
                "    state: [1, 5, 0]\n"
                "obstacle:\n"
                "  - {shape: {name: 'circle', radius: 0.4}, state: [2, 5, 0], mass: 1.0, friction: 0}\n"
                "  - {shape: {name: 'circle', radius: 0.4}, state: [6, 5, 0], mass: 3.0, friction: 0}\n",
            )
        )
        light, heavy = env.obstacle_list
        self._drive(env, 10)  # launch the light box at 1 m/s
        assert light.velocity_xy[0, 0] == pytest.approx(1.0, abs=1e-6)
        self._drive(env, 60, action=(0.0, 0.0))
        # a perfectly inelastic collision: both continue at 1 / (1 + 3) m/s
        assert light.velocity_xy[0, 0] == pytest.approx(0.25, abs=1e-6)
        assert heavy.velocity_xy[0, 0] == pytest.approx(0.25, abs=1e-6)
        assert not light.geometry.intersects(heavy.geometry)

    def test_pressed_box_sticks_inside_the_friction_cone(self):
        """A passive body against a wall slides only when the push leaves the
        friction cone (mean coefficient 0.5: more than 26.6 degrees from the
        wall normal); a driven robot always slides."""

        def scene(angle_deg):
            wall = ObjectBase(
                shape={"name": "linestring", "vertices": [[0, 4.5], [8, 4.5]]},
                state=[0, 0, 0],
            )
            box = ObjectBase(
                shape={"name": "circle", "radius": 0.4}, state=[3, 4.1, 0], mass=1.0
            )
            theta = np.deg2rad(angle_deg)
            reach = 0.4 + 0.3 - 0.05  # overlap the box by 5 cm from below
            robot = ObjectBase(
                kinematics={"name": "omni"},
                shape={"name": "circle", "radius": 0.3},
                state=[3 - reach * np.sin(theta), 4.1 - reach * np.cos(theta), 0],
            )
            return wall, box, robot

        wall, box, robot = scene(15)
        start = robot.state.copy()
        contact_step([(box, wall), (robot, box)])
        assert box.state[0, 0] == pytest.approx(3.0, abs=1e-9)  # stuck
        assert np.linalg.norm(robot.state[:2] - start[:2]) == pytest.approx(
            0.05 + CONTACT_SLOP, abs=1e-6
        )  # the robot yields instead

        wall, box, robot = scene(45)
        start = robot.state.copy()
        contact_step([(box, wall), (robot, box)])
        assert box.state[0, 0] > 3.05  # slid along the wall
        assert box.state[1, 0] == pytest.approx(4.1, abs=1e-5)
        assert np.allclose(robot.state, start)  # the driver keeps its motion

    def test_contact_report_records_and_timing(self, env_factory, tmp_path):
        """What a contact sensor reports: the records with point, normal and
        force on the environment and on each object, and how long an object
        has been touching or free."""
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        robot, box = env.robot, env.obstacle_list[0]
        assert env.contacts == []
        assert robot.contact.reports == []
        self._drive(env, 13)
        assert not robot.contact.started
        env.step([1.0, 0.0])  # step 14: the robot reaches the box
        assert robot.contact.started
        assert box.contact.started
        self._drive(env, 6)
        assert not robot.contact.started
        (contact,) = env.contacts
        assert {contact.a, contact.b} == {robot, box}
        # each object sees the contact from its own side
        (report,) = robot.contact.reports
        assert report.other is box
        assert report.force == pytest.approx(0.5 * 1.0 * 9.81)
        assert report.point[1] == pytest.approx(5.0)  # on the box's left face
        # midway through this step's overlap, just inside the face
        assert box.state[0, 0] - 0.4 - 0.06 < report.point[0] < box.state[0, 0] - 0.4
        assert np.allclose(report.normal, [-1, 0])  # from the box toward the robot
        (seen_by_box,) = box.contact.reports
        assert seen_by_box.other is robot
        assert np.allclose(seen_by_box.normal, [1, 0])
        assert str(report).startswith(f"{box.name} (1 kg) at (")
        assert robot.contact.contact_time == pytest.approx(
            0.7
        )  # touching since step 14
        assert box.contact.contact_time == pytest.approx(0.7)
        assert robot.contact.air_time == 0.0
        robot.set_state([1, 9, 0])
        env.step([0.0, 0.0])
        assert robot.contact.ended
        assert not robot.contact.started
        self._drive(env, 2, action=(0.0, 0.0))
        assert not robot.contact.ended
        assert env.contacts == []
        assert robot.contact.reports == []
        assert robot.contact.air_time == pytest.approx(0.3)
        assert robot.contact.contact_time == 0.0
        env.reset()
        assert (robot.contact.contact_time, robot.contact.air_time, env.contacts) == (
            0.0,
            0.0,
            [],
        )
        assert not robot.contact.started
        assert not robot.contact.ended

    def test_unobstructed_objects_take_no_part(self, env_factory, tmp_path):
        text = _push_world(wall=False).replace(
            "    mass: 1.0\nobstacle", "    mass: 1.0\n    unobstructed: true\nobstacle"
        )
        env = env_factory(_yaml(tmp_path, text))
        robot, box = env.robot, env.obstacle_list[0]
        self._drive(env, 20)
        assert box.state[0, 0] == pytest.approx(3.0)
        assert robot.state[0, 0] == pytest.approx(3.0 - LAUNCH_SHORTFALL)
        assert not robot.contact.in_contact
        assert not box.contact.in_contact

    def test_reset_clears_contact(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        robot, box = env.robot, env.obstacle_list[0]
        self._drive(env, 20)
        assert robot.contact.in_contact
        env.reset()
        assert not robot.contact.in_contact
        assert not box.contact.in_contact
        assert robot.contact.partners == []
        assert box.contact.partners == []
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
        assert robot.state[0, 0] == pytest.approx(
            2 + _ramp([0.7, 0.7], 60)[0], abs=1e-6
        )
        assert robot.velocity[0, 0] == pytest.approx(0.7, abs=1e-3)
        assert robot.velocity[1, 0] == pytest.approx(0.0, abs=1e-3)
        assert robot.contact.in_contact
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
        assert len(object_pieces(grid)) > 100  # the whole boundary without a partner
        self._drive(env, 200, action=(3.0, 0.0))
        assert robot.contact.in_contact
        assert grid in robot.contact.partners
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
                "    mass: 4.0\n"  # enough traction for the 0.5 kg and 3 kg bodies it pushes in a row
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
        assert not robot.contact.in_contact
        assert env.contacts == []
        assert robot.contact.reports == []
        assert robot.contact.contact_time == 0.0
        assert robot.contact.air_time == 0.0

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
        assert not robot.contact.in_contact

    @pytest.mark.parametrize("projection", ["2d", "3d"])
    def test_projection_and_messages(self, env_factory, tmp_path, projection):
        env = env_factory(_yaml(tmp_path, _push_world()), projection=projection)
        self._drive(env, 20)
        env.render(0.01)
        msg = env.get_msg()
        box = msg.obstacles[0]
        assert box.static is False
        assert box.odom.twist.twist.linear.x == pytest.approx(1.0, abs=1e-3)
        assert env.robot.contact.in_contact
        # the contact report travels with the object state
        robot_state = msg.robots[0]
        (report,) = robot_state.contacts
        assert report.other == env.obstacle_list[0].name
        assert report.force == pytest.approx(0.5 * 9.81)
        assert report.normal.x == pytest.approx(-1.0)  # from the box toward the robot
        assert robot_state.contact_force.x == pytest.approx(-0.5 * 9.81)
        assert robot_state.contact_time > 0
        assert robot_state.air_time == 0
        assert msg.to_dict()["objects"][0]["contacts"][0]["other"] == report.other

    def test_grip_bounds_launch_and_braking(self, env_factory, tmp_path):
        """Wheels change a robot's speed by at most friction * g per second in
        contact mode: three steps to launch or stop at the default grip, and
        a robot without friction cannot move at all; stop mode is unaffected."""
        env = env_factory(_yaml(tmp_path, _push_world(wall=False)))
        robot = env.robot
        for expected in (GRIP * 0.1, 2 * GRIP * 0.1, 1.0):
            env.step([1.0, 0.0])
            assert robot.velocity[0, 0] == pytest.approx(expected)
        x = robot.state[0, 0]
        for expected in (1.0 - GRIP * 0.1, 1.0 - 2 * GRIP * 0.1, 0.0):
            env.step([0.0, 0.0])
            assert robot.velocity[0, 0] == pytest.approx(expected)
        assert 0.04 < robot.state[0, 0] - x < 0.11  # braking distance
        text = _push_world(wall=False).replace(
            "    mass: 1.0\nobstacle", "    mass: 1.0\n    friction: 0\nobstacle"
        )
        env = env_factory(_yaml(tmp_path, text))
        self._drive(env, 10)
        assert env.robot.state[0, 0] == pytest.approx(1.0)  # no grip, no motion
        assert env.robot.velocity[0, 0] == 0.0
        env = env_factory(_yaml(tmp_path, text.replace("'contact'", "'stop'")))
        self._drive(env, 10)
        assert env.robot.state[0, 0] == pytest.approx(2.0)  # stop mode ignores grip

    def test_drive_lag_in_contact_mode(self, env_factory, tmp_path):
        """With a world drive lag of 0.2 s a robot's velocity follows a step
        command as a first-order response: half the gap closes every 0.1 s."""
        text = _push_world(wall=False).replace(
            "  collision_mode: 'contact'\n",
            "  collision_mode: 'contact'\n  drive_tau: 0.2\n",
        )
        env = env_factory(_yaml(tmp_path, text))
        robot = env.robot
        assert robot.drive_tau == 0.2
        assert robot.kf.tau is None
        expected = 0.0
        for _ in range(5):
            env.step([1.0, 0.0])
            # the lag closes half the gap, but never more than the grip allows
            expected += min(GRIP * 0.1, 0.5 * (1.0 - expected))
            assert robot.velocity[0, 0] == pytest.approx(expected)
        assert robot.state[0, 0] < 1.5  # travelled less than five full steps
        text = _push_world(wall=False).replace(
            "{name: 'diff'}", "{name: 'diff', tau: 0.5}"
        )
        env = env_factory(_yaml(tmp_path, text))
        env.step([1.0, 0.0])
        assert env.robot.velocity[0, 0] == pytest.approx(0.2)

    def test_drive_lag_is_off_outside_contact_mode(self, env_factory, tmp_path):
        text = _push_world(wall=False, mode="stop").replace(
            "{name: 'diff'}", "{name: 'diff', tau: 0.5}"
        )
        env = env_factory(_yaml(tmp_path, text))
        env.step([1.0, 0.0])
        assert env.robot.velocity[0, 0] == pytest.approx(1.0)
        assert env.robot.state[0, 0] == pytest.approx(1.1)

    def test_stalled_robot_keeps_pushing_at_full_drive(self, env_factory, tmp_path):
        """The drive's velocity is separate from the body's: stalled against
        a 4 kg box the body reads zero but the drive keeps pushing with its
        full traction."""
        text = _push_world(box_mass=4.0, wall=False).replace(
            "  collision_mode: 'contact'\n",
            "  collision_mode: 'contact'\n  drive_tau: 0.2\n",
        )
        env = env_factory(_yaml(tmp_path, text))
        robot = env.robot
        self._drive(env, 40)
        assert robot.velocity[0, 0] == pytest.approx(0.0, abs=1e-3)
        assert robot.contact.force[0, 0] == pytest.approx(-robot.friction_force)

    def test_invalid_tau_raises(self, env_factory, tmp_path):
        text = _push_world(wall=False).replace(
            "{name: 'diff'}", "{name: 'diff', tau: -1}"
        )
        with pytest.raises(ValueError, match="tau"):
            env_factory(_yaml(tmp_path, text))

    def test_world_section_sets_the_physics_defaults(self, env_factory, tmp_path):
        """``gravity``, ``friction``, ``restitution`` and ``drive_tau`` under
        ``world`` are what objects start from; a per-object value still wins."""
        env = env_factory(
            _yaml(
                tmp_path,
                "world:\n"
                "  height: 12\n  width: 12\n  step_time: 0.1\n"
                "  collision_mode: 'contact'\n"
                "  gravity: 2.0\n  friction: 0.2\n  restitution: 0.5\n  drive_tau: 0.4\n"
                "robot:\n"
                "  - kinematics: {name: 'diff'}\n"
                "    shape: {name: 'circle', radius: 0.3}\n"
                "    state: [1, 5, 0]\n"
                "    vel_max: [1, 1]\n"
                "obstacle:\n"
                "  - {shape: {name: 'rectangle', length: 0.8, width: 0.8}, state: [3, 5, 0], mass: 1.0}\n"
                "  - {shape: {name: 'rectangle', length: 0.8, width: 0.8}, state: [3, 8, 0], mass: 1.0, "
                "friction: 0.7, restitution: 0.1}\n",
            )
        )
        robot, box, custom = env.robot, env.obstacle_list[0], env.obstacle_list[1]
        assert (robot.friction, robot.restitution, robot.drive_tau) == (0.2, 0.5, 0.4)
        assert (box.friction, box.restitution) == (0.2, 0.5)
        assert (custom.friction, custom.restitution) == (0.7, 0.1)
        assert box.friction_force == pytest.approx(0.2 * 1.0 * 2.0)
        env.step([1.0, 0.0])
        # the lag would allow 0.25, the grip (0.2 * 2 m/s^2) only 0.04 per step
        assert robot.velocity[0, 0] == pytest.approx(0.2 * 2.0 * 0.1)
        self._drive(env, 40)
        x, v = box.state[0, 0], box.velocity_xy[0, 0]
        assert v > 0.8  # the lag and the grip together are still ramping the robot
        env.robot.set_state([1, 9, 0])
        self._drive(env, 5, action=(0.0, 0.0))
        # ground friction 0.2 g at g = 2: the box slows by only 0.04 m/s per step
        assert box.velocity_xy[0, 0] == pytest.approx(v - 5 * 0.2 * 2.0 * 0.1, abs=1e-6)
        assert box.state[0, 0] > x + 0.3

    @pytest.mark.parametrize(
        "world_line",
        [
            "gravity: 0",
            "gravity: -9.81",
            "friction: -0.1",
            "restitution: 1.5",
            "drive_tau: -1",
            "gravity: 'abc'",
        ],
    )
    def test_invalid_world_physics_raises(self, env_factory, tmp_path, world_line):
        text = _push_world().replace(
            "  collision_mode: 'contact'\n",
            f"  collision_mode: 'contact'\n  {world_line}\n",
        )
        with pytest.raises(ValueError, match=world_line.split(":")[0]):
            env_factory(_yaml(tmp_path, text))

    def test_contact_step_without_movable_bodies(self):
        a = ObjectBase(shape={"name": "circle", "radius": 0.5}, state=[0, 0, 0])
        b = ObjectBase(shape={"name": "circle", "radius": 0.5}, state=[0.5, 0, 0])
        assert contact_step([(a, b)]) == []
        assert not a.contact.in_contact

    def test_contact_step_records_normal_and_depth(self):
        a = ObjectBase(
            kinematics={"name": "omni"},
            shape={"name": "circle", "radius": 0.5},
            state=[0, 0, 0],
        )
        b = ObjectBase(shape={"name": "circle", "radius": 0.5}, state=[0.8, 0, 0])
        contacts = contact_step([(a, b)])
        assert len(contacts) == 1
        assert contacts[0].a is a
        assert contacts[0].b is b
        assert np.allclose(contacts[0].normal, [-1, 0])
        assert contacts[0].depth == pytest.approx(0.2 + CONTACT_SLOP)
        assert contacts[0].force == 0.0  # no step time given
        assert a.state[0, 0] == pytest.approx(-0.2 - CONTACT_SLOP)
        assert b.state[0, 0] == pytest.approx(0.8)
        assert a.contact.in_contact
        assert b.contact.in_contact


class TestSolverEdges:
    """Branches of the solver and its helpers that ordinary scenes rarely take."""

    def test_geometry_pieces_of_empty_and_nested_collections(self):
        assert geometry_pieces(shapely.Polygon()) == []
        nested = shapely.GeometryCollection(
            [shapely.GeometryCollection([shapely.box(0, 0, 1, 1)])]
        )
        assert len(geometry_pieces(nested)) == 1

    def test_circle_axis_toward_another_circle(self):
        axes = _axes(_circle(0, 0, 1), _circle(3, 4, 1))
        assert np.allclose(axes, [[-0.6, -0.8]])

    def test_transform_rotates_a_circle_piece(self):
        # a circle never turns from a contact (its normals pass through the
        # center), so the branch is exercised directly
        kind, center, radius = _transform(
            ("circle", np.array([1.0, 0.0]), 0.5), None, np.pi / 2, np.zeros(2)
        )
        assert (kind, radius) == ("circle", 0.5)
        assert np.allclose(center, [0.0, 1.0])

    def test_transform_rotates_a_polygon_without_cached_normals(self):
        piece = ("polygon", np.array([[1.0, 0.0], [2.0, 0.0], [2.0, 1.0]]))
        kind, vertices = _transform(piece, np.array([0.0, 1.0]), np.pi / 2, np.zeros(2))
        assert kind == "polygon"
        assert np.allclose(
            vertices[0], [0.0, 2.0]
        )  # (1, 0) turned to (0, 1), shifted up

    def test_driver_on_the_right_of_a_pair(self):
        """Candidate pairs from the environment put the robot first; a direct
        call with the passive body first takes the mirrored driver branch."""

        def scene(box_mass):
            box = ObjectBase(
                shape={"name": "rectangle", "length": 0.8, "width": 0.8},
                state=[3, 5, 0],
                mass=box_mass,
            )
            robot = ObjectBase(
                kinematics={"name": "omni"},
                shape={"name": "circle", "radius": 0.3},
                state=[2.35, 5, 0],  # overlaps the box's left face by 5 cm
            )
            return box, robot

        box, robot = scene(1.0)  # the robot can push: the box takes it all
        contact_step([(box, robot)])
        assert box.state[0, 0] == pytest.approx(3.05 + CONTACT_SLOP)
        assert robot.state[0, 0] == pytest.approx(2.35)
        box, robot = scene(5.0)  # too heavy: the robot yields instead
        contact_step([(box, robot)])
        assert box.state[0, 0] == pytest.approx(3.0)
        assert robot.state[0, 0] == pytest.approx(2.30 - CONTACT_SLOP)

    def test_squeezed_body_is_left_overlapping(self):
        left = ObjectBase(
            shape={"name": "rectangle", "length": 0.4, "width": 2}, state=[-0.8, 0, 0]
        )
        right = ObjectBase(
            shape={"name": "rectangle", "length": 0.4, "width": 2}, state=[0.8, 0, 0]
        )
        disc = ObjectBase(
            shape={"name": "circle", "radius": 0.7}, state=[0, 0, 0], mass=1
        )
        contacts = contact_step([(disc, left), (disc, right)])
        # pushed out of the left wall, then blocked from moving back: the
        # second pair cannot be resolved and is not reported as a contact
        assert len(contacts) == 1
        assert disc.state[0, 0] == pytest.approx(0.1 + CONTACT_SLOP)
        assert disc.geometry.intersects(right.geometry)

    def test_bounce_already_covered_by_the_position_fold(self):
        """A slow disc found deep inside a wall is thrown out faster by the
        position correction than restitution asks for, so no bounce is added."""
        wall = ObjectBase(
            shape={"name": "rectangle", "length": 0.4, "width": 2},
            state=[1.0, 0, 0],
            restitution=1.0,
        )
        disc = ObjectBase(
            shape={"name": "circle", "radius": 0.4},
            state=[0.45, 0, 0],  # 5 cm into the wall face at x = 0.8
            mass=1.0,
            friction=0,
            restitution=1.0,
        )
        disc.set_velocity([0.1, 0.0, 0.0])
        contact_step([(disc, wall)], step_time=0.1)
        assert disc.velocity_xy[0, 0] == pytest.approx(
            0.1 - (0.05 + CONTACT_SLOP) / 0.1
        )

    def test_compound_body_turns_when_hit_off_center(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 12, width: 12, step_time: 0.1, collision_mode: 'contact'}\n"
                "robot:\n"
                "  - kinematics: {name: 'omni'}\n"
                "    shape: {name: 'circle', radius: 0.2}\n"
                "    state: [2, 5.3, 0]\n"
                "obstacle:\n"
                "  - shape: {name: 'compound', parts: [{name: 'rectangle', length: 0.6, width: 0.6}, "
                "{name: 'circle', radius: 0.3, pose: [0.4, 0.3, 0]}]}\n"
                "    state: [3, 5, 0]\n"
                "    mass: 0.5\n",
            )
        )
        body = env.obstacle_list[0]
        for _ in range(20):
            env.step([1.0, 0.0])
        assert abs(body.state[2, 0]) > 0.1
        assert not env.robot.geometry.intersects(body.geometry)

    def test_polygon_robot_against_a_grid_map(self, env_factory, tmp_path):
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
                "    shape: {name: 'rectangle', length: 1.6, width: 1.0}\n"
                "    state: [5, 5, 0]\n"
                "    vel_max: [4, 1]\n",
            )
        )
        robot = env.robot
        grid = next(obj for obj in env.objects if obj.shape == "map")
        for _ in range(200):
            env.step([3.0, 0.0])
        assert grid in robot.contact.partners
        assert not grid.is_collision(robot.geometry)

    def test_contact_mode_without_objects(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(tmp_path, "world: {height: 5, width: 5, collision_mode: 'contact'}\n")
        )
        env.step()
        assert env.objects == []

    def test_passive_coast_ignores_flat_velocities(self):
        model = PassiveKinematics()
        assert np.allclose(model.coast(np.zeros(3), 0.1, 0.5), 0)
        assert np.allclose(model.coast(np.zeros((1, 1)), 0.1, 0.5), 0)


def _sensor_world():
    return _push_world().replace(
        "    mass: 1.0\nobstacle:",
        "    mass: 1.0\n    sensors:\n      - type: 'contact2d'\nobstacle:",
    )


class TestContact2D:
    """The contact bookkeeping lives in a sensor every object carries."""

    def test_factory_creates_contact2d(self):
        sensor = SensorFactory().create_sensor(
            np.zeros((3, 1)),
            obj_id=1,
            type="contact2d",
            marker_size=3,
            plot={"force_scale": 0.2, "color": "g"},
        )
        assert isinstance(sensor, Contact2D)
        assert sensor.sensor_type == "contact2d"
        assert sensor.force_scale == 0.2
        assert sensor.color == "g"
        assert sensor.marker_size == 3
        assert sensor.parent is None

    def test_bookkeeping(self):
        owner, other = object(), object()
        sensor = Contact2D()
        sensor.parent = owner
        contact = Contact(
            other, owner, np.array([1.0, 0.0]), 0.01, np.array([2.0, 3.0]), 4.0
        )
        sensor.add(contact)
        sensor.add(contact)
        assert sensor.in_contact
        assert sensor.partners == [other]
        assert sensor.records == [contact, contact]
        report = sensor.reports[0]
        assert report.other is other
        assert report.normal.tolist() == [-1.0, 0.0]  # toward the owner
        sensor.add_force([1.0, 2.0])
        sensor.add_force(np.array([[1.0], [0.0]]))
        assert sensor.force.ravel().tolist() == [2.0, 2.0]

        sensor.tick(0.1)
        assert sensor.started
        assert sensor.contact_time == pytest.approx(0.1)
        sensor.clear()
        assert not sensor.in_contact
        assert sensor.reports == []
        sensor.tick(0.1)
        assert sensor.ended
        assert sensor.air_time == pytest.approx(0.1)
        assert sensor.contact_time == 0.0
        sensor.reset()
        assert not sensor.ended
        assert sensor.air_time == 0.0

    def test_every_object_has_a_built_in_sensor(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _push_world()))
        for obj in env.objects:
            assert obj.contact.parent is obj
            assert obj.sensors == []
        robot = env.robot
        for _ in range(25):
            env.step([1.0, 0.0])
        assert robot.contact.in_contact
        assert robot.contact.partners == [env.obstacle_list[0]]

    def test_second_listed_sensor_warns(self, dummy_logger):
        warnings_collected = []
        dummy_logger.warning = lambda msg, *a, **kw: warnings_collected.append(msg)
        obj = ObjectBase(
            shape={"name": "circle", "radius": 0.2},
            sensors=[{"type": "contact2d"}, {"type": "contact2d"}],
        )
        assert obj.contact is obj.sensors[0]
        assert any("contact2d" in w for w in warnings_collected)

    def test_configured_sensor_is_the_objects_sensor(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _sensor_world()))
        robot = env.robot
        assert robot.sensors == [robot.contact]
        assert robot.lidar is None
        assert robot.contact.parent is robot
        for _ in range(25):
            env.step([1.0, 0.0])
        assert robot.contact.in_contact
        assert len(robot.contact.reports) == 1
        assert robot.contact.reports[0].other is env.obstacle_list[0]
        # the same sensor is what the message reads
        state = env.get_msg().robots[0]
        assert state.contacts[0].other_id == env.obstacle_list[0].id
        assert state.scans == []

    def test_plot_draws_points_and_force_lines(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _sensor_world()))
        robot = env.robot
        sensor = robot.contact
        for _ in range(25):
            env.step([1.0, 0.0])
        env.render(0.01)
        xs, ys = sensor._point_artist.get_data()
        report = robot.contact.reports[0]
        assert list(xs) == [pytest.approx(report.point[0])]
        assert list(ys) == [pytest.approx(report.point[1])]
        (segment,) = sensor._force_artist.get_segments()
        length = float(np.linalg.norm(segment[1] - segment[0]))
        assert length == pytest.approx(report.force * sensor.force_scale)
        assert report.force > 0

        env.reset()
        env.render(0.01)
        xs, _ = sensor._point_artist.get_data()
        assert len(xs) == 0  # nothing touches after the reset
        sensor.plot_clear()
        assert sensor._point_artist is None
        sensor.step_plot()  # nothing to update once cleared

    def test_plot_skips_3d_axes(self, env_factory, tmp_path):
        env = env_factory(_yaml(tmp_path, _sensor_world()), projection="3d")
        env.step([1.0, 0.0])
        env.render(0.01)
        assert env.robot.contact._point_artist is None
