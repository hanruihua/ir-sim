"""Collision-cone CBF controller for IR-SIM examples.

This file keeps the original ``cbf_qp.py`` untouched and provides a separate
controller inspired by:
    A Collision Cone Approach for Control Barrier Functions
    https://arxiv.org/abs/2403.07043

The paper derives C3BF-QPs for acceleration-controlled unicycle, bicycle, and
quadrotor models. IR-SIM's demo robots are velocity-controlled, so this module
adapts the same collision-cone barrier idea to:
    - ``omni`` with control ``u = [vx, vy]``
    - ``diff`` with control ``u = [v, omega]``

For both cases, the collision-cone barrier is linearized around the current
control. If that linearization point is already inside the cone, the controller
falls back to the standard distance CBF for that obstacle.
"""

import math

import cvxpy as cp
import numpy as np


class CollisionConeCBFController:
    """Generate collision-cone safe commands for omni and diff robots."""

    def __init__(
        self,
        robot_type: str = "omni",
        safety_margin: float = 0.05,
        goal_gain: float = 0.8,
        angle_gain: float = 2.0,
        angle_tolerance: float = 0.1,
        lookahead: float = 0.25,
        distance_alpha: float = 2.0,
        linearization_eps: float = 1e-6,
    ) -> None:
        self.robot_type = robot_type
        self.safe_margin = float(safety_margin)
        self.goal_gain = float(goal_gain)
        self.angle_gain = float(angle_gain)
        self.angle_tolerance = float(angle_tolerance)
        self.lookahead = float(lookahead)
        self.distance_alpha = float(distance_alpha)
        self.linearization_eps = float(linearization_eps)

    def _goal_error(self, robot) -> np.ndarray:
        if robot.goal is None:
            return np.zeros(2, dtype=float)
        return robot.goal[:2, 0] - robot.position[:, 0]

    def _nominal_omni_control(self, robot) -> np.ndarray:
        goal_error = self._goal_error(robot)
        if np.linalg.norm(goal_error) < robot.goal_threshold:
            return np.zeros(2, dtype=float)

        u_nom = self.goal_gain * goal_error
        lower = robot.vel_min[:, 0]
        upper = robot.vel_max[:, 0]
        return np.clip(u_nom, lower, upper)

    def _nominal_diff_control(self, robot) -> np.ndarray:
        goal_error = self._goal_error(robot)
        if np.linalg.norm(goal_error) < robot.goal_threshold:
            return np.zeros(2, dtype=float)

        theta = float(robot.state[2, 0])
        target_heading = math.atan2(goal_error[1], goal_error[0])
        heading_error = math.atan2(
            math.sin(target_heading - theta),
            math.cos(target_heading - theta),
        )

        speed = (
            self.goal_gain
            * np.linalg.norm(goal_error)
            * max(
                0.0,
                math.cos(heading_error),
            )
        )
        desired_omega = self.angle_gain * heading_error
        if abs(desired_omega) < self.angle_tolerance:
            desired_omega = 0.0

        lower = robot.vel_min[:, 0]
        upper = robot.vel_max[:, 0]
        return np.clip(np.array([speed, desired_omega], dtype=float), lower, upper)

    def _current_omni_control(self, robot) -> np.ndarray:
        lower = robot.vel_min[:, 0]
        upper = robot.vel_max[:, 0]
        return np.clip(robot.velocity_xy[:, 0], lower, upper)

    def _current_diff_control(self, robot) -> np.ndarray:
        lower = robot.vel_min[:, 0]
        upper = robot.vel_max[:, 0]
        return np.clip(robot.velocity[:, 0], lower, upper)

    def _obstacle_velocity_xy(self, obstacle) -> np.ndarray:
        if hasattr(obstacle, "velocity_xy"):
            return obstacle.velocity_xy[:, 0]
        if obstacle.velocity.shape[0] >= 2:
            return obstacle.velocity[:2, 0]
        return np.zeros(2, dtype=float)

    def _safe_radius(self, robot, obstacle) -> float:
        return float(robot.radius + obstacle.radius + self.safe_margin)

    def _cone_scale(self, rel_pos: np.ndarray, safe_radius: float) -> float | None:
        distance_sq = float(rel_pos @ rel_pos)
        cone_term_sq = distance_sq - safe_radius**2
        if cone_term_sq <= self.linearization_eps:
            return None
        return math.sqrt(cone_term_sq)

    def _distance_constraint_for_omni(
        self,
        robot,
        obstacle,
    ) -> tuple[np.ndarray, float]:
        rel_pos = obstacle.position[:, 0] - robot.position[:, 0]
        obstacle_velocity = self._obstacle_velocity_xy(obstacle)
        safe_radius = self._safe_radius(robot, obstacle)
        h_val = float(rel_pos @ rel_pos - safe_radius**2)
        cbf_lhs_row = -2.0 * rel_pos
        cbf_rhs_bound = -self.distance_alpha * h_val - 2.0 * float(
            rel_pos @ obstacle_velocity
        )
        return cbf_lhs_row, cbf_rhs_bound

    def _distance_constraint_for_diff(
        self,
        robot,
        obstacle,
    ) -> tuple[np.ndarray, float]:
        point_position, point_jacobian = self._diff_point_kinematics(robot)
        rel_pos = obstacle.position[:, 0] - point_position
        obstacle_velocity = self._obstacle_velocity_xy(obstacle)
        safe_radius = self._safe_radius(robot, obstacle)
        h_val = float(rel_pos @ rel_pos - safe_radius**2)
        cbf_lhs_row = -2.0 * rel_pos @ point_jacobian
        cbf_rhs_bound = -self.distance_alpha * h_val - 2.0 * float(
            rel_pos @ obstacle_velocity
        )
        return np.asarray(cbf_lhs_row, dtype=float), cbf_rhs_bound

    def _linearized_cone_constraint_for_omni(
        self,
        robot,
        obstacle,
        u_linearize: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        rel_pos = obstacle.position[:, 0] - robot.position[:, 0]
        safe_radius = self._safe_radius(robot, obstacle)
        cone_scale = self._cone_scale(rel_pos, safe_radius)
        if cone_scale is None:
            return self._distance_constraint_for_omni(robot, obstacle)

        obstacle_velocity = self._obstacle_velocity_xy(obstacle)
        rel_velocity = u_linearize - obstacle_velocity
        rel_speed = float(np.linalg.norm(rel_velocity))
        if rel_speed < self.linearization_eps:
            rel_velocity = (
                -self.linearization_eps
                * rel_pos
                / max(
                    np.linalg.norm(rel_pos),
                    self.linearization_eps,
                )
            )
            rel_speed = float(np.linalg.norm(rel_velocity))

        h_val = -float(rel_pos @ rel_velocity) + cone_scale * rel_speed
        if h_val <= 0.0:
            return self._distance_constraint_for_omni(robot, obstacle)

        grad_h = -rel_pos + cone_scale * rel_velocity / rel_speed
        cbf_lhs_row = grad_h
        cbf_rhs_bound = float(grad_h @ u_linearize - h_val)
        return cbf_lhs_row, cbf_rhs_bound

    def _diff_point_kinematics(self, robot) -> tuple[np.ndarray, np.ndarray]:
        theta = float(robot.state[2, 0])
        c = math.cos(theta)
        s = math.sin(theta)
        point_position = robot.position[:, 0] + self.lookahead * np.array(
            [c, s],
            dtype=float,
        )
        point_jacobian = np.array(
            [[c, -self.lookahead * s], [s, self.lookahead * c]],
            dtype=float,
        )
        return point_position, point_jacobian

    def _linearized_cone_constraint_for_diff(
        self,
        robot,
        obstacle,
        u_linearize: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        point_position, point_jacobian = self._diff_point_kinematics(robot)
        rel_pos = obstacle.position[:, 0] - point_position
        safe_radius = self._safe_radius(robot, obstacle)
        cone_scale = self._cone_scale(rel_pos, safe_radius)
        if cone_scale is None:
            return self._distance_constraint_for_diff(robot, obstacle)

        obstacle_velocity = self._obstacle_velocity_xy(obstacle)
        rel_velocity = point_jacobian @ u_linearize - obstacle_velocity
        rel_speed = float(np.linalg.norm(rel_velocity))
        if rel_speed < self.linearization_eps:
            rel_velocity = (
                -self.linearization_eps
                * rel_pos
                / max(
                    np.linalg.norm(rel_pos),
                    self.linearization_eps,
                )
            )
            rel_speed = float(np.linalg.norm(rel_velocity))

        h_val = -float(rel_pos @ rel_velocity) + cone_scale * rel_speed
        if h_val <= 0.0:
            return self._distance_constraint_for_diff(robot, obstacle)

        grad_q = -rel_pos + cone_scale * rel_velocity / rel_speed
        grad_h = point_jacobian.T @ grad_q
        cbf_lhs_row = grad_h
        cbf_rhs_bound = float(grad_h @ u_linearize - h_val)
        return np.asarray(cbf_lhs_row, dtype=float), cbf_rhs_bound

    def _solve_omni_qp(
        self,
        u_nom: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        cbf_lhs_rows: list[np.ndarray],
        cbf_rhs_bounds: list[float],
    ) -> np.ndarray:
        x0 = np.clip(u_nom, lower, upper)
        if not cbf_lhs_rows:
            return x0

        cbf_constraint_matrix = np.asarray(cbf_lhs_rows, dtype=float)
        cbf_constraint_bounds = np.asarray(cbf_rhs_bounds, dtype=float)

        u = cp.Variable(2)
        objective = cp.Minimize(cp.sum_squares(u - u_nom))
        qp_constraints = [
            cbf_constraint_matrix @ u >= cbf_constraint_bounds,
            u >= lower,
            u <= upper,
        ]
        problem = cp.Problem(objective, qp_constraints)
        u.value = x0

        try:
            problem.solve(solver=cp.OSQP)
        except Exception:
            problem.solve()

        if (
            problem.status in {cp.OPTIMAL, cp.OPTIMAL_INACCURATE}
            and u.value is not None
        ):
            return np.asarray(u.value, dtype=float).reshape(-1)
        print(f"C3BF-QP infeasible: {problem.status}")
        return np.zeros(2, dtype=float)

    def _solve_diff_qp(
        self,
        robot,
        u_nom: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        cbf_lhs_rows: list[np.ndarray],
        cbf_rhs_bounds: list[float],
    ) -> np.ndarray:
        x0 = np.clip(u_nom, lower, upper)
        if not cbf_lhs_rows:
            return x0

        _, point_jacobian = self._diff_point_kinematics(robot)
        point_velocity_nom = point_jacobian @ u_nom

        cbf_constraint_matrix = np.asarray(cbf_lhs_rows, dtype=float)
        cbf_constraint_bounds = np.asarray(cbf_rhs_bounds, dtype=float)

        u = cp.Variable(2)
        objective = cp.Minimize(
            cp.sum_squares(point_jacobian @ u - point_velocity_nom)
            + 0.1 * cp.square(u[1] - u_nom[1])
        )
        qp_constraints = [
            cbf_constraint_matrix @ u >= cbf_constraint_bounds,
            u >= lower,
            u <= upper,
        ]
        problem = cp.Problem(objective, qp_constraints)
        u.value = x0

        try:
            problem.solve(solver=cp.OSQP)
        except Exception:
            problem.solve()

        if (
            problem.status in {cp.OPTIMAL, cp.OPTIMAL_INACCURATE}
            and u.value is not None
        ):
            return np.asarray(u.value, dtype=float).reshape(-1)
        print(f"C3BF-QP infeasible: {problem.status}")
        return np.zeros(2, dtype=float)

    def get_action(self, robot, obstacles) -> np.ndarray:
        lower = robot.vel_min[:, 0]
        upper = robot.vel_max[:, 0]

        if self.robot_type == "omni":
            u_nom = self._nominal_omni_control(robot)
            u_linearize = self._current_omni_control(robot)
            cbf_lhs_rows = []
            cbf_rhs_bounds = []
            for obstacle in obstacles:
                cbf_lhs_row, cbf_rhs_bound = self._linearized_cone_constraint_for_omni(
                    robot,
                    obstacle,
                    u_linearize,
                )
                cbf_lhs_rows.append(cbf_lhs_row)
                cbf_rhs_bounds.append(cbf_rhs_bound)

            return self._solve_omni_qp(
                u_nom=u_nom,
                lower=lower,
                upper=upper,
                cbf_lhs_rows=cbf_lhs_rows,
                cbf_rhs_bounds=cbf_rhs_bounds,
            )

        if self.robot_type == "diff":
            u_nom = self._nominal_diff_control(robot)
            u_linearize = self._current_diff_control(robot)
            cbf_lhs_rows = []
            cbf_rhs_bounds = []
            for obstacle in obstacles:
                cbf_lhs_row, cbf_rhs_bound = self._linearized_cone_constraint_for_diff(
                    robot,
                    obstacle,
                    u_linearize,
                )
                cbf_lhs_rows.append(cbf_lhs_row)
                cbf_rhs_bounds.append(cbf_rhs_bound)

            return self._solve_diff_qp(
                robot=robot,
                u_nom=u_nom,
                lower=lower,
                upper=upper,
                cbf_lhs_rows=cbf_lhs_rows,
                cbf_rhs_bounds=cbf_rhs_bounds,
            )

        raise ValueError(f"Unsupported robot_type: {self.robot_type}")
