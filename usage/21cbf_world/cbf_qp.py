"""Simple CBF-QP controller for the IR-SIM examples.

The controller solves:
    minimize    ||u - u_nom||^2
    subject to  control barrier function constraints
                input bounds

For the two demo modes:
    - ``omni`` uses ``u = [vx, vy]``
    - ``diff`` uses ``u = [v, omega]``
"""

import math

import cvxpy as cp
import numpy as np


class CBFQPController:
    """Generate safe control commands for omni and differential-drive robots."""

    def __init__(
        self,
        robot_type: str = "omni",
        safety_margin: float = 0.05,
        cbf_alpha: float = 2.0,
        goal_gain: float = 0.8,
        angle_gain: float = 2.0,
        angle_tolerance: float = 0.1,
        lookahead: float = 0.25,
    ) -> None:
        """Store controller tuning parameters."""
        self.robot_type = robot_type
        self.safe_margin = float(safety_margin)
        self.alpha = float(cbf_alpha)
        self.goal_gain = float(goal_gain)
        self.angle_gain = float(angle_gain)
        self.angle_tolerance = float(angle_tolerance)
        self.lookahead = float(lookahead)

    def _goal_error(self, robot) -> np.ndarray:
        """Return goal position minus current robot position in x-y."""
        if robot.goal is None:
            return np.zeros(2, dtype=float)
        return robot.goal[:2, 0] - robot.position[:, 0]

    def _nominal_velocity(self, robot) -> np.ndarray:
        """Build the nominal goal-seeking velocity before safety filtering."""
        goal_error = self._goal_error(robot)
        if np.linalg.norm(goal_error) < robot.goal_threshold:
            return np.zeros(2, dtype=float)

        u_nom = self.goal_gain * goal_error
        lower = robot.vel_min[:, 0]
        upper = robot.vel_max[:, 0]
        return np.clip(u_nom, lower, upper)

    def _nominal_lookahead_velocity(self, robot) -> np.ndarray:
        """Build a nominal planar velocity for the unicycle lookahead point."""
        goal_error = self._goal_error(robot)
        if np.linalg.norm(goal_error) < robot.goal_threshold:
            return np.zeros(2, dtype=float)

        theta = float(robot.state[2, 0])
        target_heading = math.atan2(goal_error[1], goal_error[0])
        heading_error = math.atan2(
            math.sin(target_heading - theta), math.cos(target_heading - theta)
        )

        v_max = float(robot.vel_max[0, 0])
        speed = self.goal_gain * np.linalg.norm(goal_error) * max(
            0.0, math.cos(heading_error)
        )
        speed = min(speed, v_max)

        return speed * np.array([math.cos(theta), math.sin(theta)], dtype=float)

    def _dist_barrier_cbf(self, robot, obstacle) -> tuple[np.ndarray, float]:
        """Create one linear CBF inequality for a circular obstacle.

        We use:
            h = ||p - p_obs||^2 - d_safe^2

        and enforce:
            h_dot + alpha * h >= 0
        """
        rel_pos = robot.position[:, 0] - obstacle.position[:, 0]
        safe_distance = robot.radius + obstacle.radius + self.safe_margin
        h_val = float(rel_pos @ rel_pos - safe_distance**2)
        obstacle_velocity = (
            obstacle.velocity[:, 0]
            if obstacle.velocity.shape[0] >= 2
            else np.zeros(2, dtype=float)
        )
        cbf_lhs_row = 2.0 * rel_pos
        cbf_rhs_bound = -self.alpha * h_val + 2.0 * float(
            rel_pos @ obstacle_velocity
        )
        return cbf_lhs_row, cbf_rhs_bound

    def _solve_qp(
        self,
        u_nom: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        cbf_lhs_rows: list[np.ndarray],
        cbf_rhs_bounds: list[float],
    ) -> np.ndarray:
        """Solve the QP and return a safe control command."""
        x0 = np.clip(u_nom, lower, upper)

        if not cbf_lhs_rows:
            return x0

        cbf_constraint_matrix = np.asarray(cbf_lhs_rows, dtype=float)
        cbf_constraint_bounds = np.asarray(cbf_rhs_bounds, dtype=float)

        u = cp.Variable(u_nom.size)
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


        if problem.status in {cp.OPTIMAL, cp.OPTIMAL_INACCURATE} and u.value is not None:
            return np.asarray(u.value, dtype=float).reshape(-1)
        # import pdb; pdb.set_trace()
        print(f"CBF-QP infeasible: {problem.status}")
        return np.zeros_like(u_nom, dtype=float)

    def _solve_unicycle_qp(
        self,
        robot,
        u_nom_xy: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        cbf_lhs_rows: list[np.ndarray],
        cbf_rhs_bounds: list[float],
    ) -> np.ndarray:
        """Solve the lookahead-point unicycle CBF-QP."""
        theta = float(robot.state[2, 0])
        goal_error = self._goal_error(robot)
        target_heading = math.atan2(goal_error[1], goal_error[0])
        desired_omega = self.angle_gain * math.atan2(
            math.sin(target_heading - theta), math.cos(target_heading - theta)
        )
        if abs(desired_omega) < self.angle_tolerance:
            desired_omega = 0.0

        x0 = np.array([min(float(robot.vel_max[0, 0]), 0.5), 0.0], dtype=float)
        x0 = np.clip(x0, lower, upper)

        if not cbf_lhs_rows:
            return x0

        cbf_constraint_matrix = np.asarray(cbf_lhs_rows, dtype=float)
        cbf_constraint_bounds = np.asarray(cbf_rhs_bounds, dtype=float)

        epsilon = self.lookahead
        c = math.cos(theta)
        s = math.sin(theta)
        j_mat = np.array([[c, -epsilon * s], [s, epsilon * c]], dtype=float)

        u = cp.Variable(2)
        objective = cp.Minimize(
            cp.sum_squares(j_mat @ u - u_nom_xy) + 0.1 * cp.square(u[1] - desired_omega)
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

        if problem.status in {cp.OPTIMAL, cp.OPTIMAL_INACCURATE} and u.value is not None:
            return np.asarray(u.value, dtype=float).reshape(-1)
        print(f"CBF-QP infeasible: {problem.status}")
        return np.zeros(2, dtype=float)

    def get_action(self, robot, obstacles) -> np.ndarray:
        """Compute one safe velocity command for the current simulator step."""
        lower = robot.vel_min[:, 0]
        upper = robot.vel_max[:, 0]

        if self.robot_type == "omni":
            u_nom = self._nominal_velocity(robot)
            cbf_lhs_rows = []
            cbf_rhs_bounds = []
            for obstacle in obstacles:
                cbf_lhs_row, cbf_rhs_bound = self._dist_barrier_cbf(
                    robot, obstacle
                )
                cbf_lhs_rows.append(cbf_lhs_row)
                cbf_rhs_bounds.append(cbf_rhs_bound)

            return self._solve_qp(
                u_nom=u_nom,
                lower=lower,
                upper=upper,
                cbf_lhs_rows=cbf_lhs_rows,
                cbf_rhs_bounds=cbf_rhs_bounds,
            )

        if self.robot_type == "diff":
            theta = float(robot.state[2, 0])
            epsilon = self.lookahead
            c = math.cos(theta)
            s = math.sin(theta)
            j_mat = np.array([[c, -epsilon * s], [s, epsilon * c]], dtype=float)
            lookahead_position = robot.position[:, 0] + epsilon * np.array(
                [c, s], dtype=float
            )

            cbf_lhs_rows = []
            cbf_rhs_bounds = []
            for obstacle in obstacles:
                rel_pos = lookahead_position - obstacle.position[:, 0]
                safe_distance = (
                    robot.radius + obstacle.radius + self.safe_margin + epsilon
                )
                h_val = float(rel_pos @ rel_pos - safe_distance**2)
                obstacle_velocity = (
                    obstacle.velocity[:, 0]
                    if obstacle.velocity.shape[0] >= 2
                    else np.zeros(2, dtype=float)
                )
                cbf_lhs_row = 2.0 * rel_pos @ j_mat
                cbf_rhs_bound = -self.alpha * h_val + 2.0 * float(
                    rel_pos @ obstacle_velocity
                )
                cbf_lhs_rows.append(cbf_lhs_row)
                cbf_rhs_bounds.append(cbf_rhs_bound)

            return self._solve_unicycle_qp(
                robot=robot,
                u_nom_xy=self._nominal_lookahead_velocity(robot),
                lower=lower,
                upper=upper,
                cbf_lhs_rows=cbf_lhs_rows,
                cbf_rhs_bounds=cbf_rhs_bounds,
            )

        raise ValueError(f"Unsupported robot_type: {self.robot_type}")


