"""
This file is the implementation of the Reciprocal Velocity Obstacle (RVO) algorithm for multi-robot collision avoidance.

Author: Ruihua Han

reference: https://github.com/MengGuo/RVO_Py_MAS
"""

from math import asin, atan2, cos, pi, sin, sqrt

import numpy as np

from irsim.util.util import dist_hypot, log_error


class reciprocal_vel_obs:
    """
    A class to implement the Reciprocal Velocity Obstacle (RVO) algorithm for multi-robot collision avoidance.

    Args:

        state (list): The rvo state of the agent [x, y, vx, vy, radius, vx_des, vy_des].
        obs_state_list (list) : List of states of static obstacles [[x, y, vx, vy, radius]].
        vxmax (float): Maximum velocity in the x direction.
        vymax (float): Maximum velocity in the y direction.
        acce (float): Acceleration limit.
        factor (float): Penalty weighting factor for velocity selection.
        line_obs_list (list): List of line segments [[x1, y1, x2, y2], ...].
    """

    def __init__(
        self,
        state: list,
        obs_state_list=None,
        vxmax=1.5,
        vymax=1.5,
        acce=0.5,
        factor=1.0,
        line_obs_list=None,
    ):
        if obs_state_list is None:
            obs_state_list = []
        if line_obs_list is None:
            line_obs_list = []
        self.state = state
        self.obs_state_list = obs_state_list
        self.line_obs_list = line_obs_list
        self.vxmax = vxmax
        self.vymax = vymax
        self.acce = acce
        self.factor = factor

    def update(self, state, obs_state_list, line_obs_list=None):
        """Update the agent, circular-obstacle, and line-obstacle states."""
        self.state = state
        self.obs_state_list = obs_state_list
        self.line_obs_list = line_obs_list if line_obs_list is not None else []

    def cal_vel(self, mode="rvo"):
        """
        Calculate the velocity of the agent based on the Reciprocal Velocity Obstacle (RVO) algorithm.

        Args:
            mode (str): The vo configure to calculate the velocity. It can be "rvo", "hrvo", or "vo".
                - rvo: Reciprocal Velocity Obstacle (RVO) algorithm, for multi-robot collision avoidance.
                - hrvo: Hybrid Reciprocal Velocity Obstacle (HRVO) algorithm, for multi-robot collision avoidance.
                - vo: Velocity Obstacle (VO) algorithm, for obstacle-robot collision avoidance.

        Returns:
            list[float]: Selected velocity [vx, vy].
        """

        if mode == "rvo":
            rvo_list = self.config_rvo()
        elif mode == "hrvo":
            rvo_list = self.config_hrvo()
        elif mode == "vo":
            rvo_list = self.config_vo()

        else:  # pragma: no cover - defensive guard; callers pass a valid mode
            log_error("wrong method mode, please input vo, rvo or hrvo")

        velocities, outside = self._candidate_velocities(rvo_list)
        return self.vel_select(velocities[outside], velocities[~outside])

    @staticmethod
    def _cone_angles(
        x: float, y: float, r: float, mx: float, my: float, mr: float
    ) -> tuple[float, float, float]:
        """
        Compute the edges of the velocity-obstacle cone towards one obstacle.

        The cone spans the directions from ``(x, y)`` that touch the obstacle
        disc at ``(mx, my)`` inflated by the combined radius. Obstacles closer
        than that combined radius are treated as tangent, giving a half cone of
        90 degrees instead of an undefined one.

        Args:
            x (float): Ego x position.
            y (float): Ego y position.
            r (float): Ego radius.
            mx (float): Obstacle x position.
            my (float): Obstacle y position.
            mr (float): Obstacle radius.

        Returns:
            tuple: ``(line_left_ori, line_right_ori, half_angle)`` in radians.
        """

        dis_mr = np.sqrt((my - y) ** 2 + (mx - x) ** 2)
        angle_mr = atan2(my - y, mx - x)

        if dis_mr < r + mr:
            dis_mr = r + mr

        ratio = min(max((r + mr) / dis_mr, -1.0), 1.0)
        half_angle = asin(ratio)

        return angle_mr + half_angle, angle_mr - half_angle, half_angle

    @staticmethod
    def _neighbour_state(obstacle):
        """Unpack a neighbour as ``(x, y, vx, vy, r)``.

        Accepts the moving form ``[x, y, vx, vy, r]`` and the static form
        ``[x, y, r]``, which is a neighbour with zero velocity. The layout is
        told from the length of ``obstacle``; the ego's own velocity plays no
        part in it.
        """
        if len(obstacle) == 3:
            x, y, r = obstacle
            return x, y, 0.0, 0.0, r
        return obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4]

    def config_rvo(self):
        """Build reciprocal velocity-obstacle cones for all obstacles."""
        rvo_list = []

        for obstacle in self.obs_state_list:
            rvo = self.config_rvo_mode(obstacle)
            rvo_list.append(rvo)

        rvo_list.extend(self.config_vo_lines())

        return rvo_list

    def config_rvo_mode(self, obstacle):
        """Build one RVO cone for a circular obstacle.

        Args:
            obstacle: Neighbour state ``[x, y, vx, vy, radius]``, or the
                static form ``[x, y, radius]`` (zero velocity).

        Returns:
            list: ``[apex, left_vector, right_vector]`` cone description.
        """
        x = self.state[0]
        y = self.state[1]
        vx = self.state[2]
        vy = self.state[3]
        r = self.state[4]

        # A static neighbour is one with zero velocity; the reciprocal apex
        # handles it without a special case.
        mx, my, mvx, mvy, mr = self._neighbour_state(obstacle)
        rvo_apex = [(vx + mvx) / 2, (vy + mvy) / 2]

        line_left_ori, line_right_ori, _ = self._cone_angles(x, y, r, mx, my, mr)
        line_left_vector = [cos(line_left_ori), sin(line_left_ori)]
        line_right_vector = [cos(line_right_ori), sin(line_right_ori)]

        return [rvo_apex, line_left_vector, line_right_vector]

    def config_hrvo(self):
        """Build hybrid reciprocal velocity-obstacle cones for all obstacles."""
        hrvo_list = []

        for obstacle in self.obs_state_list:
            # for circular: [x, y, radius]
            hrvo = self.config_hrvo_mode(obstacle)
            hrvo_list.append(hrvo)

        hrvo_list.extend(self.config_vo_lines())

        return hrvo_list

    def config_hrvo_mode(self, obstacle):
        """Build one HRVO cone for a circular obstacle.

        Args:
            obstacle: Neighbour state ``[x, y, vx, vy, radius]``, or the
                static form ``[x, y, radius]`` (zero velocity).

        Returns:
            list: ``[apex, left_vector, right_vector]`` cone description.
        """
        x = self.state[0]
        y = self.state[1]
        vx = self.state[2]
        vy = self.state[3]
        r = self.state[4]

        mx, my, mvx, mvy, mr = self._neighbour_state(obstacle)
        rvo_apex = [(vx + mvx) / 2, (vy + mvy) / 2]
        vo_apex = [mvx, mvy]

        line_left_ori, line_right_ori, half_angle = self._cone_angles(
            x, y, r, mx, my, mr
        )
        line_left_vector = [cos(line_left_ori), sin(line_left_ori)]
        line_right_vector = [cos(line_right_ori), sin(line_right_ori)]

        cl_vector = [mx - x, my - y]

        cur_v = [vx - rvo_apex[0], vy - rvo_apex[1]]

        dis_rv = dist_hypot(rvo_apex[0], rvo_apex[1], vo_apex[0], vo_apex[1])
        radians_rv = atan2(rvo_apex[1] - vo_apex[1], rvo_apex[0] - vo_apex[0])

        diff = line_left_ori - radians_rv

        temp = pi - 2 * half_angle

        if temp == 0:
            temp = temp + 0.01

        dis_diff = dis_rv * sin(diff) / sin(temp)

        if reciprocal_vel_obs.cross_product(cl_vector, cur_v) <= 0:
            hrvo_apex = [
                rvo_apex[0] - dis_diff * cos(line_right_ori),
                rvo_apex[1] - dis_diff * sin(line_right_ori),
            ]
        else:
            hrvo_apex = [
                vo_apex[0] + dis_diff * cos(line_right_ori),
                vo_apex[1] + dis_diff * sin(line_right_ori),
            ]

        return [hrvo_apex, line_left_vector, line_right_vector]

    def config_vo(self):
        """Build standard velocity-obstacle cones for all obstacles."""
        vo_list = []

        for obstacle in self.obs_state_list:
            vo = self.config_vo_mode(obstacle)
            vo_list.append(vo)

        vo_list.extend(self.config_vo_lines())

        return vo_list

    def config_vo_mode(self, obstacle):
        """Build one VO cone for a circular obstacle.

        Args:
            obstacle: Neighbour state ``[x, y, vx, vy, radius]``, or the
                static form ``[x, y, radius]`` (zero velocity).

        Returns:
            list: ``[apex, left_vector, right_vector]`` cone description.
        """
        x = self.state[0]
        y = self.state[1]
        r = self.state[4]

        mx, my, mvx, mvy, mr = self._neighbour_state(obstacle)
        vo_apex = [mvx, mvy]
        line_left_ori, line_right_ori, _ = self._cone_angles(x, y, r, mx, my, mr)
        line_left_vector = [cos(line_left_ori), sin(line_left_ori)]
        line_right_vector = [cos(line_right_ori), sin(line_right_ori)]

        return [vo_apex, line_left_vector, line_right_vector]

    def config_vo_lines(self):
        """Compute VO cones for line segment obstacles.

        For each segment, compute the angular span as seen from the agent,
        expanded by asin(r / dist) on each side to account for the agent radius.
        The apex is [0, 0] since line obstacles are static.
        """
        vo_list = []
        x = self.state[0]
        y = self.state[1]
        r = self.state[4]

        for seg in self.line_obs_list:
            x1, y1, x2, y2 = seg

            # Closest point on segment to agent
            dx_seg = x2 - x1
            dy_seg = y2 - y1
            seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg

            if seg_len_sq < 1e-12:
                continue

            t = ((x - x1) * dx_seg + (y - y1) * dy_seg) / seg_len_sq
            t = max(0.0, min(1.0, t))
            cx = x1 + t * dx_seg
            cy = y1 + t * dy_seg
            dist_closest = sqrt((x - cx) ** 2 + (y - cy) ** 2)

            if dist_closest < 1e-6:
                dist_closest = 1e-6

            # Angles from agent to each endpoint
            angle1 = atan2(y1 - y, x1 - x)
            angle2 = atan2(y2 - y, x2 - x)

            # Expand by agent radius at each endpoint
            dist1 = sqrt((x1 - x) ** 2 + (y1 - y) ** 2)
            dist2 = sqrt((x2 - x) ** 2 + (y2 - y) ** 2)

            expand1 = asin(min(1.0, r / max(dist1, r))) if dist1 > 1e-6 else pi / 2
            expand2 = asin(min(1.0, r / max(dist2, r))) if dist2 > 1e-6 else pi / 2

            # Determine which endpoint is "left" and which is "right"
            # by checking the angular difference
            diff = atan2(sin(angle2 - angle1), cos(angle2 - angle1))

            if diff >= 0:
                # angle2 is to the left of angle1
                left_angle = angle2 + expand2
                right_angle = angle1 - expand1
            else:
                # angle1 is to the left of angle2
                left_angle = angle1 + expand1
                right_angle = angle2 - expand2

            line_left_vector = [cos(left_angle), sin(left_angle)]
            line_right_vector = [cos(right_angle), sin(right_angle)]

            vo_list.append([[0, 0], line_left_vector, line_right_vector])

        return vo_list

    def vel_candidate(self, rvo_list):
        """Sample reachable velocities and split them by VO feasibility.

        Args:
            rvo_list: Velocity-obstacle cone descriptions ``[apex, left, right]``.

        Returns:
            tuple[list, list]: Feasible velocities outside all cones and
            infeasible velocities inside at least one cone.
        """
        velocities, outside = self._candidate_velocities(rvo_list)
        return velocities[outside].tolist(), velocities[~outside].tolist()

    def _axis_candidates(self, current, vmax):
        """Velocities reachable along one axis within ``acce`` of ``current``.

        The window is clipped to ``[-vmax, vmax]``. When the current velocity
        lies more than ``acce`` outside that range (an initial or externally
        set velocity above the limit), the window would be empty and the
        selection would fail, so the nearest admissible velocity is the one
        candidate instead.
        """
        low = max(current - self.acce, -vmax)
        high = min(current + self.acce, vmax)
        if high <= low:
            return np.array([float(np.clip(current, -vmax, vmax))])
        return np.arange(low, high, 0.05)

    def _candidate_velocities(self, rvo_list):
        """Reachable velocity grid and a mask of those outside every cone.

        The grid is evaluated against all cones at once with numpy, ordered as
        the nested ``vx``/``vy`` loops would produce it so that
        :meth:`vel_select` picks the same velocity.
        """
        vxs = self._axis_candidates(self.state[2], self.vxmax)
        vys = self._axis_candidates(self.state[3], self.vymax)
        grid_x, grid_y = np.meshgrid(vxs, vys, indexing="ij")
        vx, vy = grid_x.ravel(), grid_y.ravel()

        outside = np.ones(vx.size, dtype=bool)
        for apex, left, right, *_ in rvo_list:
            rel_x, rel_y = vx - apex[0], vy - apex[1]
            inside = (left[0] * rel_y - rel_x * left[1] <= 0) & (
                right[0] * rel_y - rel_x * right[1] >= 0
            )
            outside &= ~inside

        return np.column_stack((vx, vy)), outside

    def vo_out(self, vx, vy, rvo_list):
        """Return whether a candidate velocity lies outside every VO cone."""
        for rvo in rvo_list:
            rel_vx = vx - rvo[0][0]
            rel_vy = vy - rvo[0][1]

            rel_vector = [rel_vx, rel_vy]

            if reciprocal_vel_obs.between_vector(rvo[1], rvo[2], rel_vector):
                return False

        return True

    def vel_select(self, vo_outside, vo_inside):
        """Select the best velocity from feasible candidates or penalized fallback.

        Both arguments may be lists of ``[vx, vy]`` or ``(N, 2)`` arrays; the
        first minimum is returned, as ``min`` would.
        """
        vel_des = [self.state[5], self.state[6]]

        if len(vo_outside) != 0:
            velocities = np.asarray(vo_outside)
            distances = np.hypot(
                velocities[:, 0] - vel_des[0], velocities[:, 1] - vel_des[1]
            )
            return list(velocities[int(np.argmin(distances))])

        velocities = np.asarray(vo_inside)
        costs = self.penalties(velocities, vel_des, self.factor)
        return list(velocities[int(np.argmin(costs))])

    def penalties(self, vels, vel_des, factor):
        """Vectorized :meth:`penalty` for an ``(N, 2)`` array of velocities.

        Same formulas and branches as the scalar version, evaluated for all
        candidates at once; the crowded-scene fallback otherwise calls
        ``penalty`` hundreds of times per robot per step.
        """
        x, y, vx, vy, r = self.state[:5]
        tc_columns = []
        for moving in self.obs_state_list:
            distance = dist_hypot(moving[0], moving[1], x, y)
            diff = max(distance**2 - (r + moving[4]) ** 2, 0)
            dis_vel = np.sqrt(diff)
            trans_x = 2 * vels[:, 0] - vx - moving[2]
            trans_y = 2 * vels[:, 1] - vy - moving[3]
            tc_columns.append(dis_vel / (np.sqrt(trans_x**2 + trans_y**2) + 1e-7))
        for seg in self.line_obs_list:
            tc_columns.append(self._tc_line_segment_all(x, y, r, vels, seg))

        dist_des = np.hypot(vels[:, 0] - vel_des[0], vels[:, 1] - vel_des[1])
        if not tc_columns:
            return dist_des
        tc_min = np.min(tc_columns, axis=0)
        tc_min = np.where(tc_min == 0, 0.0001, tc_min)
        return factor * (1 / tc_min) + dist_des

    def penalty(self, vel, vel_des, factor):
        """Scalar :meth:`penalties` for one velocity, kept for compatibility."""
        return float(self.penalties(np.array([vel], dtype=float), vel_des, factor)[0])

    @staticmethod
    def _tc_line_segment_all(x, y, r, vels, seg):
        """Vectorized :meth:`_tc_line_segment` for an ``(N, 2)`` array of velocities."""
        x1, y1, x2, y2 = seg
        dx_seg, dy_seg = x2 - x1, y2 - y1
        seg_len = sqrt(dx_seg * dx_seg + dy_seg * dy_seg)
        if seg_len < 1e-12:
            return np.full(len(vels), 1e6)
        nx, ny = -dy_seg / seg_len, dx_seg / seg_len
        if nx * (x - x1) + ny * (y - y1) < 0:
            nx, ny = -nx, -ny
        perp_dist = nx * (x - x1) + ny * (y - y1)

        vel_toward = -(nx * vels[:, 0] + ny * vels[:, 1])
        approaching = vel_toward > 1e-7
        tc = np.maximum((perp_dist - r) / np.where(approaching, vel_toward, 1.0), 0)
        col_x = x + vels[:, 0] * tc
        col_y = y + vels[:, 1] * tc
        t_proj = ((col_x - x1) * dx_seg + (col_y - y1) * dy_seg) / (seg_len * seg_len)
        margin = r / seg_len
        hits = approaching & (t_proj >= -margin) & (t_proj <= 1 + margin)
        return np.where(hits, tc, 1e6)

    @staticmethod
    def between_vector(line_left_vector, line_right_vector, line_vector):
        """Return whether ``line_vector`` lies inside a cone boundary pair."""
        return bool(
            reciprocal_vel_obs.cross_product(line_left_vector, line_vector) <= 0
            and reciprocal_vel_obs.cross_product(line_right_vector, line_vector) >= 0
        )

    @staticmethod
    def cross_product(vector1, vector2):
        """Compute the 2D cross product ``vector1 x vector2``."""
        return float(vector1[0] * vector2[1] - vector2[0] * vector1[1])
