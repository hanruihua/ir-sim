"""
Social Force Model (SFM) for pedestrian / mobile-agent navigation.

Independent Python implementation of the Social Force Model described in:

    Helbing, D. & Molnar, P. (1995), *Social force model for pedestrian
    dynamics.* Phys. Rev. E 51:4282.

    Moussaid, M., Helbing, D., Garnier, S., Johansson, A., Combe, M.,
    Theraulaz, G. (2009), *Experimental study of the behavioural
    mechanisms underlying self-organization in human crowds.*
    Proc. R. Soc. B 276:2755.

The neighbor interaction is the anisotropic, velocity-aware variant from
Moussaid-Helbing (2009); the obstacle term is the Helbing-Molnar (1995)
summation form -- every nearby line obstacle contributes an exponentially
decayed push, which (unlike a nearest-segment-only rule) lets symmetric
walls cancel and keeps the agent on the centreline.

Each agent integrates Newton-like dynamics ``v += dt * a`` where ``a``
is the weighted sum of three forces:

* desired force  -- relaxation of velocity toward ``v0 * e_goal``;
* social force   -- anisotropic repulsion from each neighbor
  (Moussaid-Helbing 2009 form);
* obstacle force -- exponential repulsion summed over all line
  obstacles within ``5 * sigma_obstacle`` (Helbing-Molnar 1995 form).

:class:`social_force_model` evaluates these forces for one agent at a time
and backs the per-object ``sfm`` behavior. :class:`SocialForceModelBatch`
evaluates the same forces for a whole crowd in one NumPy pass from a single
state snapshot and adds the optional social-group forces of Moussaid et al.
(2010); it backs the ``sfm`` group behavior.

Acknowledgement:

    pedsim_ros (https://github.com/srl-freiburg/pedsim_ros) and its
    vendored ``libpedsim`` by Christian Gloor were consulted as
    reference implementations while writing this module.
"""

from math import atan2, exp, sqrt

import numpy as np


class social_force_model:
    """Social Force Model controller for a single agent.

    The interface mirrors :class:`reciprocal_vel_obs` so the two
    algorithms are interchangeable from a behavior method.

    Args:
        state (list): Agent state ``[x, y, vx, vy, radius, vx_des, vy_des, theta]``.
        neighbor_list (list): Other moving/static circular agents
            ``[[x, y, vx, vy, radius], ...]``.
        line_obs_list (list): Line obstacles
            ``[[x1, y1, x2, y2], ...]``.
        vmax (float): Speed cap applied after the velocity update.
        step_time (float): Integration step ``dt``.
        relaxation_time (float): ``tau`` in the desired-force term.
        force_factor_desired (float): Weight ``alpha_D`` on the desired force.
        force_factor_social (float): Weight ``alpha_S`` on the social force.
        force_factor_obstacle (float): Weight ``alpha_O`` on the obstacle force.
        sigma_obstacle (float): Decay length of the obstacle repulsion.
        lambda_importance (float): Weight of relative velocity in the
            interaction direction (``lambda`` in Moussaïd 2009).
        gamma (float): Sets the interaction range
            ``B = gamma * ||t||``.
        n_angular (float): Angular sharpness ``n`` for the sideways force.
        n_velocity (float): Angular sharpness ``n'`` for the slowdown force.
        neighbor_range (float): Max distance for an agent to count as a
            social-force neighbor.
        safety_radius (float): Personal-space buffer subtracted from the
            agent-to-agent distance inside the social-force exponential.
            ``0`` reproduces the upstream behavior (point agents).
            ``> 0`` shifts the decay closer-in so the repulsion saturates
            at ``2 * safety_radius`` of centre-to-centre clearance,
            effectively giving each agent a body radius for SFM.
    """

    def __init__(
        self,
        state: list,
        neighbor_list: list | None = None,
        line_obs_list: list | None = None,
        vmax: float = 1.5,
        step_time: float = 0.1,
        relaxation_time: float = 0.5,
        force_factor_desired: float = 1.0,
        force_factor_social: float = 2.1,
        force_factor_obstacle: float = 10.0,
        sigma_obstacle: float = 0.8,
        lambda_importance: float = 2.0,
        gamma: float = 0.35,
        n_angular: float = 2.0,
        n_velocity: float = 3.0,
        neighbor_range: float = 10.0,
        safety_radius: float = 0.0,
    ) -> None:
        # Catch non-positive divisors early — these would otherwise raise
        # ``ZeroDivisionError`` deep inside the per-neighbour loop or
        # silently produce non-physical dynamics on the first ``cal_vel``.
        if relaxation_time <= 0.0:
            raise ValueError(f"relaxation_time must be > 0, got {relaxation_time}")
        if gamma <= 0.0:
            raise ValueError(f"gamma must be > 0, got {gamma}")
        if sigma_obstacle <= 0.0:
            raise ValueError(f"sigma_obstacle must be > 0, got {sigma_obstacle}")

        self.state = state
        self.neighbor_list = neighbor_list if neighbor_list is not None else []
        self.line_obs_list = line_obs_list if line_obs_list is not None else []

        self.vmax = vmax
        self.step_time = step_time
        self.relaxation_time = relaxation_time

        self.force_factor_desired = force_factor_desired
        self.force_factor_social = force_factor_social
        self.force_factor_obstacle = force_factor_obstacle

        self.sigma_obstacle = sigma_obstacle
        self.lambda_importance = lambda_importance
        self.gamma = gamma
        self.n_angular = n_angular
        self.n_velocity = n_velocity
        self.neighbor_range = neighbor_range
        self.safety_radius = safety_radius

    def update(
        self,
        state: list,
        neighbor_list: list,
        line_obs_list: list | None = None,
    ) -> None:
        """Refresh the per-step inputs without re-instantiating."""
        self.state = state
        self.neighbor_list = neighbor_list
        self.line_obs_list = line_obs_list if line_obs_list is not None else []

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def cal_vel(self) -> list:
        """Integrate one SFM step and return the new global velocity.

        Returns:
            list[float]: Updated velocity ``[vx, vy]``, clipped to ``vmax``.
        """
        fd = self.desired_force()
        fs = self.social_force()
        fo = self.obstacle_force()

        ax = (
            self.force_factor_desired * fd[0]
            + self.force_factor_social * fs[0]
            + self.force_factor_obstacle * fo[0]
        )
        ay = (
            self.force_factor_desired * fd[1]
            + self.force_factor_social * fs[1]
            + self.force_factor_obstacle * fo[1]
        )

        vx = self.state[2] + self.step_time * ax
        vy = self.state[3] + self.step_time * ay

        speed = sqrt(vx * vx + vy * vy)
        if speed > self.vmax and speed > 1e-9:
            scale = self.vmax / speed
            vx *= scale
            vy *= scale

        return [vx, vy]

    # ------------------------------------------------------------------
    # Individual force terms
    # ------------------------------------------------------------------

    def desired_force(self) -> list:
        """Relaxation toward the desired velocity ``v0 * e_goal``.

        The desired velocity is supplied directly via ``state[5:7]``.
        """
        vx = self.state[2]
        vy = self.state[3]
        vx_des = self.state[5]
        vy_des = self.state[6]

        return [
            (vx_des - vx) / self.relaxation_time,
            (vy_des - vy) / self.relaxation_time,
        ]

    def social_force(self) -> list:
        """Anisotropic neighbor repulsion (Moussaid-Helbing 2009).

        Iterates over all neighbors within ``neighbor_range`` and sums
        their contribution.
        """
        x = self.state[0]
        y = self.state[1]
        vx = self.state[2]
        vy = self.state[3]

        fx = 0.0
        fy = 0.0

        for nb in self.neighbor_list:
            mx, my, mvx, mvy, _mr = nb

            dx = mx - x
            dy = my - y
            dist = sqrt(dx * dx + dy * dy)
            if dist < 1e-6 or dist > self.neighbor_range:
                continue

            d_hat_x = dx / dist
            d_hat_y = dy / dist

            # velocity diff is self - other (upstream convention)
            vdx = vx - mvx
            vdy = vy - mvy

            # interaction vector  t = lambda * dv + d_hat
            tx = self.lambda_importance * vdx + d_hat_x
            ty = self.lambda_importance * vdy + d_hat_y
            t_norm = sqrt(tx * tx + ty * ty)
            if t_norm < 1e-9:
                continue

            t_hat_x = tx / t_norm
            t_hat_y = ty / t_norm
            B = self.gamma * t_norm

            # signed angle from t_hat to d_hat
            theta = atan2(
                t_hat_x * d_hat_y - t_hat_y * d_hat_x,
                t_hat_x * d_hat_x + t_hat_y * d_hat_y,
            )

            # Shift the decay closer in by 2 * safety_radius so the
            # repulsion saturates at the safety-bubble surface instead of
            # at zero centre-to-centre distance.
            gap = max(dist - 2.0 * self.safety_radius, 0.0)
            f_v = -exp(-gap / B - (self.n_velocity * B * theta) ** 2)
            sign_theta = 1.0 if theta > 0 else (-1.0 if theta < 0 else 0.0)
            f_a = -sign_theta * exp(-gap / B - (self.n_angular * B * theta) ** 2)

            # left normal of t_hat
            t_perp_x = -t_hat_y
            t_perp_y = t_hat_x

            fx += f_v * t_hat_x + f_a * t_perp_x
            fy += f_v * t_hat_y + f_a * t_perp_y

        return [fx, fy]

    def obstacle_force(self) -> list:
        """Exponential repulsion summed over all nearby line obstacles.

        The upstream reference uses only the single nearest obstacle,
        which oscillates in symmetric environments (two parallel walls
        flip which one is "nearest" each step). We use the
        Helbing-Molnar (1995) summation form instead: every segment
        within ``5 * sigma_obstacle`` contributes an exponentially
        decayed push, so symmetric walls cancel and the agent walks the
        centreline. The integration is also clamped against overlap
        (``distance < 0`` would otherwise make ``exp(-distance/sigma)``
        explode).
        """
        x = self.state[0]
        y = self.state[1]
        r = self.state[4]

        if not self.line_obs_list:
            return [0.0, 0.0]

        cutoff = 5.0 * self.sigma_obstacle + r

        fx = 0.0
        fy = 0.0
        for seg in self.line_obs_list:
            cx, cy = self._closest_point_on_segment(x, y, seg)
            dx = x - cx
            dy = y - cy
            dist_center = sqrt(dx * dx + dy * dy)
            if dist_center > cutoff:
                continue
            if dist_center < 1e-9:
                # overlap — push along an arbitrary axis at full strength
                fx += 1.0
                continue
            # clamp at the surface so exp() can't blow up on penetration
            distance = max(dist_center - r, 0.0)
            amount = exp(-distance / self.sigma_obstacle)
            fx += amount * dx / dist_center
            fy += amount * dy / dist_center

        return [fx, fy]

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _closest_point_on_segment(x: float, y: float, seg: list) -> tuple[float, float]:
        """Closest point on the segment ``(x1,y1)->(x2,y2)`` to ``(x,y)``."""
        x1, y1, x2, y2 = seg
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx * dx + dy * dy
        if l2 < 1e-12:
            return x1, y1
        t = ((x - x1) * dx + (y - y1) * dy) / l2
        t = max(0.0, min(1.0, t))
        return x1 + t * dx, y1 + t * dy


class SocialForceModelBatch:
    """Vectorized Social Force Model for a crowd of agents.

    Evaluates the same desired, social and obstacle forces as
    :class:`social_force_model` for ``N`` agents at once from one state
    snapshot. Every agent therefore reacts to the same instant, so the
    result does not depend on the order in which agents are stepped, and
    the pairwise social term is array math over all ``N x (N + M)`` pairs
    instead of a Python loop.

    Agents can optionally be partitioned into *social groups* (people
    walking together). Members of a group receive the three extra forces
    of Moussaid et al. (2010), in the formulation used by pedsim_ros:

    * coherence -- pull toward the group's centre of mass, ramping in
      smoothly (``tanh``) once a member is further than ``(size - 1) / 2``
      metres from it;
    * repulsion -- push apart members whose centres come closer than
      ``group_repulsion_threshold``;
    * gaze -- brake along the velocity, ``f = -alpha * v``, when the other
      members' centre of mass lies outside a ``gaze_vision_angle`` cone
      around the walking direction; ``alpha`` is the missing head rotation
      in radians, so an agent slows down until it can see its group again.

    References:
        Moussaid, M., Perozo, N., Garnier, S., Helbing, D., Theraulaz, G.
        (2010), *The walking behaviour of pedestrian social groups and its
        impact on crowd dynamics.* PLoS ONE 5(4): e10047.

    Args:
        states (array-like): ``(N, 8)`` agent states
            ``[x, y, vx, vy, radius, vx_des, vy_des, theta]``; ``theta`` is
            carried for the caller and not used by the forces.
        neighbor_states (array-like): ``(M, 5)`` circular neighbours that
            are *not* part of the batch, ``[[x, y, vx, vy, radius], ...]``
            (robots driven by other behaviors, circular obstacles). They
            push batch agents but are not moved.
        line_obs_list (array-like): ``(K, 4)`` line obstacles
            ``[[x1, y1, x2, y2], ...]``.
        social_groups (list[list[int]] | None): Index lists into ``states``;
            each agent may appear in at most one group. ``None`` or ``[]``
            disables the group forces.
        vmax (float): Speed cap applied after the velocity update.
        step_time (float): Integration step ``dt``.
        relaxation_time (float): ``tau`` in the desired-force term.
        force_factor_desired (float): Weight on the desired force.
        force_factor_social (float): Weight on the social force.
        force_factor_obstacle (float): Weight on the obstacle force.
        sigma_obstacle (float): Decay length of the obstacle repulsion.
        lambda_importance (float): Weight of relative velocity in the
            interaction direction.
        gamma (float): Sets the interaction range ``B = gamma * ||t||``.
        n_angular (float): Angular sharpness ``n`` for the sideways force.
        n_velocity (float): Angular sharpness ``n'`` for the slowdown force.
        neighbor_range (float): Agents further apart than this ignore each
            other in the social force.
        safety_radius (float): Personal-space buffer subtracted from the
            agent-to-agent distance inside the social-force exponential.
        force_factor_coherence (float): Weight on the group coherence force.
        force_factor_group_repulsion (float): Weight on the intra-group
            repulsion force.
        group_repulsion_threshold (float | None): Centre-to-centre distance
            below which two group members repel each other. ``None`` uses
            the sum of their radii (``states[:, 4]``), i.e. members push
            apart as soon as their discs touch.
        force_factor_gaze (float): Weight on the group gaze force.
        gaze_vision_angle (float): Half-angle, in degrees, of the cone
            around the walking direction inside which the group's centre
            of mass is considered visible.
    """

    def __init__(
        self,
        states,
        neighbor_states=None,
        line_obs_list=None,
        social_groups: list[list[int]] | None = None,
        vmax: float = 1.5,
        step_time: float = 0.1,
        relaxation_time: float = 0.5,
        force_factor_desired: float = 1.0,
        force_factor_social: float = 2.1,
        force_factor_obstacle: float = 10.0,
        sigma_obstacle: float = 0.8,
        lambda_importance: float = 2.0,
        gamma: float = 0.35,
        n_angular: float = 2.0,
        n_velocity: float = 3.0,
        neighbor_range: float = 10.0,
        safety_radius: float = 0.0,
        force_factor_coherence: float = 2.0,
        force_factor_group_repulsion: float = 1.0,
        group_repulsion_threshold: float | None = None,
        force_factor_gaze: float = 3.0,
        gaze_vision_angle: float = 90.0,
    ) -> None:
        if relaxation_time <= 0.0:
            raise ValueError(f"relaxation_time must be > 0, got {relaxation_time}")
        if gamma <= 0.0:
            raise ValueError(f"gamma must be > 0, got {gamma}")
        if sigma_obstacle <= 0.0:
            raise ValueError(f"sigma_obstacle must be > 0, got {sigma_obstacle}")
        if group_repulsion_threshold is not None and group_repulsion_threshold < 0.0:
            raise ValueError(
                f"group_repulsion_threshold must be >= 0, got {group_repulsion_threshold}"
            )
        if not 0.0 < gaze_vision_angle <= 180.0:
            raise ValueError(
                f"gaze_vision_angle must be in (0, 180] degrees, got {gaze_vision_angle}"
            )

        self.vmax = vmax
        self.step_time = step_time
        self.relaxation_time = relaxation_time

        self.force_factor_desired = force_factor_desired
        self.force_factor_social = force_factor_social
        self.force_factor_obstacle = force_factor_obstacle

        self.sigma_obstacle = sigma_obstacle
        self.lambda_importance = lambda_importance
        self.gamma = gamma
        self.n_angular = n_angular
        self.n_velocity = n_velocity
        self.neighbor_range = neighbor_range
        self.safety_radius = safety_radius

        self.force_factor_coherence = force_factor_coherence
        self.force_factor_group_repulsion = force_factor_group_repulsion
        self.group_repulsion_threshold = group_repulsion_threshold
        self.force_factor_gaze = force_factor_gaze
        self.gaze_vision_angle = gaze_vision_angle

        self.social_groups: list[np.ndarray] = []
        self.update(
            states,
            neighbor_states,
            line_obs_list,
            social_groups if social_groups is not None else [],
        )

    def update(
        self,
        states,
        neighbor_states=None,
        line_obs_list=None,
        social_groups: list[list[int]] | None = None,
    ) -> None:
        """Refresh the per-step inputs without re-instantiating.

        Args:
            states: ``(N, 8)`` agent states.
            neighbor_states: ``(M, 5)`` external circular neighbours.
            line_obs_list: ``(K, 4)`` line obstacles.
            social_groups: New group partition, or ``None`` to keep the
                current one. Pass ``[]`` to switch the group forces off.
        """
        self.states = np.asarray(states, dtype=float).reshape(-1, 8)
        self.neighbor_states = np.asarray(
            neighbor_states if neighbor_states is not None else [], dtype=float
        ).reshape(-1, 5)
        self.line_obs_list = np.asarray(
            line_obs_list if line_obs_list is not None else [], dtype=float
        ).reshape(-1, 4)
        if social_groups is not None:
            self.social_groups = self._validate_groups(social_groups, self.n)
        else:
            self._validate_groups([g.tolist() for g in self.social_groups], self.n)

    @staticmethod
    def _validate_groups(social_groups, n: int) -> list[np.ndarray]:
        """Check the group partition and drop singleton groups.

        Raises:
            ValueError: If an index is out of range or an agent is listed
                more than once.
        """
        groups: list[np.ndarray] = []
        seen: set[int] = set()
        for group in social_groups:
            idx = [int(i) for i in group]
            for i in idx:
                if not 0 <= i < n:
                    raise ValueError(
                        f"social_groups index {i} is out of range for {n} agents"
                    )
                if i in seen:
                    raise ValueError(f"agent {i} appears in more than one social group")
                seen.add(i)
            if len(idx) >= 2:
                groups.append(np.asarray(idx, dtype=int))
        return groups

    # ------------------------------------------------------------------
    # Convenience views
    # ------------------------------------------------------------------

    @property
    def n(self) -> int:
        """Number of agents in the batch."""
        return self.states.shape[0]

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def cal_vel(self) -> np.ndarray:
        """Integrate one SFM step for every agent.

        Returns:
            np.ndarray: ``(N, 2)`` updated world-frame velocities, each
            clipped to ``vmax``.
        """
        acc = (
            self.force_factor_desired * self.desired_force()
            + self.force_factor_social * self.social_force()
            + self.force_factor_obstacle * self.obstacle_force()
        )
        if self.social_groups:
            acc = (
                acc
                + self.force_factor_coherence * self.coherence_force()
                + self.force_factor_group_repulsion * self.group_repulsive_force()
                + self.force_factor_gaze * self.gaze_force()
            )

        vel = self.states[:, 2:4] + self.step_time * acc
        speed = np.hypot(vel[:, 0], vel[:, 1])
        over = speed > self.vmax
        scale = np.where(over, self.vmax / np.where(over, speed, 1.0), 1.0)
        return vel * scale[:, None]

    # ------------------------------------------------------------------
    # Individual force terms (all return ``(N, 2)`` unweighted forces)
    # ------------------------------------------------------------------

    def desired_force(self) -> np.ndarray:
        """Relaxation toward the desired velocity ``states[:, 5:7]``."""
        return (self.states[:, 5:7] - self.states[:, 2:4]) / self.relaxation_time

    def social_force(self) -> np.ndarray:
        """Anisotropic neighbour repulsion (Moussaid-Helbing 2009).

        Each agent sums the contribution of every other batch agent and
        every external neighbour closer than ``neighbor_range``. Matches
        :meth:`social_force_model.social_force` term by term.
        """
        n = self.n
        if n == 0:
            return np.zeros((0, 2))

        pos = self.states[:, 0:2]
        vel = self.states[:, 2:4]
        others_pos = np.vstack((pos, self.neighbor_states[:, 0:2]))
        others_vel = np.vstack((vel, self.neighbor_states[:, 2:4]))

        d = others_pos[None, :, :] - pos[:, None, :]
        dist = np.hypot(d[..., 0], d[..., 1])
        valid = (dist >= 1e-6) & (dist < self.neighbor_range)
        valid[np.arange(n), np.arange(n)] = False
        if not valid.any():
            return np.zeros((n, 2))

        safe_dist = np.where(valid, dist, 1.0)
        d_hat = d / safe_dist[..., None]

        # velocity diff is self - other (upstream convention)
        dv = vel[:, None, :] - others_vel[None, :, :]

        # interaction vector  t = lambda * dv + d_hat
        t = self.lambda_importance * dv + d_hat
        t_norm = np.hypot(t[..., 0], t[..., 1])
        valid &= t_norm >= 1e-9
        safe_t = np.where(valid, t_norm, 1.0)
        t_hat = t / safe_t[..., None]
        b = self.gamma * safe_t

        # signed angle from t_hat to d_hat
        theta = np.arctan2(
            t_hat[..., 0] * d_hat[..., 1] - t_hat[..., 1] * d_hat[..., 0],
            t_hat[..., 0] * d_hat[..., 0] + t_hat[..., 1] * d_hat[..., 1],
        )

        gap = np.maximum(dist - 2.0 * self.safety_radius, 0.0)
        decay = -gap / b
        f_v = -np.exp(decay - (self.n_velocity * b * theta) ** 2)
        f_a = -np.sign(theta) * np.exp(decay - (self.n_angular * b * theta) ** 2)

        # f_v along t_hat plus f_a along the left normal of t_hat
        fx = np.where(valid, f_v * t_hat[..., 0] - f_a * t_hat[..., 1], 0.0)
        fy = np.where(valid, f_v * t_hat[..., 1] + f_a * t_hat[..., 0], 0.0)
        return np.stack((fx.sum(axis=1), fy.sum(axis=1)), axis=1)

    def obstacle_force(self) -> np.ndarray:
        """Exponential repulsion summed over all nearby line obstacles.

        Same rule as :meth:`social_force_model.obstacle_force`: every
        segment within ``5 * sigma_obstacle + radius`` of an agent's centre
        contributes, the decay is clamped at the body surface, and a
        segment passing exactly through the centre pushes along ``+x``.
        """
        n = self.n
        force = np.zeros((n, 2))
        if n == 0 or self.line_obs_list.shape[0] == 0:
            return force

        pos = self.states[:, 0:2]
        r = self.states[:, 4]
        seg = self.line_obs_list
        a = seg[:, 0:2]
        ab = seg[:, 2:4] - a
        l2 = np.einsum("kj,kj->k", ab, ab)
        degenerate = l2 < 1e-12

        ap = pos[:, None, :] - a[None, :, :]
        t = np.einsum("nkj,kj->nk", ap, ab) / np.where(degenerate, 1.0, l2)[None, :]
        t = np.clip(np.where(degenerate[None, :], 0.0, t), 0.0, 1.0)
        closest = a[None, :, :] + t[..., None] * ab[None, :, :]

        dvec = pos[:, None, :] - closest
        dist_center = np.hypot(dvec[..., 0], dvec[..., 1])
        within = dist_center <= 5.0 * self.sigma_obstacle + r[:, None]
        overlap = within & (dist_center < 1e-9)
        regular = within & ~overlap

        distance = np.maximum(dist_center - r[:, None], 0.0)
        amount = np.exp(-distance / self.sigma_obstacle)
        scale = np.where(regular, amount / np.where(regular, dist_center, 1.0), 0.0)

        force[:, 0] = (scale * dvec[..., 0]).sum(axis=1) + overlap.sum(axis=1)
        force[:, 1] = (scale * dvec[..., 1]).sum(axis=1)
        return force

    # ------------------------------------------------------------------
    # Social-group force terms (Moussaid et al. 2010)
    # ------------------------------------------------------------------

    def coherence_force(self) -> np.ndarray:
        """Pull each member toward its group's centre of mass.

        The pull is ``(com - p) * (tanh(d - d_max) + 1) / 2`` with
        ``d_max = (size - 1) / 2`` metres, the smooth variant of the
        paper's step rule used by pedsim_ros.
        """
        force = np.zeros((self.n, 2))
        pos = self.states[:, 0:2]
        for idx in self.social_groups:
            members = pos[idx]
            rel = members.mean(axis=0) - members
            dist = np.hypot(rel[:, 0], rel[:, 1])
            max_dist = (len(idx) - 1) / 2.0
            soft = (np.tanh(dist - max_dist) + 1.0) / 2.0
            force[idx] = rel * soft[:, None]
        return force

    def group_repulsive_force(self) -> np.ndarray:
        """Push apart group members that come too close.

        Each pair closer than ``group_repulsion_threshold`` (or the sum of
        the two radii when that is ``None``) contributes the raw offset
        ``p_self - p_other``.
        """
        force = np.zeros((self.n, 2))
        pos = self.states[:, 0:2]
        radius = self.states[:, 4]
        for idx in self.social_groups:
            members = pos[idx]
            diff = members[:, None, :] - members[None, :, :]
            dist = np.hypot(diff[..., 0], diff[..., 1])
            if self.group_repulsion_threshold is None:
                threshold = radius[idx][:, None] + radius[idx][None, :]
            else:
                threshold = self.group_repulsion_threshold
            close = dist < threshold
            np.fill_diagonal(close, False)
            force[idx] = (diff * close[..., None]).sum(axis=1)
        return force

    def gaze_force(self) -> np.ndarray:
        """Brake when the rest of the group falls out of view.

        With ``phi`` the angle between the walking direction ``v`` and the
        other members' centre of mass, the force is ``-(phi - phi_vis) * v``
        when ``phi`` exceeds ``gaze_vision_angle`` and zero otherwise. A
        stationary member has no walking direction and gets no gaze force.
        """
        force = np.zeros((self.n, 2))
        pos = self.states[:, 0:2]
        vel = self.states[:, 2:4]
        vision = np.radians(self.gaze_vision_angle)
        for idx in self.social_groups:
            m = len(idx)
            members = pos[idx]
            com_others = (m * members.mean(axis=0) - members) / (m - 1)
            rel = com_others - members
            v = vel[idx]
            speed = np.hypot(v[:, 0], v[:, 1])
            rel_dist = np.hypot(rel[:, 0], rel[:, 1])
            ok = (speed > 1e-9) & (rel_dist > 1e-9)
            cos_phi = np.einsum("ij,ij->i", v, rel) / np.where(
                ok, speed * rel_dist, 1.0
            )
            phi = np.arccos(np.clip(cos_phi, -1.0, 1.0))
            rotation = np.where(ok, np.maximum(phi - vision, 0.0), 0.0)
            force[idx] = -rotation[:, None] * v
        return force


# ----------------------------------------------------------------------
# Standalone sanity check (run: python -m irsim.lib.algorithm.social_force_model)
# ----------------------------------------------------------------------

if __name__ == "__main__":  # pragma: no cover - standalone sanity demo
    # Two agents approaching head-on along the x axis.
    # Agent A at (0,0) heading +x; Agent B at (4,0.05) heading -x.
    state_a = [0.0, 0.0, 1.0, 0.0, 0.3, 1.0, 0.0, 0.0]
    state_b = [4.0, 0.05, -1.0, 0.0, 0.3]

    sfm = social_force_model(
        state=state_a,
        neighbor_list=[state_b],
        line_obs_list=[[-5.0, 1.5, 5.0, 1.5], [-5.0, -1.5, 5.0, -1.5]],
        vmax=1.5,
        step_time=0.05,
    )

    for k in range(60):
        vx, vy = sfm.cal_vel()
        state_a[0] += sfm.step_time * vx
        state_a[1] += sfm.step_time * vy
        state_a[2] = vx
        state_a[3] = vy

        # toy update of B coming toward A
        state_b[0] += sfm.step_time * state_b[2]
        state_b[1] += sfm.step_time * state_b[3]

        sfm.update(state_a, [state_b], sfm.line_obs_list)

        if k % 10 == 0:
            print(
                f"t={k * sfm.step_time:4.2f}  "
                f"A=({state_a[0]:5.2f},{state_a[1]:5.2f})  "
                f"v=({vx:5.2f},{vy:5.2f})"
            )
