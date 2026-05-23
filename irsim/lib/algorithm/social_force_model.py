"""
Social Force Model (SFM) for pedestrian / mobile-agent navigation.

Implements the anisotropic, velocity-aware variant of Moussaid, Helbing
et al. (2009), with obstacle repulsion from the closest point of the
nearest line obstacle (Helbing & Molnar 1995 form).

Each agent integrates Newton-like dynamics ``v += dt * a`` where ``a``
is the weighted sum of three forces:

* desired force  -- relaxation of velocity toward ``v0 * e_goal``;
* social force   -- anisotropic repulsion from each neighbor;
* obstacle force -- exponential repulsion summed over nearby line
  obstacles (Helbing-Molnar 1995 sum form; libpedsim uses only the
  single nearest obstacle, which oscillates between symmetric walls).

Upstream reference (this file is a faithful Python port):

    pedsim_ros -- ROS pedestrian simulator by the Social Robotics Lab,
    University of Freiburg.

        https://github.com/srl-freiburg/pedsim_ros

    The core social-force code lives in the vendored ``libpedsim``
    (``3rdparty/libpedsim/src/ped_agent.cpp``) by Christian Gloor; the
    ROS wrapper ``pedsim_simulator/`` layers optional extras
    (RandomForce, AlongWallForce, group coherence/gaze/repulsion).
    Only the three core forces from ``libpedsim`` are ported here.

    Local checkout used as the reference implementation:
        /Users/han/tech/pedsim_ros/3rdparty/libpedsim/src/ped_agent.cpp

Algorithmic references:

    Helbing, D. & Molnar, P. (1995), *Social force model for pedestrian
    dynamics.* Phys. Rev. E 51:4282.

    Moussaid, M., Helbing, D., Garnier, S., Johansson, A., Combe, M.,
    Theraulaz, G. (2009), *Experimental study of the behavioural
    mechanisms underlying self-organization in human crowds.*
    Proc. R. Soc. B 276:2755.
"""

from math import atan2, exp, sqrt


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
            ``0`` reproduces libpedsim (points). ``> 0`` shifts the
            decay closer-in so the repulsion saturates at
            ``2 * safety_radius`` of centre-to-centre clearance,
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

        Mirrors ``Ped::Twaypoint::getForce`` in libpedsim
        (``3rdparty/libpedsim/src/ped_waypoint.cpp`` in
        https://github.com/srl-freiburg/pedsim_ros). The desired
        velocity is supplied directly via ``state[5:7]``.
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
        their contribution. The implementation follows
        ``Ped::Tagent::socialForce()`` line-for-line
        (``3rdparty/libpedsim/src/ped_agent.cpp`` in
        https://github.com/srl-freiburg/pedsim_ros).
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

            # velocity diff is self - other (libpedsim convention)
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

        The libpedsim/pedsim_ros reference uses only the single nearest
        obstacle, which oscillates in symmetric environments (two
        parallel walls flip which one is "nearest" each step). We use
        the Helbing-Molnar (1995) summation form instead: every segment
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


# ----------------------------------------------------------------------
# Standalone sanity check (run: python -m irsim.lib.algorithm.social_force_model)
# ----------------------------------------------------------------------

if __name__ == "__main__":
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
