"""Compiled kernel for :mod:`irsim.lib.algorithm.lidar_batch` (optional).

Requires `numba <https://numba.pydata.org>`_ (``pip install ir-sim[fast]``).
The kernel evaluates the same beam/segment and beam/polygon formulas as the
NumPy caster in plain loops, so its ranges agree with the per-sensor scan to
floating-point precision, and it culls (sensor, primitive) pairs by range
before touching any beam. Compiled code is cached on disk after the first
call (``cache=True``).
"""

from __future__ import annotations

import math

try:
    from numba import njit
except ImportError:  # pragma: no cover - optional dependency
    njit = None

AVAILABLE = njit is not None

if AVAILABLE:

    @njit(cache=True, nogil=True)
    def _edge_hit(ox, oy, dx, dy, ax, ay, bx, by, eps, maxr, round_off):
        """Return the ray/segment distance (``inf`` on a miss) and ambiguity flag."""
        vx = bx - ax
        vy = by - ay
        aox = ax - ox
        aoy = ay - oy
        denom = dx * vy - dy * vx
        num_t = aox * vy - aoy * vx
        tol = round_off * max(1.0, maxr)
        ambiguous = abs(denom) <= tol and abs(num_t) <= eps + tol
        if denom != 0.0:
            t = num_t / denom
            u = (aox * dy - aoy * dx) / denom
            nearby = -tol <= t <= maxr + tol and -tol <= u <= 1 + tol
            boundary = (
                u <= tol or u >= 1 - tol or abs(t - maxr) <= tol or abs(t - eps) <= tol
            )
            ambiguous = ambiguous or (nearby and boundary)
            if t > eps and t <= maxr and u >= 0.0 and u <= 1.0:
                return t, ambiguous
            return math.inf, ambiguous
        if abs(num_t) <= eps:
            da = aox * dx + aoy * dy
            db = (bx - ox) * dx + (by - oy) * dy
            lo = min(da, db)
            hi = max(da, db)
            if hi > eps and lo <= maxr:
                hit = lo if lo > eps else min(hi, maxr)
                if hit <= maxr:
                    return hit, ambiguous
        return math.inf, ambiguous

    @njit(cache=True, nogil=True)
    def cast_kernel(
        origins,
        D,
        ranges,
        owner,
        seg_start,
        seg_end,
        seg_box,
        seg_owner,
        c_pos,
        c_theta,
        c_lc,
        c_radius,
        c_owner,
        c_local,
        c_angles,
        analytic,
        eps,
        round_off,
        best,
        best_obj,
        uncertain,
    ):
        """Nearest hit of every beam of every sensor.

        Args:
            origins: ``(S, 2)`` sensor origins; ``D``: ``(S, N, 2)`` unit beam
                directions; ``ranges``: ``(S,)``; ``owner``: ``(S,)`` object
                index of each sensor's body (excluded from its own scan).
            seg_start, seg_end: ``(E, 2)`` boundary segments; ``seg_box``:
                ``(E, 4)`` ``[min_x, max_x, min_y, max_y]``; ``seg_owner``: ``(E,)``.
            c_pos: ``(C, 2)`` body positions of ``circle`` objects; ``c_theta``:
                ``(C,)`` headings; ``c_lc``: ``(C, 2)`` body-frame polygon
                centres; ``c_radius``: ``(C,)`` circumscribed radii; ``c_owner``:
                ``(C,)``; ``c_local``: ``(C, M, 2)`` body-frame vertices sorted
                by angle; ``c_angles``: ``(C, M)`` those angles.
            analytic: Treat circles as true circles (approximate).
            eps: Origin epsilon of the per-sensor caster.
            round_off: Relative tolerance selecting reference fallback only.
            best, best_obj: ``(S, N)`` outputs, initialised to ``inf`` / ``-1``.
            uncertain: ``(S,)`` flags for scans requiring reference fallback.
        """
        S = origins.shape[0]
        N = D.shape[1]
        n_seg = seg_start.shape[0]
        C = c_pos.shape[0]
        M = c_local.shape[1]
        two_pi = 2.0 * math.pi
        for s in range(S):
            ox = origins[s, 0]
            oy = origins[s, 1]
            r = ranges[s]
            own = owner[s]
            for j in range(n_seg):
                if seg_owner[j] == own:
                    continue
                if (
                    seg_box[j, 0] > ox + r
                    or seg_box[j, 1] < ox - r
                    or seg_box[j, 2] > oy + r
                    or seg_box[j, 3] < oy - r
                ):
                    continue
                ax = seg_start[j, 0]
                ay = seg_start[j, 1]
                bx = seg_end[j, 0]
                by = seg_end[j, 1]
                for n in range(N):
                    t, ambiguous = _edge_hit(
                        ox,
                        oy,
                        D[s, n, 0],
                        D[s, n, 1],
                        ax,
                        ay,
                        bx,
                        by,
                        eps,
                        r,
                        round_off,
                    )
                    tied = math.isfinite(t) and abs(t - best[s, n]) <= round_off * max(
                        1.0, r
                    )
                    uncertain[s] = uncertain[s] or ambiguous or tied
                    if t < best[s, n]:
                        best[s, n] = t
                        best_obj[s, n] = seg_owner[j]
            for c in range(C):
                if c_owner[c] == own:
                    continue
                ct = math.cos(c_theta[c])
                st = math.sin(c_theta[c])
                px = c_pos[c, 0]
                py = c_pos[c, 1]
                cx = px + (ct * c_lc[c, 0] - st * c_lc[c, 1])
                cy = py + (st * c_lc[c, 0] + ct * c_lc[c, 1])
                R = c_radius[c]
                ddx = cx - ox
                ddy = cy - oy
                reach = r + R + round_off * max(1.0, r + R)
                if ddx * ddx + ddy * ddy > reach * reach:
                    continue
                ocx = ox - cx
                ocy = oy - cy
                cc = ocx * ocx + ocy * ocy - R * R
                for n in range(N):
                    dx = D[s, n, 0]
                    dy = D[s, n, 1]
                    b = ocx * dx + ocy * dy
                    disc = b * b - cc
                    if not analytic and abs(disc) <= round_off * max(
                        1.0, b * b + abs(cc)
                    ):
                        uncertain[s] = True
                    if disc < 0.0:
                        continue
                    sq = math.sqrt(disc)
                    t_in = -b - sq
                    t_out = -b + sq
                    if not analytic and abs(t_in - r) <= round_off * max(1.0, r):
                        uncertain[s] = True
                    if t_out <= eps or t_in > r:
                        continue
                    if analytic:
                        t = t_in if t_in > eps else t_out
                        if t > eps and t <= r and t < best[s, n]:
                            best[s, n] = t
                            best_obj[s, n] = c_owner[c]
                        continue
                    if t_in <= eps:
                        e_first = 0
                        e_count = M
                    else:
                        qx = ox + t_in * dx
                        qy = oy + t_in * dy
                        phi = math.atan2(qy - cy, qx - cx) - c_theta[c]
                        phi = (phi + math.pi) % two_pi - math.pi
                        lo = 0
                        hi = M
                        while lo < hi:
                            mid = (lo + hi) // 2
                            if c_angles[c, mid] <= phi:
                                lo = mid + 1
                            else:
                                hi = mid
                        e_first = lo - 2
                        e_count = 3
                    for k in range(e_count):
                        e = (e_first + k) % M
                        f = (e + 1) % M
                        ax = px + (c_local[c, e, 0] * ct - c_local[c, e, 1] * st)
                        ay = py + (c_local[c, e, 0] * st + c_local[c, e, 1] * ct)
                        bx = px + (c_local[c, f, 0] * ct - c_local[c, f, 1] * st)
                        by = py + (c_local[c, f, 0] * st + c_local[c, f, 1] * ct)
                        t, ambiguous = _edge_hit(
                            ox, oy, dx, dy, ax, ay, bx, by, eps, r, round_off
                        )
                        tied = math.isfinite(t) and abs(
                            t - best[s, n]
                        ) <= round_off * max(1.0, r)
                        uncertain[s] = uncertain[s] or ambiguous or tied
                        if t < best[s, n]:
                            best[s, n] = t
                            best_obj[s, n] = c_owner[c]

else:  # pragma: no cover - optional dependency
    cast_kernel = None
