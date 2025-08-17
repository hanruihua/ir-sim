"""
This file is the implementation of the Reciprocal Velocity Obstacle (RVO) algorithm for multi-robot collision avoidance.

Author: Ruihua Han

reference: https://github.com/MengGuo/RVO_Py_MAS
"""

from math import asin, atan2, cos, pi, sin

import numpy as np

from irsim.util.util import dist_hypot


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
    """

    def __init__(
        self,
        state: list,
        obs_state_list=None,
        vxmax=1.5,
        vymax=1.5,
        acce=0.5,
        factor=1.0,
    ):
        if obs_state_list is None:
            obs_state_list = []
        self.state = state
        self.obs_state_list = obs_state_list
        self.vxmax = vxmax
        self.vymax = vymax
        self.acce = acce
        self.factor = factor

    def update(self, state, obs_state_list):
        self.state = state
        self.obs_state_list = obs_state_list

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

        else:
            print("wrong method mode, pleas input vo, rvo or hrvo")

        vo_outside, vo_inside = self.vel_candidate(rvo_list)
        return self.vel_select(vo_outside, vo_inside)

    def config_rvo(self):
        rvo_list = []

        for obstacle in self.obs_state_list:
            rvo = self.config_rvo_mode(obstacle)
            rvo_list.append(rvo)

        return rvo_list

    def config_rvo_mode(self, obstacle):
        x = self.state[0]
        y = self.state[1]
        vx = self.state[2]
        vy = self.state[3]
        r = self.state[4]

        mode = "sta_circular" if vx == 0 and vy == 0 else "moving"

        if mode == "moving":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = obstacle[2]
            mvy = obstacle[3]
            mr = obstacle[4]

            rvo_apex = [(vx + mvx) / 2, (vy + mvy) / 2]

        elif mode == "sta_circular":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = 0
            mvy = 0
            mr = obstacle[2] + 0.2

            vo_apex = [mvx, mvy]
            rvo_apex = vo_apex  # vo
        else:
            print("wrong rvo mode")

        dis_mr = np.sqrt((my - y) ** 2 + (mx - x) ** 2)
        angle_mr = atan2(my - y, mx - x)

        if dis_mr < r + mr:
            dis_mr = r + mr

        ratio = (r + mr) / dis_mr

        if ratio > 1:
            ratio = 1
        if ratio < -1:
            ratio = -1

        half_angle = asin(ratio)
        line_left_ori = angle_mr + half_angle
        line_right_ori = angle_mr - half_angle

        line_left_vector = [cos(line_left_ori), sin(line_left_ori)]
        line_right_vector = [cos(line_right_ori), sin(line_right_ori)]

        # return [rvo_apex, line_left_ori, line_right_ori]

        return [rvo_apex, line_left_vector, line_right_vector]

    def config_hrvo(self):
        hrvo_list = []

        for obstacle in self.obs_state_list:
            # for circular: [x, y, radius]
            hrvo = self.config_hrvo_mode(obstacle)
            hrvo_list.append(hrvo)

        return hrvo_list

    def config_hrvo_mode(self, obstacle):
        x = self.state[0]
        y = self.state[1]
        vx = self.state[2]
        vy = self.state[3]
        r = self.state[4]

        mode = "sta_circular" if vx == 0 and vy == 0 else "moving"

        if mode == "moving":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = obstacle[2]
            mvy = obstacle[3]
            mr = obstacle[4]

        elif mode == "sta_circular":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = 0
            mvy = 0
            mr = obstacle[2] + 0.2

        else:
            print("wrong hrvo mode")

        rvo_apex = [(vx + mvx) / 2, (vy + mvy) / 2]
        vo_apex = [mvx, mvy]

        dis_mr = np.sqrt((my - y) ** 2 + (mx - x) ** 2)
        angle_mr = atan2(my - y, mx - x)

        if dis_mr < r + mr:
            dis_mr = r + mr

        ratio = (r + mr) / dis_mr

        half_angle = asin(ratio)
        line_left_ori = angle_mr + half_angle
        line_right_ori = angle_mr - half_angle

        line_left_vector = [cos(line_left_ori), sin(line_left_ori)]
        line_right_vector = [cos(line_right_ori), sin(line_right_ori)]

        if mode == "moving":
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

        if mode == "sta_circular":
            return [vo_apex, line_left_vector, line_right_vector]
        return None

    def config_vo(self):
        vo_list = []

        for obstacle in self.obs_state_list:
            vo = self.config_vo_mode(obstacle)
            vo_list.append(vo)

        return vo_list

    def config_vo_mode(self, obstacle):
        x = self.state[0]
        y = self.state[1]
        vx = self.state[2]
        vy = self.state[3]
        r = self.state[4]

        mode = "sta_circular" if vx == 0 and vy == 0 else "moving"

        if mode == "moving":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = obstacle[2]
            mvy = obstacle[3]
            mr = obstacle[4]

        elif mode == "sta_circular":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = 0
            mvy = 0
            mr = obstacle[2] + 0.2

        else:
            print("wrong obstacle mode")

        vo_apex = [mvx, mvy]
        dis_mr = np.sqrt((my - y) ** 2 + (mx - x) ** 2)
        angle_mr = atan2(my - y, mx - x)

        if dis_mr < r + mr:
            dis_mr = r + mr

        ratio = (r + mr) / dis_mr

        if ratio > 1:
            ratio = 1
        if ratio < -1:
            ratio = -1

        half_angle = asin(ratio)
        line_left_ori = angle_mr + half_angle
        line_right_ori = angle_mr - half_angle

        line_left_vector = [cos(line_left_ori), sin(line_left_ori)]
        line_right_vector = [cos(line_right_ori), sin(line_right_ori)]

        return [vo_apex, line_left_vector, line_right_vector]

    def vel_candidate(self, rvo_list):
        vo_outside = []
        vo_inside = []

        cur_vx = self.state[2]
        cur_vy = self.state[3]

        cur_vx_min = max((cur_vx - self.acce), -self.vxmax)
        cur_vx_max = min((cur_vx + self.acce), self.vxmax)

        cur_vy_min = max((cur_vy - self.acce), -self.vymax)
        cur_vy_max = min((cur_vy + self.acce), self.vymax)

        for new_vx in np.arange(cur_vx_min, cur_vx_max, 0.05):
            for new_vy in np.arange(cur_vy_min, cur_vy_max, 0.05):
                if self.vo_out(new_vx, new_vy, rvo_list):
                    vo_outside.append([new_vx, new_vy])

                else:
                    vo_inside.append([new_vx, new_vy])

        return vo_outside, vo_inside

    def vo_out(self, vx, vy, rvo_list):
        for rvo in rvo_list:
            rel_vx = vx - rvo[0][0]
            rel_vy = vy - rvo[0][1]

            # rel_radians = atan2(rel_vy, rel_vx)
            rel_vector = [rel_vx, rel_vy]

            if reciprocal_vel_obs.between_vector(rvo[1], rvo[2], rel_vector):
                return False

        return True

    def vel_select(self, vo_outside, vo_inside):
        vel_des = [self.state[5], self.state[6]]

        if len(vo_outside) != 0:
            return min(
                vo_outside, key=lambda v: dist_hypot(v[0], v[1], vel_des[0], vel_des[1])
            )

        return min(vo_inside, key=lambda v: self.penalty(v, vel_des, self.factor))

    def penalty(self, vel, vel_des, factor):
        tc_list = []

        for moving in self.obs_state_list:
            distance = dist_hypot(moving[0], moving[1], self.state[0], self.state[1])
            diff = distance**2 - (self.state[4] + moving[4]) ** 2

            if diff < 0:
                diff = 0

            dis_vel = np.sqrt(diff)
            vel_trans = [
                2 * vel[0] - self.state[2] - moving[2],
                2 * vel[1] - self.state[3] - moving[3],
            ]
            vel_trans_speed = np.sqrt(vel_trans[0] ** 2 + vel_trans[1] ** 2) + 1e-7

            tc = dis_vel / vel_trans_speed

            tc_list.append(tc)

        tc_min = min(tc_list)

        if tc_min == 0:
            tc_min = 0.0001

        return factor * (1 / tc_min) + dist_hypot(
            vel_des[0], vel_des[1], vel[0], vel[1]
        )

    # judge the direction by vector
    @staticmethod
    def between_vector(line_left_vector, line_right_vector, line_vector):
        return bool(
            reciprocal_vel_obs.cross_product(line_left_vector, line_vector) <= 0
            and reciprocal_vel_obs.cross_product(line_right_vector, line_vector) >= 0
        )

    @staticmethod
    def cross_product(vector1, vector2):
        return float(vector1[0] * vector2[1] - vector2[0] * vector1[1])
