"""
Created on 15.02.2017

@author: kaiser

Module including common methods to calculate potential field vectors
"""

from geometry_msgs.msg import Vector3
import calc
import numpy as np


#  Potential field calculations based on Balch and Hybinette
# (doi:10.1109/ROBOT.2000.844042)
def calc_attractive_gradient(gradient, pose):
    """
    :param gradient: position of the goal
    :param pose: position SoMessage of the robot
    :return: attractive vector / norm vector
    """
    v = Vector3()

    # distance goal - agent
    tmp = calc.delta_vector(gradient.p, pose.p)

    # shortest distance considered (goal_radius of agent == size of agent)
    d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) - pose.goal_radius

    if d <= gradient.goal_radius:
        v = Vector3()
    elif gradient.goal_radius < d <= gradient.goal_radius + gradient.diffusion:
        # calculate magnitude of vector
        magnitude = (d - gradient.goal_radius) / gradient.diffusion
        v = calc.adjust_length(tmp, magnitude)
    elif d > gradient.goal_radius + gradient.diffusion:
        # calculate attraction vector
        v = calc.adjust_length(tmp, 1.0)

    return v


def calc_repulsive_gradient(gradient, pose):
    """
    :param gradient: position of the obstacle
    :param pose: position SoMessage of the robot
    :return: repulsive vector
    """
    v = Vector3()

    # distance goal - agent
    tmp = calc.delta_vector(pose.p, gradient.p)

    # shortest distance considered (goal_radius of agent == size of agent)
    d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) - pose.goal_radius

    if d <= gradient.goal_radius:  # infinitely large repulsion
        v = Vector3(np.inf, np.inf, np.inf)
        # calculate norm vector for direction
        tmp = calc.unit_vector3(tmp)

        # calculate repulsion vector / adjust sign/direction
        if tmp.x != 0.0:
            v.x *= tmp.x
        if tmp.y != 0.0:
            v.y *= tmp.y
        if tmp.z != 0.0:
            v.z *= tmp.z

    elif gradient.goal_radius < d <= gradient.diffusion + \
            gradient.goal_radius:
        # calculate magnitude of vector
        magnitude = (gradient.diffusion + gradient.goal_radius - d) / \
                    gradient.diffusion
        # calculate vector
        v = calc.adjust_length(tmp, magnitude)

    elif d > gradient.diffusion + gradient.goal_radius:
        v = Vector3()

    return v


#  Potential field calculations based on Ge and Cui
def calc_attractive_gradient_ge(gradient, pose):
    """
    calculate attractive gradient based on Ge & Cui - no normalization of
    vectors!
    normalized version same as _calc_attractive_gradient
    :param gradient: position of the goal
    :param pose: position SoMessage of the robot
    :return: attractive vector
    """
    v = Vector3()

    # distance goal - agent
    tmp = calc.delta_vector(gradient.p, pose.p)

    # shortest distance considered (goal_radius of agent == size of agent)
    d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) - pose.goal_radius

    if d <= gradient.goal_radius:
        v = Vector3()
    elif gradient.goal_radius < d <= gradient.goal_radius + gradient.diffusion:
        v = tmp
    elif d > gradient.goal_radius + gradient.diffusion:
        v = calc.adjust_length(tmp, (gradient.goal_radius+gradient.diffusion))

    return v


def calc_repulsive_gradient_ge(gradient, goal, pose):
    """
    :param gradient: position of the obstacle
    :param goal: attractive gradient to be followed
    :param pose: position SoMessage of the robot
    :return: repulsive vector
    distance of influence of obstacle = goal_radius + diffusion
    """
    v = Vector3()

    # distance obstacle - agent
    tmp = calc.delta_vector(pose.p, gradient.p)

    # shortest distance considered (goal_radius of agent == size of agent)
    d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) - pose.goal_radius

    if d <= gradient.goal_radius:
        v = Vector3(np.inf, np.inf, np.inf)
        # calculate norm vector for direction
        tmp = calc.unit_vector3(tmp)

        # calculate repulsion vector - adjust sign/direction
        if tmp.x != 0.0:
            v.x *= tmp.x
        if tmp.y != 0.0:
            v.y *= tmp.y
        if tmp.z != 0.0:
            v.z *= tmp.z
    elif gradient.goal_radius < d <= gradient.goal_radius + gradient.diffusion:
        # unit vector obstacle - agent
        tmp = calc.unit_vector3(tmp)
        # distance repulsive gradient - goal
        d_goal = calc.get_gradient_distance(pose.p, goal.p)
        # unit vector agent - goal
        ag = Vector3()
        ag.x = (goal.p.x - pose.p.x) / d_goal
        ag.y = (goal.p.y - pose.p.y) / d_goal
        ag.z = (goal.p.z - pose.p.z) / d_goal
        # closest distance to obstacle  - diffusion
        d_obs_diff = (1.0 / (d - gradient.goal_radius)) - \
                     (1.0 / gradient.diffusion)
        # parameters
        n = 1.0
        eta = 1.0
        # weighting rep1 and rep2
        f_rep1 = eta * d_obs_diff * ((d_goal ** n) /
                                     ((d - gradient.goal_radius) ** n))
        f_rep2 = eta * (n / 2.0) * np.square(d_obs_diff) * (d_goal ** (n - 1))
        v.x = f_rep1 * tmp.x + f_rep2 * ag.x
        v.y = f_rep1 * tmp.y + f_rep2 * ag.y
        v.z = f_rep1 * tmp.z + f_rep2 * ag.z

    elif d > gradient.goal_radius + gradient.diffusion:
        v = Vector3()

    return v
