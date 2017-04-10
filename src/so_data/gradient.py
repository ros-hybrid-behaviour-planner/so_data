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
    d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) - pose.goal_radius \
        - gradient.goal_radius

    if d <= 0:
        v = Vector3()
    elif 0 < d <= gradient.diffusion:
        # calculate magnitude of vector
        magnitude = d / gradient.diffusion
        v = calc.adjust_length(tmp, magnitude)
    elif d > gradient.diffusion:
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
    d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) - pose.goal_radius \
        - gradient.goal_radius

    if d <= 0 or gradient.diffusion == np.inf:  # infinitely large repulsion
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

    elif 0 < d <= gradient.diffusion:
        # calculate magnitude of vector
        magnitude = (gradient.diffusion - d) / gradient.diffusion
        # calculate vector
        v = calc.adjust_length(tmp, magnitude)

    elif d > gradient.diffusion:
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
    d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) - pose.goal_radius \
        - gradient.goal_radius

    if d <= 0:
        v = Vector3()
    elif 0 < d <= gradient.diffusion:
        v = calc.adjust_length(tmp, d)
    elif d > gradient.diffusion:
        if gradient.diffusion == 0:
            v = calc.adjust_length(tmp, 1)
        else:
            v = calc.adjust_length(tmp, gradient.diffusion)

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
    d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) - pose.goal_radius \
        - gradient.goal_radius

    if d <= 0:
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
    elif 0 < d <= gradient.diffusion:
        # unit vector obstacle - agent
        tmp = calc.unit_vector3(tmp)
        # distance agent - goal
        d_goal = calc.get_gradient_distance(pose.p, goal.p) - \
                 pose.goal_radius - goal.goal_radius
        # unit vector agent - goal
        ag = calc.delta_vector(goal.p, pose.p)
        ag = calc.unit_vector3(ag)
        # closest distance to obstacle  - diffusion
        d_obs_diff = (1.0 / d) - (1.0 / gradient.diffusion)
        # parameters
        n = 1.0
        eta = 1.0
        # weighting rep1 and rep2
        f_rep1 = eta * d_obs_diff * ((d_goal ** n) / (d ** n))
        f_rep2 = eta * (n / 2.0) * np.square(d_obs_diff) * (d_goal ** (n - 1))
        v.x = f_rep1 * tmp.x + f_rep2 * ag.x
        v.y = f_rep1 * tmp.y + f_rep2 * ag.y
        v.z = f_rep1 * tmp.z + f_rep2 * ag.z

    elif d > gradient.diffusion:
        v = Vector3()

    return v
