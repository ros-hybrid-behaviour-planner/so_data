'''
Created on 01.12.2016

@author: kaiser
'''

import numpy as np
from so_data.msg import Vector


def get_gradient_distance(gradpos, pose):
    '''
    :param gradpos: pose of the gradient to be investigated (Vector)
    :param pose: pose of the robot (Pose)
    :return: euclidian distance robot to last received gradient
    '''
    return np.linalg.norm([(gradpos.x - pose.x), (gradpos.y - pose.y)])


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the directed angle in radians between vectors 'v1' and 'v2' - only working in 2D!::

            >>> angle_between((1, 0), (0, 1))
            1.5707963267948966
            >>> angle_between((0, 1), (1, 0))
            -1.5707963267948966
            >>> angle_between((1, 0), (1, 0))
            0.0
            >>> angle_between((1, 0), (-1, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    # det needs square matrix as input!
    angle = np.math.atan2(np.linalg.det([v1_u, v2_u]), np.dot(v1_u, v2_u))

    if np.isnan(angle):
        return 0.0

    return angle

# Potential field calculations
def calc_attractive_gradient(gradient, pose):
    '''
    :param gradient: position of the goal
    :param pose: position of the robot
    :return: attractive vector
    '''

    v = Vector()

    # distance goal - agent
    d = get_gradient_distance(gradient.p, pose)
    # angle between agent and goal
    angle = np.math.atan2((gradient.p.y - pose.y), (gradient.p.x - pose.x))

    if d < gradient.goal_radius:
        v.x = 0
        v.y = 0
    elif gradient.goal_radius <= d <= gradient.goal_radius + gradient.diffusion:
        v.x = (d - gradient.goal_radius) * np.cos(angle)
        v.y = (d - gradient.goal_radius) * np.sin(angle)
    elif d > gradient.goal_radius + gradient.diffusion:
        v.x = gradient.diffusion * np.cos(angle)
        v.y = gradient.diffusion * np.sin(angle)

    return v


def calc_repulsive_gradient(gradient, pose):
    '''
    :param gradient: position of the goal
    :param pose: position of the robot
    :return: repulsive vector
    '''
    v = Vector()

    # distance goal - agent
    d = get_gradient_distance(gradient.p, pose)
    # angle between agent and goal
    angle = np.math.atan2((gradient.p.y - pose.y), (gradient.p.x - pose.x))

    if d < gradient.goal_radius:
        v.x = -1 * np.sign(np.cos(angle)) * np.inf
        v.y = -1 * np.sign(np.sin(angle)) * np.inf
    elif gradient.goal_radius <= d <= gradient.goal_radius + gradient.diffusion:
        v.x = -1 * (gradient.goal_radius + gradient.diffusion - d) * np.cos(angle)
        v.y = -1 * (gradient.goal_radius + gradient.diffusion - d) * np.sin(angle)
    elif d > gradient.goal_radius + gradient.diffusion:
        v.x = 0
        v.y = 0

    return v