"""
Created on 01.12.2016

@author: hrabia, kaiser

Module offers various methods for vector calculations

"""

import numpy as np
from geometry_msgs.msg import Vector3
import random


def unit_vector(vector):
    """
    Returns the unit vector of the vector.
    """
    return vector/np.linalg.norm(vector)


def unit_vector3(vector):
    """
    Returns the unit vector of a vector3
    """
    d = vector_length(vector)
    if d > 0:
        vector.x /= d
        vector.y /= d
        vector.z /= d

    return vector


def angle_between(v1, v2):
    """ Returns the directed angle in radians between vectors 'v1' and 'v2'
    - only working in 2D!::

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
        if (v1_u == v2_u).all:
            return 0.0
        else:
            return np.pi

    return angle


def get_gradient_distance(gradpos, pose):
    """
    :param gradpos: pose of the gradient to be investigated (Vector)
    :param pose: pose of the robot (Pose)
    :return: euclidian distance robot to last received gradient
    """
    return np.linalg.norm([(gradpos.x - pose.x), (gradpos.y - pose.y),
                           (gradpos.z - pose.z)])


def vector_length(vector):
    """
    :param vector: Vector3
    :return: vector length
    """
    return np.linalg.norm([vector.x, vector.y, vector.z])


def delta_vector(q1, q2):
    """
    :param q1: Vector3
    :param q2: Vector3
    :return: Vector3 which is q1 - q2
    """
    d = Vector3()
    d.x = q1.x - q2.x
    d.y = q1.y - q2.y
    d.z = q1.z - q2.z

    return d


def add_vectors(q1, q2):
    """
    :param q1: Vector3
    :param q2: Vector3
    :return: Vector3 which is q1 + q2
    """
    d = Vector3()
    d.x = q1.x + q2.x
    d.y = q1.y + q2.y
    d.z = q1.z + q2.z

    return d


def adjust_length(q, length):
    """
    :param q: Vector3
    :param length: desired vector length
    :return: Vector3 with lenght = length
    """
    result = unit_vector3(q)
    result.x *= length
    result.y *= length
    result.z *= length

    return result


def random_vector(length):
    """
    :return: random vector with length = length
    """
    tmp = unit_vector3(Vector3(random.uniform(-1, 1), random.uniform(-1, 1),
                  random.uniform(-1, 1)))
    tmp.x *= length
    tmp.y *= length
    tmp.z *= length

    return tmp