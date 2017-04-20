"""
Created on 01.12.2016

@author: kaiser, hrabia

Module offers various methods for vector calculations

"""

import numpy as np
from geometry_msgs.msg import Vector3
import random


def unit_vector(vector):
    """
    determines unit vector
    :param vector: np.array to be transformed
    :return: unit vector
    """

    if np.linalg.norm(vector) == 0.0:
        return [0, 0]
    return vector/np.linalg.norm(vector)


def unit_vector3(vector):
    """
    determines unit vector
    :param vector: Vector3 to be transformed
    :return: unit vector
    """
    d = vector_length(vector)
    if d > 0:
        vector.x /= d
        vector.y /= d
        vector.z /= d

    return vector


def angle_between(v1, v2):
    """ Returns the directed angle in radians between vectors 'v1' and 'v2'
    (related to two axis):

            >>> angle_between((1, 0), (0, 1))
            1.5707963267948966
            >>> angle_between((0, 1), (1, 0))
            -1.5707963267948966
            >>> angle_between((1, 0), (1, 0))
            0.0
            >>> angle_between((1, 0), (-1, 0))
            3.141592653589793
    :param v1: vector as np.array
    :param v2: vector as np.array
    :return: angle in radians
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
    determines distance between two points
    :param gradpos: pose of the gradient to be investigated (Vector)
    :param pose: pose of the robot (Pose)
    :return: euclidian distance robot to last received gradient
    """
    return np.linalg.norm([(gradpos.x - pose.x), (gradpos.y - pose.y),
                           (gradpos.z - pose.z)])


def vector_length(vector):
    """
    determines vector length
    :param vector: Vector3
    :return: vector length
    """
    return np.linalg.norm([vector.x, vector.y, vector.z])


def delta_vector(q1, q2):
    """
    calculates vector difference
    :param q1: Vector3
    :param q2: Vector3
    :return: q1 - q2
    """
    d = Vector3()
    d.x = q1.x - q2.x
    d.y = q1.y - q2.y
    d.z = q1.z - q2.z

    return d


def add_vectors(q1, q2):
    """
    calculates vector sum
    :param q1: Vector3
    :param q2: Vector3
    :return: q1 + q2
    """
    d = Vector3()
    d.x = q1.x + q2.x
    d.y = q1.y + q2.y
    d.z = q1.z + q2.z

    return d


def adjust_length(q, length):
    """
    adjusts vector length
    :param q: Vector3
    :param length: desired vector length
    :return: Vector3 with length = length
    """
    result = unit_vector3(q)
    result.x *= length
    result.y *= length
    result.z *= length

    return result


def random_vector(length):
    """
    creates random vector with specified length
    :param length: desired vector length
    :return: random vector with length = length
    """
    tmp = unit_vector3(Vector3(random.uniform(-1, 1), random.uniform(-1, 1),
                  random.uniform(-1, 1)))
    tmp.x *= length
    tmp.y *= length
    tmp.z *= length

    return tmp