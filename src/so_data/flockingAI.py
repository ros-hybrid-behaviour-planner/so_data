"""
Created on 24.01.2017

flocking based on Reynolds Description
www.red3d.com/cwr/steer/gdc99/

@author: kaiser
"""

from __future__ import division  # ensures floating point divisions
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
import calc
import tf.transformations


def separation(agent, neighbors):
    """
    Calculates separation steering force between agent and neighbors
    :param agent: agent position and orientation
    :param neighbors: neighbor positions and orientations
    :return: normalized movement vector for separation
    """
    sep = Vector3()

    if len(neighbors) > 0:
        # 1/r weighting
        weight = 1.0 / 1.0
        for q in neighbors:
            diff = calc.delta_vector(agent.p, q.p)
            dist = calc.vector_length(diff)
            if dist > 0.0:
                diff = calc.unit_vector3(diff)
                sep.x += weight * (diff.x / dist)
                sep.y += weight * (diff.y / dist)
                sep.z += weight * (diff.z / dist)
            else:  # two agents on top of each other: add random vector
                tmp = np.random.rand(1, 3)[0]
                dist = np.linalg.norm(tmp)
                sep.x += weight * (tmp[0] / dist)
                sep.y += weight * (tmp[1] / dist)
                sep.z += weight * (tmp[2] / dist)

    return sep


def alignment(agent, neighbors):
    """
    calculates alignment steering force based on direction vectors
    :param agent: agent position and orientation (np.array, [Vector, 1])
    :param neighbors: list of neighbors specifying position and orientation
    :return: normalized movement vector
    """
    result = Vector3()
    tmp = np.zeros(4)

    if len(neighbors) > 0:
        # sum of direction vectors
        for q in neighbors:
            tmp += q.h
        # average
        tmp /= len(neighbors)

        # diff agent direction and average direction
        steering = tmp - agent.h

        # resulting vector is
        result.x = steering[0]
        result.y = steering[1]
        result.z = steering[2]

    return result


def cohesion(agent, neighbors):
    """
    calculates cohesion steering force
    average of neighbor positions
    :param agent: agent position and orientation
    :param neighbors: list of neighbor positions and orientations
    :return: norm vector steering towards average position of neighbors
    """

    coh = Vector3()

    for q in neighbors:
        coh = calc.add_vectors(coh, q.p)

    if len(neighbors) > 0:
        coh.x /= len(neighbors)
        coh.y /= len(neighbors)
        coh.z /= len(neighbors)

        coh = calc.delta_vector(coh, agent.p)

    return coh
