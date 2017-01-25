"""
Created on 24.01.2017

flocking based on Programming Game AI by Example by Mat Buckland

@author: kaiser
"""

from __future__ import division  # ensures floating point divisions
from geometry_msgs.msg import Vector3
import numpy as np
import calc
import tf.transformations
from geometry_msgs.msg import Quaternion


def separation(agent, neighbors):
    """
    Calculates separation steering force between agent and neighbors
    :param agent: agent position and orientation
    :param neighbors: neighbor positions and orientations
    :return: movement vector for separation
    """
    sep = Vector3()

    for q in neighbors:
        diff = calc.delta_vector(agent.p, q.p)
        dist = calc.vector_length(diff)
        if dist > 0.0:
            diff = calc.unit_vector3(diff)
            sep.x += diff.x / dist
            sep.y += diff.y / dist
            sep.z += diff.z / dist
        else:  # two agents on top of each other: add random vector
            tmp = np.random.rand(1, 3)
            sep.x += tmp[0]
            sep.y += tmp[1]
            sep.z += tmp[2]

    return sep


def alignment(neighbors, orientation=[1, 0, 0, 1]):
    """
    calculates alignment steering force
    averaged angle of neighbors
    :param neighbors: list of neighbors specifying position and orientation
    :param orientation: specifies initial orientation of agent
    :return: movement vector
    """
    yaw = 0.0
    pitch = 0.0
    roll = 0.0
    result = Vector3()

    for q in neighbors:
        yaw += q.h[0]
        pitch += q.h[1]
        roll += q.h[2]

    if len(neighbors) > 0:
        yaw /= len(neighbors)
        pitch /= len(neighbors)
        roll /= len(neighbors)
        # yaw -= agent.h[0] #TODO check if valid in our case with twists etc .
        # --> no as angle between current orientation and returned vector is
        #  handed back

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        q = Quaternion(*quaternion)

        current_orientation = tf.transformations.quaternion_matrix(
            [q.x, q.y, q.z, q.w]).dot(np.array(orientation))

        result.x = current_orientation[0]
        result.y = current_orientation[1]
        result.z = current_orientation[2]

    return result


def cohesion(agent, neighbors):
    """
    calculates cohesion steering force
    average of neighbor positions
    :param agent: agent position and orientation
    :param neighbors: list of neighbor positions and orientations
    :return: vector steering towards average position of neighbors
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
