"""
Created on 24.01.2017
@author: kaiser

Module contains flocking methods based on Reynolds Description
www.red3d.com/cwr/steer/gdc99/
and flocking mechanism
"""

from __future__ import division
import numpy as np
import calc
import tf.transformations
from geometry_msgs.msg import Vector3, Quaternion
from patterns import MovementPattern


def separation(agent, neighbors, r=1.0):
    """
    Calculates separation steering force between agent and neighbors
    :param agent: agent position and orientation
    :param neighbors: neighbor positions and orientations
    :param r: weighting parameter (1/r)
    :return: normalized movement vector for separation
    """
    sep = Vector3()

    if len(neighbors) > 0:
        # 1/r weighting
        weight = 1.0 / r
        for q in neighbors:
            diff = calc.delta_vector(agent.p, q.p)
            # shortest distance between two robots
            dist = calc.vector_length(diff) - agent.goal_radius - q.goal_radius
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
    :param agent: agent soMessage
    :param neighbors: list of neighbors (soMessage)
    :return: movement vector
    """
    result = Vector3()
    tmp = np.zeros(4)

    if len(neighbors) > 0:
        # sum of direction vectors
        for n in neighbors:
            # add current heading vector
            tmp += tf.transformations.quaternion_matrix(
                [n.q.x, n.q.y, n.q.z, n.q.w]).dot(
                np.array([n.direction.x, n.direction.y, n.direction.z, 1]))

        # average
        tmp /= len(neighbors)

        # diff agent direction and average direction
        steering = tmp - tf.transformations.quaternion_matrix(
                [agent.q.x, agent.q.y, agent.q.z, agent.q.w]).dot(
                np.array([agent.direction.x, agent.direction.y,
                          agent.direction.z, 1]))

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


class FlockingRey(MovementPattern):
    """
    class containing flocking mechanism based on Reynold's formulas
    """
    def __init__(self, buffer, frame=None, moving=True, static=False,
                 maxvel=1.0):
        """
        :param buffer: soBuffer returning gradient data
        :param frame: agent frame ID
        :param moving: consider moving gradients
        :param static: consider static gradients
        :param maxvel: maximum flocking velocity
        """
        # set standard agent frame if no frame is specified
        if not frame:
            self.frame = buffer.pose_frame
        else:
            self.frame = frame

        super(FlockingRey, self).__init__(buffer, [self.frame], static=static,
                                          moving=moving, maxvel=maxvel)

    def move(self):
        """
        calculates flocking vector based on Reynolds
        :return: movement Vector (Vector3)
        """

        pose = self._buffer.get_own_pose()
        view = self._buffer.agent_list([self.frame])

        mov = None

        if pose:
            mov = separation(pose, view)
            mov = calc.add_vectors(mov, cohesion(pose, view))
            mov = calc.add_vectors(mov, alignment(pose, view))

        if mov and calc.vector_length(mov) > self.max_velocity:
            mov = calc.adjust_length(mov, self.max_velocity)

        return mov
