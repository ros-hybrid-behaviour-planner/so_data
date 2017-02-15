"""
Created on 15.02.2017

@author: kaiser
"""

from geometry_msgs.msg import Vector3
import calc
import gradient
import numpy as np


class RepulsionFernandez(object):

    def __init__(self, buffer, frame=None):
        """

        :param buffer:
        :param maxvel:
        """
        self._buffer = buffer

        if not frame:
            self.frame = self._buffer.pose_frame
        else:
            self.frame = frame

        self._current_pos = None

    def move(self):
        """

        :return:
        """

        self._current_pos = self._buffer.get_own_pose()
        view = self._buffer.decision_list(self._buffer.pose_frame)

        mov = Vector3()

        if self._current_pos:
            repulsion_radius = self._current_pos.diffusion + \
                               self._current_pos.goal_radius

            for el in view:
                distance = calc.get_gradient_distance(el.p, self._current_pos.p) \
                           - el.goal_radius

                if distance <= self._buffer._view_distance:
                    if distance > 0:
                        diff = repulsion_radius - distance
                        mov.x += (self._current_pos.p.x - el.p.x) * \
                               diff / distance
                        mov.y += (self._current_pos.p.y - el.p.y) * \
                               diff / distance
                        mov.z += (self._current_pos.p.z - el.p.z) * \
                               diff / distance

                    elif distance <= 0:
                        # create random vector with length=
                        # repulsion radius
                        if distance == - el.goal_radius:
                            mov = calc.add_vectors(mov,
                                                   calc.random_vector(
                                                       repulsion_radius))
                        # use direction leading away if available
                        else:
                            mov = calc.add_vectors(mov, calc.adjust_length(
                                calc.delta_vector(self._current_pos.p, el.p),
                                repulsion_radius))

            if calc.vector_length(mov) > repulsion_radius:
                mov = calc.adjust_length(mov, repulsion_radius)

        return mov

    def get_pos(self):
        return self._current_pos


# Repulsion based on gradients
class RepulsionGradient(object):

    def __init__(self, buffer, frame=None):
        """

        :param buffer:
        :param maxvel:
        """
        self._buffer = buffer

        if not frame:
            self.frame = self._buffer.pose_frame
        else:
            self.frame = frame

        self._current_pos = None

    def move(self):
        self._current_pos = self._buffer.get_own_pose()
        view = self._buffer.decision_list(self._buffer.pose_frame)
        repulsion = Vector3()

        if self._current_pos:
            repulsion_radius = self._current_pos.diffusion + \
                               self._current_pos.goal_radius

            for el in view:
                grad = gradient.calc_repulsive_gradient(el, self._current_pos)

                if grad.x == np.inf or grad.x == -1 * np.inf:
                    dv = calc.delta_vector(self._current_pos.p, el.p)

                    if calc.vector_length(dv) == 0:
                        repulsion = calc.add_vectors(repulsion,
                                                     calc.random_vector(
                                                         repulsion_radius))
                    else:
                        repulsion = calc.add_vectors(repulsion,
                                                     calc.adjust_length(
                                                         dv, repulsion_radius
                                                     ))

                else:
                    repulsion = calc.add_vectors(repulsion, grad)

            if calc.vector_length(repulsion) > repulsion_radius:
                repulsion = calc.adjust_length(repulsion, repulsion_radius)

        return repulsion

    def get_pos(self):
        return self._current_pos
