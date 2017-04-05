"""
Created on 15.02.2017

@author: kaiser

Module including two repulsion mechanisms
* repulsion based on formulas by Fernandez-Marquez
* repulsion based on repulsive gradient calculations
"""

import calc
import gradient
import numpy as np
from geometry_msgs.msg import Vector3
from patterns import MovementPattern


class RepulsionFernandez(MovementPattern):
    """
    Repulsion calculation based on formulas presented by Fernandez-Marquez
    in "Description and composition of bio-inspired design patterns"
    """
    def __init__(self, buffer, frames=None, static=False, moving=True):
        """
        :param buffer: soBuffer returning gradient data
        :param frame: agent frame ID
        :param static: consider static gradients
        :param moving: consider moving gradients
        """
        # set standard agent frame if no frame is specified
        if not frames:
            frames = [buffer.pose_frame]

        super(RepulsionFernandez, self).__init__(buffer, frames, static=static,
                                                 moving=moving)

    def move(self):
        """
        calculates repulsion vector based on Fernandez-Marquez
        :return: repulsion movement vector
        """

        pose = self._buffer.get_own_pose()
        view = self._buffer.agent_list(self.frames)

        mov = None

        if pose:
            repulsion_radius = pose.diffusion + pose.goal_radius

            for el in view:
                distance = calc.get_gradient_distance(el.p, pose.p) \
                           - el.goal_radius

                if distance <= self._buffer.view_distance:
                    if not mov:
                        mov = Vector3()

                    if distance > 0:
                        diff = repulsion_radius - distance
                        mov.x += (pose.p.x - el.p.x) * diff / distance
                        mov.y += (pose.p.y - el.p.y) * diff / distance
                        mov.z += (pose.p.z - el.p.z) * diff / distance

                    elif distance <= 0:
                        # create random vector with length=repulsion radius
                        if distance == - el.goal_radius:
                            mov = calc.add_vectors(mov, calc.random_vector(
                                repulsion_radius))
                        # use direction leading away if available
                        else:
                            mov = calc.add_vectors(mov, calc.adjust_length(
                                calc.delta_vector(pose.p, el.p),
                                repulsion_radius))

            if not mov:
                return None

            if calc.vector_length(mov) > repulsion_radius:
                mov = calc.adjust_length(mov, repulsion_radius)

        return mov


class RepulsionGradient(MovementPattern):
    """
    Repulsion calculation based on repulsive gradients presented by
    Balch & Hybinette (see gradient.py)
    """

    def __init__(self, buffer, frames=None):
        """
        :param buffer: soBuffer returning gradient data
        :param frames: agent frame ID; if no
        :param static: consider static gradients
        :param moving: consider moving gradients:
        """

        if not frames:
            frames = [buffer.pose_frame]

        super(RepulsionGradient, self).__init__(buffer, frames)

    def move(self):
        """
        calculates repulsion vector based on gradients
        :return: repulsion movement vector
        """
        pose = self._buffer.get_own_pose()
        view = self._buffer.agent_list(self.frames)
        repulsion = None

        if pose:
            repulsion_radius = pose.diffusion + pose.goal_radius

            for el in view:
                grad = gradient.calc_repulsive_gradient(el, pose)

                if abs(grad.x) == np.inf or abs(grad.y) == np.inf or \
                                abs(grad.z) == np.inf:
                    dv = calc.delta_vector(pose.p, el.p)

                    if not repulsion:
                        repulsion = Vector3()

                    if calc.vector_length(dv) == 0:
                        repulsion = calc.add_vectors(repulsion,
                                                     calc.random_vector(
                                                         repulsion_radius))
                    else:
                        repulsion = calc.add_vectors(repulsion,
                                                     calc.adjust_length(
                                                         dv, repulsion_radius))

                else:
                    if not repulsion:
                        repulsion = grad
                    else:
                        repulsion = calc.add_vectors(repulsion, grad)

            if not repulsion:
                return None

            if calc.vector_length(repulsion) > repulsion_radius:
                repulsion = calc.adjust_length(repulsion, repulsion_radius)

        return repulsion
