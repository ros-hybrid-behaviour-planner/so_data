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
    def __init__(self, buffer, frame=None, static=False, moving=True):
        """
        :param buffer: soBuffer returning gradient data
        :param frame: agent frame ID; if no
        :param static: consider static gradients
        :param moving: consider moving gradients
        """
        # set standard agent frame if no frame is specified
        if not frame:
            self.frame = buffer.pose_frame
        else:
            self.frame = frame

        super(RepulsionFernandez, self).__init__(buffer, self.frame, static,
                                                 moving, False)

    def move(self):
        """
        calculates repulsion vector based on Fernandez-Marquez
        :return: repulsion vector
        """

        pose = self._buffer.get_own_pose()
        view = self._buffer.agent_list(self.frame, self.static, self.moving)

        mov = Vector3()

        if pose:
            repulsion_radius = pose.diffusion + pose.goal_radius

            for el in view:
                distance = calc.get_gradient_distance(el.p, pose.p) \
                           - el.goal_radius

                if distance <= self._buffer.view_distance:
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

            if calc.vector_length(mov) > repulsion_radius:
                mov = calc.adjust_length(mov, repulsion_radius)

        return mov


class RepulsionGradient(MovementPattern):
    """
    Repulsion calculation based on repulsive gradients presented by
    Balch & Hybinette (see gradient.py)
    """

    def __init__(self, buffer, frame=None, static=False, moving=True):
        """
        :param buffer: soBuffer returning gradient data
        :param frame: agent frame ID; if no
        :param static: consider static gradients
        :param moving: consider moving gradients:
        """

        if not frame:
            self.frame = buffer.pose_frame
        else:
            self.frame = frame

        super(RepulsionGradient, self).__init__(buffer, self.frame, static,
                                                moving, False)

    def move(self):
        """
        calculates repulsion vector based on gradients
        :return: repulsion movement vector
        """
        pose = self._buffer.get_own_pose()
        view = self._buffer.agent_list(self.frame, self.static, self.moving)
        repulsion = Vector3()

        if pose:
            repulsion_radius = pose.diffusion + pose.goal_radius

            for el in view:
                grad = gradient.calc_repulsive_gradient(el, pose)

                if grad.x == np.inf or grad.x == -1 * np.inf:
                    dv = calc.delta_vector(pose.p, el.p)

                    if calc.vector_length(dv) == 0:
                        repulsion = calc.add_vectors(repulsion,
                                                     calc.random_vector(
                                                         repulsion_radius))
                    else:
                        repulsion = calc.add_vectors(repulsion,
                                                     calc.adjust_length(
                                                         dv, repulsion_radius))

                else:
                    repulsion = calc.add_vectors(repulsion, grad)

            if calc.vector_length(repulsion) > repulsion_radius:
                repulsion = calc.adjust_length(repulsion, repulsion_radius)

        return repulsion
