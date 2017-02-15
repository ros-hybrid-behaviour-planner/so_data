"""
Created on 15.02.2017

@author: kaiser

Module including chemotaxis implementations
"""

from geometry_msgs.msg import Vector3
import numpy as np
import gradient
import calc


class ChemotaxisGe(object):
    """
    Chemotaxis behaviour based on formulas by Ge & Cui
    """
    def __init__(self, buffer, frame=None, repulsion=False, moving=True,
                 static=True, maxvel=1.0, minvel=0.1):
        """
        :param buffer:
        :param frame:
        """
        self._buffer = buffer

        self.frames = frame
        self._current_pos = None

        # apply repulsion
        self.repulsion = repulsion
        # consider moving, static or both gradient types
        self.moving = moving
        self.static = static

        self.maxvel=maxvel
        self.minvel=minvel


    def move(self):
        """

        :return:
        """
        vector_attraction = Vector3()
        vector_repulsion = Vector3()
        tmp_att = np.inf
        attractive_gradient = None

        self._current_pos = self._buffer.get_own_pose()

        # get all gradients within view distance
        gradients = self._buffer.gradients(self.frames, self.static,
                                           self.moving, self.repulsion)
        gradients_attractive = gradients[0]
        gradients_repulsive = gradients[1]

        if self._current_pos:
            if gradients_attractive:
                for grdnt in gradients_attractive:
                    # find nearest attractive gradient
                    grad = gradient.calc_attractive_gradient(grdnt,
                                                             self._current_pos)
                    # returns value between 0 and 1
                    att = np.linalg.norm([grad.x, grad.y, grad.z])
                    # attraction smaller means distance closer, normalized
                    # value taken regarding diffusion radius + goal_radius
                    if att < tmp_att:
                        grad = gradient.calc_attractive_gradient_ge(grdnt,
                                                            self._current_pos)
                        vector_attraction = grad
                        tmp_att = att
                        attractive_gradient = grdnt

            if gradients_repulsive:
                for grdnt in gradients_repulsive:
                    if attractive_gradient:
                        grad = gradient.calc_repulsive_gradient_ge(grdnt,
                                                        attractive_gradient,
                                                        self._current_pos)
                    else:  # no attractive gradient nearby, apply repulsion only
                        grad = gradient.calc_repulsive_gradient(grdnt,
                                                            self._current_pos)

                    # robot position is within obstacle goal radius, inf can't be
                    # handled as direction
                    # --> add vector which brings robot to the boarder of the
                    # obstacle
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        dv = calc.delta_vector(self._current_pos.p, grdnt.p)

                        if calc.vector_length(dv) == 0:
                            vector_repulsion = calc.add_vectors(
                                vector_repulsion,
                                calc.random_vector(
                                    grdnt.goal_radius +
                                    grdnt.diffusion))
                        else:
                            vector_repulsion = calc.add_vectors(
                                vector_repulsion,
                                calc.adjust_length(
                                    dv,
                                    grdnt.goal_radius + grdnt.diffusion))
                    else:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                            grad)

        result = calc.add_vectors(vector_attraction, vector_repulsion)
        d = calc.vector_length(result)
        if d > self.maxvel:
            result = calc.adjust_length(result, self.maxvel)
        elif 0 < d < self.minvel:
            result = calc.adjust_length(result, self.minvel)

        return result

    def get_pos(self):
        return self._current_pos


class ChemotaxisBalch(object):
    """
    Chemotaxis behaviour based on formulas by Balch & Hybinette
    """
    def __init__(self, buffer, frame=None, repulsion=False, moving=True,
                 static=True, maxvel=1.0, minvel=0.1):
        """
        :param buffer:
        :param frame:
        """
        self._buffer = buffer

        self.frames = frame
        self._current_pos = None

        # apply repulsion
        self.repulsion = repulsion
        # consider moving, static or both gradient types
        self.moving = moving
        self.static = static

        self.maxvel=maxvel
        self.minvel=minvel


    def move(self):
        """

        :return:
        """
        vector_attraction = Vector3()
        vector_repulsion = Vector3()
        tmp_att = np.inf

        self._current_pos = self._buffer.get_own_pose()

        # get all gradients within view distance
        gradients = self._buffer.gradients(self.frames, self.static,
                                           self.moving, self.repulsion)
        gradients_attractive = gradients[0]
        gradients_repulsive = gradients[1]

        if self._current_pos:
            if gradients_attractive:
                for grdnt in gradients_attractive:
                    # find nearest attractive gradient
                    grad = gradient.calc_attractive_gradient(grdnt,
                                                             self._current_pos)
                    # returns value between 0 and 1
                    att = np.linalg.norm([grad.x, grad.y, grad.z])
                    # attraction smaller means distance closer, normalized
                    # value taken regarding diffusion radius + goal_radius
                    if att < tmp_att:
                        vector_attraction = grad
                        tmp_att = att

            if gradients_repulsive:
                for grdnt in gradients_repulsive:

                    grad = gradient.calc_repulsive_gradient(grdnt,
                                                            self._current_pos)

                    # robot position is within obstacle goal radius, inf can't be
                    # handled as direction
                    # --> add vector which brings robot to the boarder of the
                    # obstacle
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        dv = calc.delta_vector(self._current_pos.p, grdnt.p)

                        if calc.vector_length(dv) == 0:
                            vector_repulsion = calc.add_vectors(
                                vector_repulsion,
                                calc.random_vector(
                                    grdnt.goal_radius +
                                    grdnt.diffusion))
                        else:
                            vector_repulsion = calc.add_vectors(
                                vector_repulsion,
                                calc.adjust_length(
                                    dv,
                                    grdnt.goal_radius + grdnt.diffusion))
                    else:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                            grad)

        result = calc.add_vectors(vector_attraction, vector_repulsion)

        d = calc.vector_length(result)
        if d > self.maxvel:
            result = calc.adjust_length(result, self.maxvel)
        elif 0 < d < self.minvel:
            result = calc.adjust_length(result, self.minvel)

        return result

    def get_pos(self):
        return self._current_pos
