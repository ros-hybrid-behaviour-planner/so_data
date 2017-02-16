"""
Created on 15.02.2017

@author: kaiser

Module including chemotaxis implementations
"""

from geometry_msgs.msg import Vector3
import numpy as np
import gradient
import calc
from patterns import MovementPattern


class ChemotaxisGe(MovementPattern):
    """
    Chemotaxis behaviour based on formulas by Ge & Cui
    """
    def __init__(self, buffer, frames=None, repulsion=False, moving=True,
                 static=True, maxvel=1.0, minvel=0.1):
        """
        :param buffer:
        :param frame:
        """
        super(ChemotaxisGe, self).__init__(buffer, frames, repulsion, moving,
                                           static, maxvel, minvel)

    def move(self):
        """

        :return:
        """
        vector_attraction = Vector3()
        vector_repulsion = Vector3()

        self._current_pos = self._buffer.get_own_pose()

        # repulsive gradients
        gradients_repulsive = self._buffer.repulsive_gradients(self.frames,
                                                               self.static,
                                                               self.moving,
                                                               self.repulsion)

        # attractive gradient
        attractive_gradient = self._buffer.get_attractive_gradient(self.frames,
                                                                   self.static,
                                                                   self.moving)

        if self._current_pos:

            if attractive_gradient:
                vector_attraction = gradient.calc_attractive_gradient_ge(
                    attractive_gradient, self._current_pos)

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

    def goal_gradient(self):
        """

        :return: vector to goal gradient
        """
        grad = None

        attractive = self._buffer.get_attractive_gradient(self.frames,
                                                          self.static,
                                                          self.moving)

        if attractive:
            grad = gradient.calc_attractive_gradient_ge(attractive,
                                                self._buffer.get_own_pose())

        return grad


class ChemotaxisBalch(MovementPattern):
    """
    Chemotaxis behaviour based on formulas by Balch & Hybinette
    """
    def __init__(self, buffer, frames=None, repulsion=False, moving=True,
                 static=True, maxvel=1.0, minvel=0.1):
        """
        :param buffer:
        :param frame:
        """
        super(ChemotaxisBalch, self).__init__(buffer, frames, repulsion,
                                              moving, static, maxvel, minvel)

    def move(self):
        """

        :return:
        """
        vector_attraction = Vector3()
        vector_repulsion = Vector3()
        tmp_att = np.inf

        self._current_pos = self._buffer.get_own_pose()

        # get all gradients within view distance
        # repulsive gradients
        gradients_repulsive = self._buffer.repulsive_gradients(self.frames,
                                                               self.static,
                                                               self.moving,
                                                               self.repulsion)

        # attractive gradient
        attractive_gradient = self._buffer.get_attractive_gradient(self.frames,
                                                                   self.static,
                                                                   self.moving)

        if self._current_pos:
            if attractive_gradient:
                vector_attraction = gradient.calc_attractive_gradient_ge(
                    attractive_gradient, self._current_pos)

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

    def goal_gradient(self):
        """

        :return: vector to goal gradient
        """
        grad = None

        attractive = self._buffer.get_attractive_gradient(self.frames,
                                                          self.static,
                                                          self.moving)

        if attractive:
            grad = gradient.calc_attractive_gradient(attractive,
                                                self._buffer.get_own_pose())

        return grad