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


# Mechanisms to reach one attractive Gradient & to avoid repulsive gradients
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


# TODO: TEST
# Mechanisms to consider specific sets of gradients
class CollisionAvoidance(MovementPattern):
    """
    movement mechanism to avoid all repulsive gradients
    """
    def __init__(self, buffer, frames=None, repulsion=False, moving=True,
                 static=True, maxvel=1.0, minvel=0.1):
        """
        :param buffer:
        :param frame:
        """
        super(CollisionAvoidance, self).__init__(buffer, frames, repulsion,
                                              moving, static, maxvel, minvel)

    def move(self):

        self._current_pos = self._buffer.get_own_pose()
        vector_repulsion = Vector3()

        # repulsive gradients
        gradients_repulsive = self._buffer.repulsive_gradients(self.frames,
                                                               self.static,
                                                               self.moving,
                                                               self.repulsion)

        if self._current_pos:
            if gradients_repulsive:
                for grdnt in gradients_repulsive:

                    grad = gradient.calc_repulsive_gradient(grdnt,
                                                            self._current_pos)

                    # robot position is within obstacle goal radius,
                    # inf can't be handled as direction
                    # --> add vector which brings robot to the boarder of the
                    # obstacle
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        dv = calc.delta_vector(self._current_pos.p, grdnt.p)

                        if calc.vector_length(dv) == 0:
                            vector_repulsion = calc.add_vectors(
                                vector_repulsion, calc.random_vector(
                                    grdnt.goal_radius + grdnt.diffusion))
                        else:
                            vector_repulsion = calc.add_vectors(
                                vector_repulsion, calc.adjust_length(dv,
                                        grdnt.goal_radius + grdnt.diffusion))
                    else:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                            grad)

        # adjust length
        d = calc.vector_length(vector_repulsion)
        if d > self.maxvel:
            vector_repulsion = calc.adjust_length(vector_repulsion, self.maxvel)
        elif 0 < d < self.minvel:
            vector_repulsion = calc.adjust_length(vector_repulsion, self.minvel)

        return vector_repulsion


class FollowAll(MovementPattern):
    """
    movement mechanism to follow overall potential
    """
    def __init__(self, buffer, frames=None, repulsion=False, moving=True,
                 static=True, maxvel=1.0, minvel=0.1):
        """
        :param buffer:
        :param frame:
        """
        super(FollowAll, self).__init__(buffer, frames, repulsion, moving,
                                        static, maxvel, minvel)

    def move(self):

        self._current_pos = self._buffer.get_own_pose()
        result = Vector3()

        # repulsive gradients
        gradients = self._buffer.gradients(self.frames, self.static,
                                           self.moving, self.repulsion)

        if self._current_pos:
            if gradients:
                for grdnt in gradients:

                    if grdnt.attraction == 1:
                        result = calc.adjust_length(result, gradient.
                                                    calc_attractive_gradient(
                            grdnt, self._current_pos))
                    elif grdnt.attraction == -1:
                        grad = gradient.calc_repulsive_gradient(grdnt,
                                                            self._current_pos)

                        # robot position is within obstacle goal radius,
                        # inf can't be handled as direction
                        # --> add vector which brings robot to the boarder of
                        # the obstacle
                        if grad.x == np.inf or grad.x == -1 * np.inf:
                            # create random vector with length (goal_radius +
                            # gradient.diffusion)
                            dv = calc.delta_vector(self._current_pos.p,
                                                   grdnt.p)

                            if calc.vector_length(dv) == 0:
                                result = calc.add_vectors(result,
                                                          calc.random_vector(
                                                              grdnt.goal_radius
                                                              + grdnt.diffusion
                                                          ))
                            else:
                                result = calc.add_vectors(result,
                                                          calc.adjust_length(
                                                              dv,
                                                              grdnt.goal_radius
                                                              + grdnt.diffusion
                                                          ))
                        else:
                            result = calc.add_vectors(result, grad)

            # adjust length
        d = calc.vector_length(result)
        if d > self.maxvel:
            result = calc.adjust_length(result, self.maxvel)
        elif 0 < d < self.minvel:
            result = calc.adjust_length(result, self.minvel)

        return result


class AvoidAll(MovementPattern):
    """
    movement mechanism to avoid all sensed gradients
    """
    def __init__(self, buffer, frames=None, repulsion=False, moving=True,
                 static=True, maxvel=1.0, minvel=0.1):
        """
        :param buffer:
        :param frame:
        """
        super(AvoidAll, self).__init__(buffer, frames, repulsion, moving,
                                        static, maxvel, minvel)

    def move(self):

        self._current_pos = self._buffer.get_own_pose()
        result = Vector3()

        # repulsive gradients
        gradients = self._buffer.gradients(self.frames, self.static,
                                           self.moving, self.repulsion)

        if self._current_pos:
            if gradients:
                for grdnt in gradients:
                    grad = gradient.calc_repulsive_gradient(grdnt,
                                                            self._current_pos)

                    # robot position is within obstacle goal radius,
                    # inf can't be handled as direction
                    # --> add vector which brings robot to the boarder of
                    # the obstacle
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        dv = calc.delta_vector(self._current_pos.p, grdnt.p)

                        if calc.vector_length(dv) == 0:
                            result = calc.add_vectors(result,
                                                      calc.random_vector(
                                                          grdnt.goal_radius
                                                          + grdnt.diffusion))
                        else:
                            result = calc.add_vectors(result,
                                                      calc.adjust_length(dv,
                                                            grdnt.goal_radius
                                                            + grdnt.diffusion))
                    else:
                        result = calc.add_vectors(result, grad)

            # adjust length
        d = calc.vector_length(result)
        if d > self.maxvel:
            result = calc.adjust_length(result, self.maxvel)
        elif 0 < d < self.minvel:
            result = calc.adjust_length(result, self.minvel)

        return result


class FollowMax(MovementPattern):
    """
    movement mechanism to follow strongest gradient
    """
    def __init__(self, buffer, frames=None, repulsion=False, moving=True,
                 static=True, maxvel=1.0, minvel=0.1):
        """
        :param buffer:
        :param frame:
        """
        super(FollowMax, self).__init__(buffer, frames, repulsion, moving,
                                        static, maxvel, minvel)

    def move(self):

        self._current_pos = self._buffer.get_own_pose()
        g = Vector3()

        # strongest gradient
        grad = self._buffer.strongest_gradient(self.frames, self.static,
                                                   self.moving)

        if self._current_pos:
            if grad:
                if grad.attraction == 1:
                    g = gradient.calc_attractive_gradient(grad,
                                                          self._current_pos)
                elif grad.attraction == -1:
                    g = gradient.calc_repulsive_gradient(grad,
                                                         self._current_pos)

                    # robot position is within obstacle goal radius,
                    # inf can't be handled as direction
                    # --> add vector which brings robot to the boarder of
                    # the obstacle
                    if g.x == np.inf or g.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        dv = calc.delta_vector(self._current_pos.p, grad.p)

                        if calc.vector_length(dv) == 0:
                            g = calc.random_vector(grad.goal_radius +
                                                   grad.diffusion)
                        else:
                            g = calc.adjust_length(dv, grad.goal_radius
                                                   + grad.diffusion)


        # adjust length
        d = calc.vector_length(g)
        if d > self.maxvel:
            g = calc.adjust_length(g, self.maxvel)
        elif 0 < d < self.minvel:
            g = calc.adjust_length(g, self.minvel)

        return g
