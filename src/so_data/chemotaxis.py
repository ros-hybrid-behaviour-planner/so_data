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




# TODO
    # AGGREGATION - build potential field (merging of information)
    def result_max(self, frames=None):
        """
        follow higher gradient values (= gradient with shortest relative
        distance to gradient source)
        sets current gradient to direction vector (length <= 1)
        :return gradient vector
        """

        gradients = []
        tmp_att = -1
        tmp_grad = Vector3()

        # if no frameids are specified, use all data stored in buffer
        if not frames:
            frames = []
            if self.result_static:
                frames += self._static.keys()
            if self.result_moving:
                frames += self._moving.keys()

        # find moving and / or static gradients within view distance
        gradients = self.gradients(frames)
        gradients = gradients[0] + gradients[1]

        # find gradient with highest value ( = closest relative distance)
        if gradients:
            for gradient in gradients:
                if gradient.attraction == 1:
                    grad = self._calc_attractive_gradient(gradient)
                else:
                    grad = self._calc_repulsive_gradient(gradient)

                if grad.x == np.inf or grad.x == -1 * np.inf:
                    att = np.inf
                else:
                    att = np.linalg.norm([grad.x, grad.y, grad.z])
                    # inverse attraction for attractive gradients as attraction
                    #  decreases with getting closer to the
                    # goal zone of the gradient
                    if gradient.attraction == 1:
                        att = 1 - att

                if att > tmp_att:
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)

                        dv = calc.delta_vector(self._own_pos[-1].p, gradient.p)

                        if calc.vector_length(dv) == 0:
                            tmp_grad = calc.random_vector(gradient.goal_radius
                                                  + gradient.diffusion)
                        else:
                            tmp_grad = calc.adjust_length(dv,
                                                          gradient.goal_radius
                                                          + gradient.diffusion)

                    else:
                        tmp_grad = grad

                    tmp_att = att

        return tmp_grad

    def result_collision(self, frames=None):
        """
        aggregate gradients to avoid all repulsive gradients
        :return: repulsive gradient vector
        """
        vector_repulsion = Vector3()

        # if no frameids are specified, use all data stored in buffer
        if not frames:
            frames = []
            if self.result_static:
                frames += self._static.keys()
            if self.result_moving:
                frames += self._moving.keys()

        gradients = self.repulsive_gradients(frames)

        # aggregate repulsive gradients
        if gradients:
            for gradient in gradients:
                grad = self._calc_repulsive_gradient(gradient)
                # robot position is within obstacle radius, inf can't be
                # handled as direction
                # --> add vector which brings robot to the boarder of the
                # obstacle
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length (goal_radius +
                    # gradient.diffusion)
                    dv = calc.delta_vector(self._own_pos[-1].p, gradient.p)

                    if calc.vector_length(dv) == 0:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                calc.random_vector(
                                                    gradient.goal_radius +
                                                    gradient.diffusion))
                    else:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                    calc.adjust_length(dv,
                                                        gradient.goal_radius +
                                                        gradient.diffusion))
                else:
                    vector_repulsion = calc.add_vectors(vector_repulsion, grad)

        return vector_repulsion



    def result_all(self, frames=None):
        """
        aggregate all vectors within view distance
        :return: gradient vector
        """

        vector_attraction = Vector3()
        vector_repulsion = Vector3()

        # if no frameids are specified, use all data stored in buffer
        if not frames:
            frames = []
            if self.result_static:
                frames += self._static.keys()
            if self.result_moving:
                frames += self._moving.keys()

        # find gradients within view distance
        gradients = self.gradients(frames)
        gradients_attractive = gradients[0]
        gradients_repulsive = gradients[1]

        if gradients_attractive:
            for gradient in gradients_attractive:
                # sum up all attractive gradients
                grad = self._calc_attractive_gradient(gradient)
                vector_attraction = calc.add_vectors(vector_attraction, grad)

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                grad = self._calc_repulsive_gradient(gradient)
                # robot position is within obstacle radius, inf can't be
                # handled as direction
                # --> add vector which brings robot to the boarder of the
                # obstacle
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length=goal radius + diffusion
                    dv = calc.delta_vector(self._own_pos[-1].p, gradient.p)

                    if calc.vector_length(dv) == 0:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                            calc.random_vector(
                                                        gradient.goal_radius
                                                        + gradient.diffusion))
                    else:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                            calc.adjust_length(
                                                                dv,
                                    gradient.goal_radius + gradient.diffusion))
                else:
                    vector_repulsion = calc.add_vectors(vector_repulsion, grad)

        return calc.add_vectors(vector_attraction, vector_repulsion)

    def result_avoid(self, frames=None):
        """
        calculate vector which avoids all gradients within view distance
        :return gradient vector
        """
        gradients = []
        v = Vector3()

        # if no frameids are specified, use all data stored in buffer
        if not frames:
            frames = []
            if self.result_static:
                frames += self._static.keys()
            if self.result_moving:
                frames += self._moving.keys()

        # find moving and / or static gradients within view distance
        gradients = self.gradients(frames)
        gradients = gradients[0] + gradients[1]

        if gradients:
            for gradient in gradients:
                grad = self._calc_repulsive_gradient(gradient)
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length (goal_radius +
                    # gradient.diffusion)
                    dv = calc.delta_vector(self._own_pos[-1].p, gradient.p)

                    if calc.vector_length(dv) == 0:
                        v = calc.add_vectors(v, calc.random_vector(
                                            gradient.goal_radius
                                            + gradient.diffusion))
                    else:
                        v = calc.add_vectors(v, calc.adjust_length(dv,
                                    gradient.goal_radius + gradient.diffusion))

                else:
                    v = calc.add_vectors(v, grad)

        return v

