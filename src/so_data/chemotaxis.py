"""
Created on 15.02.2017

@author: kaiser

Module including chemotaxis implementations
"""

import numpy as np
import gradient
import calc
from geometry_msgs.msg import Vector3
from patterns import MovementPattern


# Mechanisms to reach one attractive Gradient & to avoid repulsive gradients
class ChemotaxisGe(MovementPattern):
    """
    Chemotaxis behaviour based on formulas by Ge & Cui
    Tries to reach a goal (attractive gradient) while avoiding obstacles
    (repulsive gradients)
    """
    def __init__(self, buffer, frames=None, moving=True, static=True,
                 maxvel=1.0, minvel=0.1):
        """
        :param buffer: soBuffer
        :param frames: frames to be included in list returned by buffer
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradients in list returned by buffer
        :param maxvel: maximum velocity of agent
        :param minvel: minimum velocity of agent
        """
        super(ChemotaxisGe, self).__init__(buffer, frames, moving, static,
                                           maxvel, minvel)

        self.goal = None

    def move(self):
        """
        :return: movement vector
        """
        vector_repulsion = Vector3()

        pose = self._buffer.get_own_pose()

        # repulsive gradients
        gradients_repulsive = self._buffer.repulsive_gradients(self.frames,
                                                               self.static,
                                                               self.moving)

        # attractive gradient / set goal
        vector_attraction = self.goal_gradient()

        if pose:
            if gradients_repulsive:
                for grdnt in gradients_repulsive:
                    if self.goal:
                        grad = gradient.calc_repulsive_gradient_ge(grdnt,
                                                                   self.goal,
                                                                   pose)
                    else:  # no attractive gradient, apply repulsion only
                        grad = gradient.calc_repulsive_gradient(grdnt, pose)

                    # robot position is within obstacle goal radius
                    # handle infinite repulsion
                    if abs(grad.x) == np.inf or abs(grad.y) == np.inf or \
                                    abs(grad.z) == np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        dv = calc.delta_vector(pose.p, grdnt.p)

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
        if vector_attraction:
            result = calc.add_vectors(vector_attraction, vector_repulsion)
        else:
            result = vector_repulsion

        d = calc.vector_length(result)
        if d > self.maxvel:
            result = calc.adjust_length(result, self.maxvel)
        elif 0 < d < self.minvel:
            result = calc.adjust_length(result, self.minvel)

        return result

    def goal_gradient(self):
        """
        :return: normalized vector to goal gradient
        """
        grad = None

        self.goal = self._buffer.max_attractive_gradient(self.frames,
                                                         self.static,
                                                         self.moving)
        pose = self._buffer.get_own_pose()

        if self.goal and pose:
            grad = gradient.calc_attractive_gradient_ge(self.goal, pose)

        return grad


class ChemotaxisBalch(MovementPattern):
    """
    Chemotaxis behaviour based on formulas by Balch & Hybinette
    Tries to reach a goal (attractive gradient) while avoiding obstacles
    (repulsive gradients)
    """
    def __init__(self, buffer, frames=None, moving=True, static=True,
                 maxvel=1.0, minvel=0.1):
        """
        :param buffer: soBuffer
        :param frames: frames to be included in list returned by buffer
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradients in list returned by buffer
        :param maxvel: maximum velocity of agent
        :param minvel: minimum velocity of agent
        """
        super(ChemotaxisBalch, self).__init__(buffer, frames, moving, static,
                                              maxvel, minvel)

    def move(self):
        """
        :return: movement vector
        """
        vector_repulsion = Vector3()

        pose = self._buffer.get_own_pose()

        # get all gradients within view distance
        # repulsive gradients
        gradients_repulsive = self._buffer.repulsive_gradients(self.frames,
                                                               self.static,
                                                               self.moving)

        # attractive gradient
        vector_attraction = self.goal_gradient()

        if pose:
            if gradients_repulsive:
                for grdnt in gradients_repulsive:

                    grad = gradient.calc_repulsive_gradient(grdnt, pose)

                    # robot position is within obstacle goal radius
                    # handle infinite repulsion
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        dv = calc.delta_vector(pose.p, grdnt.p)

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
        if vector_attraction:
            result = calc.add_vectors(vector_attraction, vector_repulsion)
        else:
            result = vector_repulsion

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

        attractive = self._buffer.max_attractive_gradient(self.frames,
                                                          self.static,
                                                          self.moving)
        pose = self._buffer.get_own_pose()

        if attractive and pose:
            grad = gradient.calc_attractive_gradient(attractive, pose)

        return grad


# Mechanisms to consider specific sets of gradients
class CollisionAvoidance(MovementPattern):
    """
    movement mechanism to avoid all repulsive gradients
    """
    def __init__(self, buffer, frames=None, moving=True, static=True,
                 maxvel=1.0, minvel=0.1):
        """
        :param buffer: soBuffer
        :param frames: frames to be included in list returned by buffer
        :param repulsion: enable collision avoidance between agents
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradients in list returned by buffer
        :param maxvel: maximum velocity of agent
        :param minvel: minimum velocity of agent
        """
        super(CollisionAvoidance, self).__init__(buffer, frames, moving,
                                                 static, maxvel, minvel)

    def move(self):
        """
        calculates movement vector to avoid all repulsive gradients
        :return: movement vector
        """

        pose = self._buffer.get_own_pose()
        vector_repulsion = Vector3()

        # repulsive gradients
        gradients_repulsive = self._buffer.repulsive_gradients(self.frames,
                                                               self.static,
                                                               self.moving)

        if pose:
            if gradients_repulsive:
                for grdnt in gradients_repulsive:

                    grad = gradient.calc_repulsive_gradient(grdnt, pose)

                    # robot position is within obstacle goal radius
                    # handle infinite repulsion
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        dv = calc.delta_vector(pose.p, grdnt.p)

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
    def __init__(self, buffer, frames=None, moving=True, static=True,
                 maxvel=1.0, minvel=0.1):
        """
        :param buffer: soBuffer
        :param frames: frames to be included in list returned by buffer
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradients in list returned by buffer
        :param maxvel: maximum velocity of agent
        :param minvel: minimum velocity of agent
        """
        super(FollowAll, self).__init__(buffer, frames, moving, static, maxvel,
                                        minvel)

    def move(self):
        """
        calculates movement vector considering all gradients
        :return: movement vector
        """

        pose = self._buffer.get_own_pose()
        result = Vector3()

        # repulsive gradients
        gradients = self._buffer.gradients(self.frames, self.static,
                                           self.moving)

        if pose:
            if gradients:
                for grdnt in gradients:

                    if grdnt.attraction == 1:
                        result = calc.add_vectors(result, gradient.
                                                  calc_attractive_gradient(
                            grdnt, pose))
                    elif grdnt.attraction == -1:
                        grad = gradient.calc_repulsive_gradient(grdnt, pose)

                        # robot position is within obstacle goal radius
                        # handle infinite repulsion
                        if grad.x == np.inf or grad.x == -1 * np.inf:
                            # create random vector with length (goal_radius +
                            # gradient.diffusion)
                            dv = calc.delta_vector(pose.p, grdnt.p)

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
        :param buffer: soBuffer
        :param frames: frames to be included in list returned by buffer
        :param repulsion: enable collision avoidance between agents
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradients in list returned by buffer
        :param maxvel: maximum velocity of agent
        :param minvel: minimum velocity of agent
        """
        super(AvoidAll, self).__init__(buffer, frames, moving, static, maxvel,
                                       minvel)

    def move(self):
        """
        calculates movement vector handling all gradients as repulsive
        :return: movement vector
        """

        pose = self._buffer.get_own_pose()
        result = Vector3()

        # repulsive gradients
        gradients = self._buffer.gradients(self.frames, self.static,
                                           self.moving)

        if pose:
            if gradients:
                for grdnt in gradients:
                    grad = gradient.calc_repulsive_gradient(grdnt, pose)

                    # robot position is within obstacle goal radius
                    # handle infinite repulsion
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        dv = calc.delta_vector(pose.p, grdnt.p)

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
    movement mechanism to follow relatively strongest gradient
    (min movement vector length)
    """
    def __init__(self, buffer, frames=None, moving=True, static=True,
                 maxvel=1.0, minvel=0.1):
        """
        :param buffer: soBuffer
        :param frames: frames to be included in list returned by buffer
        :param repulsion: enable collision avoidance between agents
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradients in list returned by buffer
        :param maxvel: maximum velocity of agent
        :param minvel: minimum velocity of agent
        """
        super(FollowMax, self).__init__(buffer, frames, moving, static, maxvel,
                                        minvel)

    def move(self):
        """
        calculate movement vector
        :return: movement vector
        """
        pose = self._buffer.get_own_pose()
        g = Vector3()

        # strongest gradient
        grad = self._buffer.strongest_gradient(self.frames, self.static,
                                               self.moving)

        if pose:
            if grad:
                if grad.attraction == 1:
                    g = gradient.calc_attractive_gradient(grad, pose)
                elif grad.attraction == -1:
                    g = gradient.calc_repulsive_gradient(grad, pose)

                    # robot position is within obstacle goal radius
                    # handle infinite repulsion
                    if g.x == np.inf or g.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        dv = calc.delta_vector(pose.p, grad.p)

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


class FollowMin(MovementPattern):
    """
    movement mechanism to follow relatively weakest gradient
    (max movement vector length)
    """
    def __init__(self, buffer, frames=None, moving=True, static=True,
                 maxvel=1.0, minvel=0.1):
        """
        :param buffer: soBuffer
        :param frames: frames to be included in list returned by buffer
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradients in list returned by buffer
        :param maxvel: maximum velocity of agent
        :param minvel: minimum velocity of agent
        """
        super(FollowMin, self).__init__(buffer, frames, moving, static, maxvel,
                                        minvel)

    def move(self):
        """
        calculate movement vector
        :return: movement vector
        """
        pose = self._buffer.get_own_pose()
        g = Vector3()

        # relatively weakest gradient
        grad = self._buffer.min_attractive_gradient(self.frames, self.static,
                                                    self.moving)

        if pose and grad:
            g = gradient.calc_attractive_gradient(grad, pose)

        # adjust length
        d = calc.vector_length(g)
        if d > self.maxvel:
            g = calc.adjust_length(g, self.maxvel)
        elif 0 < d < self.minvel:
            g = calc.adjust_length(g, self.minvel)

        return g


class FollowMinReach(MovementPattern):
    """
    movement mechanism to follow gradient with minimum reach
    """
    def __init__(self, buffer, frames=None, moving=True, static=True,
                 maxvel=1.0, minvel=0.1):
        """
        :param buffer: soBuffer
        :param frames: frames to be included in list returned by buffer
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradients in list returned by buffer
        :param maxvel: maximum velocity of agent
        :param minvel: minimum velocity of agent
        """
        super(FollowMinReach, self).__init__(buffer, frames, moving, static,
                                             maxvel, minvel)

    def move(self):
        """
        calculates movement vector
        :return: movement vector
        """
        pose = self._buffer.get_own_pose()
        g = Vector3()

        # weakest gradient
        grad = self._buffer.min_reach_attractive_gradient(self.frames,
                                                          self.static,
                                                          self.moving)

        if pose and grad:
            g = gradient.calc_attractive_gradient(grad, pose)

        # adjust length
        d = calc.vector_length(g)
        if d > self.maxvel:
            g = calc.adjust_length(g, self.maxvel)
        elif 0 < d < self.minvel:
            g = calc.adjust_length(g, self.minvel)

        return g


class FollowMaxReach(MovementPattern):
    """
    movement mechanism to follow gradient with maximum reach
    """
    def __init__(self, buffer, frames=None, moving=True, static=True,
                 maxvel=1.0, minvel=0.1):
        """
        :param buffer: soBuffer
        :param frames: frames to be included in list returned by buffer
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradients in list returned by buffer
        :param maxvel: maximum velocity of agent
        :param minvel: minimum velocity of agent
        """
        super(FollowMaxReach, self).__init__(buffer, frames, moving, static,
                                             maxvel, minvel)

    def move(self):
        """
        calculates movement vector
        :return: movement vector
        """
        pose = self._buffer.get_own_pose()
        g = Vector3()

        # strongest gradient
        grad = self._buffer.max_reach_attractive_gradient(self.frames,
                                                          self.static,
                                                          self.moving)

        if pose and grad:
            g = gradient.calc_attractive_gradient(grad, pose)

        # adjust length
        d = calc.vector_length(g)
        if d > self.maxvel:
            g = calc.adjust_length(g, self.maxvel)
        elif 0 < d < self.minvel:
            g = calc.adjust_length(g, self.minvel)

        return g