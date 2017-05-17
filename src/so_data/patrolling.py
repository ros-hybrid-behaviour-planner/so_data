"""
Created on 10.05.2017

@author: hrabia, kaiser

Module includes implementation of patrolling mechanism
"""

import random
import rospy
import numpy as np
import gradient
import calc
from geometry_msgs.msg import Vector3
from so_data.msg import SoMessage
from so_data.sobroadcaster import SoBroadcaster
from chemotaxis import AvoidAll





# Movement Behaviour
class Patrol(AvoidAll):
    """
    Patrol: environment following weakest pheromones while depositing new pheromones
    """

    # TODO Idea use a certain timeout for the pheromones after which time they are considered!? If not move into random direction
    # Use two different types of pheromones!?
    # Use a threshold density of gradients!?
    # Get lowest gradient density in sensing range!?
    # Current approach use collision avoidance from pheromones as well

    def __init__(self, buffer, frames=None, moving=True, static=True,
                 maxvel=1.0, minvel=0.5, frame='Pheromone', attraction=-1,
                 ev_factor=0.9, ev_time=10):
        """
        initialize behaviour
        :param buffer: soBuffer
        :param frames: frames to be included in list returned by buffer
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradients in list returned by buffer
        :param maxvel: maximum velocity of agent
        :param minvel: minimum velocity of agent
        :param frame: pheromone frame
        :param attraction: attraction value of pheromone
        :param ev_factor: pheromone evaporation factor
        :param ev_time: pheromone evaporation time
        """

        super(Patrol, self).__init__(buffer=buffer, frames=frames, moving=moving, static=static,
                                                maxvel=maxvel, minvel=minvel)
        self.frame = frame
        self.attraction = attraction
        self.ev_factor = ev_factor
        self.ev_time = ev_time
        self.diffusion = maxvel

        self._broadcaster = SoBroadcaster()

    # def move(self):
    #     """
    #     calculates movement vector
    #     :return: movement vector
    #     """
    #     pose = self._buffer.get_own_pose()
    #     g = None
    #
    #     # weakest gradient
    #     grad = self._buffer.min_reach_attractive_gradient(self.frames,
    #                                                       self.static,
    #                                                       self.moving)
    #
    #     if pose and grad:
    #         g = gradient.calc_attractive_gradient(grad, pose)
    #
    #     if not g:
    #         return None
    #
    #     # adjust length
    #     d = calc.vector_length(g)
    #     if d > self.maxvel:
    #         g = calc.adjust_length(g, self.maxvel)
    #     elif 0 < d < self.minvel:
    #         g = calc.adjust_length(g, self.minvel)
    #
    #     return g

    def patrol_move(self):
        """
        calculates movement vector handling all gradients as repulsive
        :return: movement vector
        """

        pose = self._buffer.get_own_pose()
        result = None

        # repulsive gradients
        gradients = self._buffer.gradients(self.frames, self.static,
                                           self.moving)

        if pose:
            if gradients:
                result = Vector3()

                if len(gradients) > 1:

                    for grdnt in gradients:
                        grad = gradient.calc_repulsive_gradient(grdnt, pose)

                        # robot position is within obstacle goal radius
                        # handle infinite repulsion
                        if grad.x == np.inf or grad.x == -1 * np.inf:
                            # create random vector with length (goal_radius +
                            # gradient.diffusion)
                            dv = calc.delta_vector(pose.p, grdnt.p)

                            # ignore zero length vectors (gradients we are directly on top of)
                            if calc.vector_length(dv) != 0:

                                result = calc.add_vectors(result,
                                                          calc.adjust_length(dv,
                                                                grdnt.goal_radius
                                                                + grdnt.diffusion))
                        else:
                            result = calc.add_vectors(result, grad)
                else:
                    grdnt = gradients[0]
                    result = calc.random_vector(grdnt.goal_radius + grdnt.diffusion)

        if not result:
            return None

        # adjust length
        d = calc.vector_length(result)
        if d > self.maxvel:
            result = calc.adjust_length(result, self.maxvel)
        elif 0 < d < self.minvel:
            result = calc.adjust_length(result, self.minvel)

        return result

    def move(self):
        """
        deposit pheromone and move towards nest
        :return: movement vector
        """
        # spread pheromone
        self.spread()

        return self.patrol_move()

    def spread(self):
        """
        method to spread pheromones
        :return:
        """
        # create gossip message
        msg = SoMessage()
        msg.header.frame_id = self.frame
        msg.parent_frame = self._buffer.id

        now = rospy.Time.now()
        msg.header.stamp = now
        msg.ev_stamp = now

        current_pose = self._buffer.get_own_pose()
        msg.p = current_pose.p
        msg.q = current_pose.q
        msg.attraction = self.attraction

        msg.diffusion = self.diffusion

        msg.goal_radius = 0  # no goal radius
        msg.ev_factor = self.ev_factor
        msg.ev_time = self.ev_time

        msg.moving = False  # static as pheromone is deposited in environment

        # spread gradient
        self._broadcaster.send_data(msg)
