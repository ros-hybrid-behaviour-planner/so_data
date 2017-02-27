"""
Created on 27.02.2017

@author: kaiser

Module including sample implementation of ant foraging
"""

import random
from patterns import MovementPattern
import gradient
import calc
import rospy
from geometry_msgs.msg import Vector3
from so_data.msg import SoMessage
import numpy as np
from so_data.sobroadcaster import SoBroadcaster


class STATE(object):
    NONE = 1
    EXPLORATION = 2
    EXPLOITATION = 3


class Foraging(MovementPattern):
    """
    Foraging
    """
    def __init__(self, probability,  buffer, frames=None, repulsion=False,
                 moving=False, static=True, maxvel=1.0, minvel=0.5,
                 state=0, frame='Pheromone', attraction=1, ev_factor=0.8,
                 ev_time=5):
        """
        initialize behaviour
        :param value: exploration probability
        """
        super(Foraging, self).__init__(buffer, frames, repulsion, moving,
                                       static, maxvel, minvel)

        self.state = state
        self.probability = probability

        self.frame = frame
        self.attraction = attraction
        self.ev_factor = ev_factor
        self.ev_time = ev_time
        self.diffusion = maxvel

        self._broadcaster = SoBroadcaster()

    def set_state(self):
        """
        defines whether agent is exploring or exploiting
        """
        # set state
        if random.random() < self.probability:
            self.state = STATE.EXPLORATION
        else:
            self.state = STATE.EXPLOITATION

    def reset_state(self):
        """
        resets state to None
        """
        self.state = STATE.NONE

    def move(self):
        """
        :return: movement vector
        """
        # spread pheromone
        self.spread()

        vector_repulsion = Vector3()

        pose = self._buffer.get_own_pose()

        # get all gradients within view distance
        # repulsive gradients
        gradients_repulsive = self._buffer.repulsive_gradients(self.frames,
                                                               self.static,
                                                               self.moving,
                                                               self.repulsion)

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

    def spread(self):
        """
        method to spread new values
        :return:
        """
        # create gossip message
        msg = SoMessage()
        msg.header.frame_id = self.frame
        msg.parent_frame = self._buffer.id

        now = rospy.Time.now()
        msg.header.stamp = now
        msg.ev_stamp = now

        # important to determine whether gradient is within view
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
