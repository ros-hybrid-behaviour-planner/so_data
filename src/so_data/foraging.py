"""
Created on 27.02.2017

@author: kaiser

Module including sample implementation of ant foraging
"""

import random
import gradient
import calc
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from so_data.msg import SoMessage
from so_data.sobroadcaster import SoBroadcaster
from patterns import MovementPattern, DecisionPattern
from chemotaxis import ChemotaxisBalch


class STATE(object):
    NONE = 1
    EXPLORATION = 2
    EXPLOITATION = 3
    RETURN = 4


class ForagingPheromones(ChemotaxisBalch):
    """
    Foraging: set state and move to nest while depositing pheromones
    enhancement of ChemotaxisBalch behaviour
    """
    def __init__(self, buffer, frames=None, repulsion=False,
                 moving=False, static=True, maxvel=1.0, minvel=0.5,
                 frame='Pheromone', attraction=1, ev_factor=0.9, ev_time=10):
        """
        initialize behaviour
        :param value: exploration probability
        """

        super(ForagingPheromones, self).__init__(buffer, frames, repulsion,
                                                 moving, static, maxvel,
                                                 minvel)
        self.frame = frame
        self.attraction = attraction
        self.ev_factor = ev_factor
        self.ev_time = ev_time
        self.diffusion = maxvel

        self._broadcaster = SoBroadcaster()

    def move(self):
        """
        :return: movement vector
        """
        # spread pheromone
        self.spread()

        return super(ForagingPheromones, self).move()

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


class ForagingDecision(DecisionPattern):
    """
    Decision mechanism for foraging: explore vs exploit
    """
    def __init__(self, buffer, probability):
        """
        :param probability:
        :param buffer:
        """
        super(ForagingDecision, self).__init__(buffer, value=probability)

    def calc_value(self):
        """
        determines whether agent will explore or exploit
        """
        # set state
        if random.random() < self.value:
            state = STATE.EXPLORATION
        else:
            state = STATE.EXPLOITATION

        return [self.value, state]


class FollowTrail(MovementPattern):
    """
    Foraging - Follow pheromone trail behaviour (chemotaxis)
    """
    def __init__(self, buffer, frame, angle=1.3, repulsion=False,
                 moving=False, static=True, maxvel=1.0, minvel=0.5):
        """
        initialize behaviour
        :param value: exploration probability
        """
        super(FollowTrail, self).__init__(buffer, frame, repulsion, moving,
                                          static, maxvel, minvel)

        self.angle = angle

    def move(self):
        """
        :return: movement vector following trail
        """
        pose = self._buffer.get_own_pose()

        # get all gradients within view distance

        view = self._buffer.static_list_angle(self.frames, self.angle)

        # attractive gradient
        result = Vector3()

        if pose:
            if view:
                for grdnt in view:
                    grad = gradient.calc_attractive_gradient(grdnt, pose)
                    result = calc.add_vectors(result, grad)

        d = calc.vector_length(result)
        if d > self.maxvel:
            result = calc.adjust_length(result, self.maxvel)
        elif 0 < d < self.minvel:
            result = calc.adjust_length(result, self.minvel)

        return result


# WalkRandom
class WalkRandom(MovementPattern):
    """
    movement mechanism to follow random movement
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
        super(WalkRandom, self).__init__(buffer, frames, repulsion, moving,
                                         static, maxvel, minvel)

    def move(self):
        """
        calculates random movement vector
        :return: movement vector
        """
        tmp = np.random.rand(1, 3)

        g = Vector3()
        g.x = 2 * tmp[0][0] - 1
        g.y = 2 * tmp[0][1] - 1
        g.z = 2 * tmp[0][2] - 1

        # adjust length
        d = calc.vector_length(g)
        if d > self.maxvel:
            g = calc.adjust_length(g, self.maxvel)
        elif 0 < d < self.minvel:
            g = calc.adjust_length(g, self.minvel)

        return g
