"""
Created on 21.03.2017

@author: kaiser

Module including supplementary mechanisms: DepositPheromonesMin,
DepositPheromonesRandom
"""

import rospy
from so_data.msg import SoMessage
from so_data.sobroadcaster import SoBroadcaster
from chemotaxis import FollowMinReach
from foraging import Exploration


# Movement Behaviour
class DepositPheromonesMin(FollowMinReach):
    """
    deposit pheromones while following pheromone with minimum reach within
    view
    """
    def __init__(self, buffer, frames=None, moving=False, static=True,
                 maxvel=1.0, minvel=0.5, frame='Pheromone', attraction=1,
                 ev_factor=0.9, ev_time=5):
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

        super(DepositPheromonesMin, self).__init__(buffer, frames, moving,
                                                   static, maxvel, minvel)
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

        return super(DepositPheromonesMin, self).move()

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


# Movement Behaviour
class DepositPheromonesRandom(Exploration):
    """
    depositing pheromones while walking random
    """
    def __init__(self, buffer, maxvel=1.0, minvel=0.5, frame='Pheromone',
                 attraction=1, ev_factor=0.9, ev_time=5):
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

        super(DepositPheromonesRandom, self).__init__(buffer, maxvel, minvel)
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

        return super(DepositPheromonesRandom, self).move()

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