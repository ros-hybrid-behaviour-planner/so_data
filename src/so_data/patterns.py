"""
Created on 15.02.2017

@author: kaiser

Module including abstract classes for movement and decision patterns
"""

from abc import ABCMeta, abstractmethod

import rospy
from diagnostic_msgs.msg import KeyValue

from so_data.msg import SoMessage
from so_data.sobroadcaster import SoBroadcaster


class MovementPattern(object):
    """
    abstract class for movement patterns / mechanisms
    """
    __metaclass__ = ABCMeta

    def __init__(self,  buffer, frames=None, moving=True, static=True,
                 maxvel=1.0, minvel=0.1):
        """
        :param buffer: SoBuffer
        :param frames: gradient frames to be considered
        :param moving: include moving gradients in calculations
        :param static: include static gradients in calculations
        :param maxvel: maximum velocity / length of movement vector
        :param minvel: minimum velocity / length of movement vector
        """
        self._buffer = buffer
        self.max_velocity = maxvel

        self.frames = frames

        # consider moving, static or both gradient types
        self.moving = moving
        self.static = static

        self.maxvel = maxvel
        self.minvel = minvel

    @abstractmethod
    def move(self):
        """
        method calculates movement vector based on a list of gradients
        determined by buffer
        :return: movement vector (Vector3)
        """
        pass

    def get_pos(self):
        """
        :return: current position of robot
        """
        return self._buffer.get_own_pose()


class DecisionPattern(object):
    """
    abstract class for decision patterns / mechanisms
    """
    __metaclass__ = ABCMeta

    def __init__(self, buffer, frame=None, key=None, value=None, state=None,
                 moving=True, static=False, goal_radius=0, ev_factor=1.0,
                 ev_time=0.0, diffusion=20, attraction=0, requres_pos = True):
        """
        initialization of decision patterns
        :param buffer: SoBuffer
        :param frame: frame that is used for the SoMessage describing the result of the decision
        :param key: payload key with values required in decision pattern
        :param value: initial value for decision pattern
        :param state: initial state for decision pattern
        :param moving: consider moving gradients
        :param static: consider static gradients
        :param goal_radius: goal radius of gradient to be spread (spread())
        :param ev_factor: evaporation factor of gradient to be spread
        :param ev_time: evaporation time of gradient to be spread
        :param diffusion: diffusion radius of gradient to be spread
        :param attraction: attraction value of gradient to be spread
        """
        self._requires_pos = requres_pos
        self._broadcaster = SoBroadcaster()

        self._buffer = buffer
        # frame to consider in decision
        self.frame = frame
        # payload keys to consider
        self.key = key
        self.moving = moving
        self.static = static

        # current used value
        self.value = value

        # current & last in behaviour used state
        self.state = state

        # message
        self.goal_radius = goal_radius
        self.ev_factor = ev_factor
        self.ev_time = ev_time
        self.attraction = attraction
        self.diffusion = diffusion

    @abstractmethod
    def calc_value(self):
        """
        method to determine current key-value value and to set state
        :return: [value, state]
        """
        pass

    def spread(self):
        """
        method to spread calculated values (determined by calc_value)
        :return:
        """

        if not self._requires_pos or self.get_pos():
            # set value and state
            val = self.calc_value()
            self.value = val[0]
            self.state = val[1]

            if val:
                if self.frame is not None:
                    # create gossip message
                    msg = self.create_message(val)

                    # spread gradient (only if data is available)
                    self.send_message(msg)

            else:
                rospy.logerr("DecisionPattern(%s):: Spread not possible. Calculation failed ...", str(type(self)))

        else:
            rospy.logerr("DecisionPattern(%s):: Spread not possible. Own position unknown ...",  str(type(self)))

    def send_message(self, msg):
        self._broadcaster.send_data(msg)

    def get_pos(self):
        """
        :return: Header
        """
        return self._buffer.get_own_pose()

    def create_message(self, val):

        msg = SoMessage()
        msg.header.frame_id = self.frame
        msg.parent_frame = self._buffer.id

        now = rospy.Time.now()
        msg.header.stamp = now
        msg.ev_stamp = now

        # important to determine whether gradient is within view
        current_pose = self.get_pos()
        if current_pose == None:
            rospy.logerr("Self organisation stopped. Last position not available")
            return None
        msg.p = current_pose.p
        msg.q = current_pose.q
        msg.attraction = self.attraction

        msg.diffusion = self.diffusion

        msg.goal_radius = self.goal_radius
        msg.ev_factor = self.ev_factor
        msg.ev_time = self.ev_time

        msg.moving = True  # set to moving as gradient is tied to agent

        msg.payload.append(KeyValue(self.key, val[0]))

        return msg
