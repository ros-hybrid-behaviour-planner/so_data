"""
Created on 15.02.2017

@author: kaiser

Module including abstract class for patterns
"""
from abc import ABCMeta, abstractmethod
from so_data.sobroadcaster import SoBroadcaster
from diagnostic_msgs.msg import KeyValue
from so_data.msg import SoMessage
import rospy


class MovementPattern(object):
    """
    abstract class for movement patterns / mechanisms
    """

    __metaclass__ = ABCMeta

    def __init__(self,  buffer, frames=None, repulsion=False, moving=True,
                 static=True, maxvel=1.0, minvel=0.1):
        """
        :param buffer: SoBuffer
        :param maxvel: maximum velocity / length of movement vector
        :param moving: include moving gradients in calculations
        :param static: include static gradients in calculations
        """

        self._buffer = buffer
        self.max_velocity = maxvel

        self.frames = frames
        # store current position used for last calculation
        self._current_pos = None

        # apply repulsion
        self.repulsion = repulsion
        # consider moving, static or both gradient types
        self.moving = moving
        self.static = static

        self.maxvel = maxvel
        self.minvel = minvel

    @abstractmethod
    def move(self):
        """
        method calculates movement vector based on Gradient List received
        by buffer
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
    class for decision patterns
    """
    __metaclass__ = ABCMeta

    def __init__(self, buffer, frame=None, key=None, state=None, moving=True,
                 static=False, goal_radius=0, ev_factor=1.0, ev_time=0.0,
                 attraction=-1, diffusion=20):

        self._broadcaster = SoBroadcaster()

        self._buffer = buffer
        # frames to consider in decision
        self.frame = frame
        # payload keys to consider
        self.key = key
        self.moving = moving
        self.static = static

        # remember last calculated value
        self.last_value = -1

        # set state
        self.state = state

        # message
        self.goal_radius = goal_radius
        self.ev_factor = ev_factor
        self.ev_time = ev_time
        self.attraction = attraction
        self.diffusion = diffusion

    @abstractmethod
    def value(self):
        """
        method to determine current key-value value
        :return:
        """
        pass

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

        msg.goal_radius = self.goal_radius
        msg.ev_factor = self.ev_factor
        msg.ev_time = self.ev_time

        msg.moving = True  # set to moving as gradient is tied to agent

        # determine value
        self.last_value = self.value()
        msg.payload.append(KeyValue(self.key, "%.9f" % self.last_value))

        # spread gradient
        self._broadcaster.send_data(msg)

    def get_pos(self):
        """
        :return: current position of robot
        """
        return self._buffer.get_own_pose()
