"""
Created on 11.01.2017

@author: kaiser

Module contains abstract base class to transform data of a topic to
SoMessages
"""

import rospy
import copy
from abc import ABCMeta, abstractmethod
from utils.ros_helpers import get_topic_type
from geometry_msgs.msg import Vector3, Quaternion
from so_data.sobroadcaster import SoBroadcaster
from so_data.msg import SoMessage


class TopicGradientTf(object):
    """
    abstract base class to transform values of a topic to a soMessage
    """
    __metaclass__ = ABCMeta

    def __init__(self, topic, frame, id, message_type=None, p=Vector3(),
                 attraction=-1, diffusion=1.0, goal_radius=0.5, ev_factor=1.0,
                 ev_time=0, quaternion=Quaternion(),
                 moving=True, payload=None, direction=Vector3(1, 0, 0)):
        """
        subscription to topic and variable initialization
        :param topic: topic to subscribe to
        :param frame: header frame id used in SoMessage
        :param id: soMessage id used for parent frame
        :param message_type: message type of topic
        :param p: soMessage pose
        :param attraction: soMessage attraction (1) or repulsion (-1)
        :param diffusion: soMessage diffusion radius
        :param goal_radius: soMessage goal radius
        :param ev_factor: soMessage evaporation factor
        :param ev_time: soMessage evaporation (delta) time
        :param quaternion: soMessage orientation
        :param direction: soMessage initial heading direction
        :param moving: soMessage boolean to specify static / moving gradient
        :param payload: soMessage payload data
        """

        # broadcaster for transformed data
        self._broadcast = self.init_so_broadcaster()
        self._current_msg = SoMessage()

        self._id = id
        self._frame = frame
        self.p = p
        self.q = quaternion
        self.attraction = attraction
        self.diffusion = diffusion
        self.goal_radius = goal_radius
        self.ev_factor = ev_factor
        self.ev_time = ev_time
        self.direction = direction
        self.moving = moving
        if payload is None:
            payload = []
        self.payload = payload

        if message_type is None:
            message_type = get_topic_type(topic)
        if message_type is not None:
            self._sub = rospy.Subscriber(topic, message_type, self.callback)
        else:
            rospy.logerr("Could not determine message type of: " + topic)

    def init_so_broadcaster(self):
        return SoBroadcaster()

    @abstractmethod
    def callback(self, topic_message):
        """
        buffer received message
        abstract - usually received message has to be assigned to some part of
        soMessage

        abstract method: should assign received data to at least one
        soMessage attribute, call self._broadcast.send_data(self.msg) for
        sending gradient with frequency of subscribed topic

        :return:
        """
        pass

        # create message
        msg = copy.deepcopy(self.create_msg())

        # To sth with topic_message
        # TODO: implement

        # set current message
        self._current_msg = msg

        # optional: send data here or in ROS Node loop
        self.send()

    def create_msg(self):
        """
        creates soMessage with set parameters
        :return: gradient message / SoMessage
        """
        msg = SoMessage()

        # current time
        now = rospy.Time.now()

        msg.header.frame_id = self._frame
        msg.parent_frame = self._id
        msg.header.stamp = now
        msg.p = self.p
        msg.q = self.q
        msg.attraction = self.attraction
        msg.diffusion = self.diffusion
        msg.goal_radius = self.goal_radius
        msg.ev_factor = self.ev_factor
        msg.ev_time = self.ev_time
        msg.ev_stamp = now
        msg.direction = self.direction
        msg.moving = self.moving
        msg.payload = self.payload

        return msg

    def send(self):
        """
        spread current gradient
        :return:
        """
        self._broadcast.send_data(self._current_msg)
