"""
Created on 03.11.2016

@author: kaiser, hrabia

Module allows to broadcast SoMessage data to topic so_data
"""

import rospy 
from so_data.msg import SoMessage
from so_data.sobuffer import serialize_msg_payload
from copy import deepcopy


class SoBroadcaster(object):
    """
    This class is a broadcaster to send data from one robot to the gradient
    topic so_data
    """
    def __init__(self):
        """
        Constructor
        Creates publisher to broadcast data to so_data as a soMessage
        """
        self._pub = self.init_publisher()

    def init_publisher(self):
        return rospy.Publisher('so_data', SoMessage, queue_size=10, latch=True)

    def send_data(self, message, copy_message=False):
        """
        method to spread gradient data
        :param message: msg or list of messages to be send
        :param copy_message: set to True to avoid that the original object is changed
        """
        if copy_message:
            message = deepcopy(message)

        if isinstance(message, SoMessage):
            m = serialize_msg_payload(message)
            self._pub.publish(m)
        elif isinstance(message, list):
            for m in message:
                if isinstance(m, SoMessage):
                    m = serialize_msg_payload(m)
                    self._pub.publish(m)
                else:
                    rospy.logwarn("SoBroadcaster: Wrong message type")

        else:
            rospy.logwarn("SoBroadcaster: Wrong message type")
