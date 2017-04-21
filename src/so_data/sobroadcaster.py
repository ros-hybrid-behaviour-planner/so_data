"""
Created on 03.11.2016

@author: kaiser

Module allows to broadcast SoMessage data to topic so_data
"""

import rospy 
from so_data.msg import SoMessage


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
        self._pub = rospy.Publisher('so_data', SoMessage, queue_size=10,
                                    latch=True)

    def send_data(self, message):
        """
        method to spread gradient data
        :param message: msg or list of messages to be send
        :return:
        """
        if isinstance(message, SoMessage):
            self._pub.publish(message)
        elif isinstance(message, list):
            for m in message:
                if isinstance(m, SoMessage):
                    self._pub.publish(m)
                else:
                    rospy.logwarn("SoBroadcaster: Wrong message type")

        else:
            rospy.logwarn("SoBroadcaster: Wrong message type")
