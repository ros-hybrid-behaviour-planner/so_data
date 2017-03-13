"""
Created on 03.11.2016

@author: kaiser

Module allows to broadcast SoMessage data to topic so_data
"""

import rospy 
from so_data.msg import * 


class SoBroadcaster():
    """
    This class is the broadcaster to send data from one robot to the data space
    """
    def __init__(self):
        """
        Constructor
        Creates publisher to broadcast data to soData as an soMessage 
        """
        self._pub = rospy.Publisher('so_data', SoMessage, queue_size=1,
                                    latch=True)

    def send_data(self, message):
        """
        :param message: msg to be send or list of messages
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
