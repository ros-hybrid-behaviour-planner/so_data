'''
Created on 03.11.2016

@author: kaiser
'''

import rospy 
import soBuffer
import math
from so_data.msg import * 

class SoListener():
    '''
    This class is the liestener to request and receive data from 
    the soData topic 
    '''
    def __init__(self):
        '''
        Constructor
        Creates subscriber to receive data from soData 
        '''
        self.buffer = soBuffer.SoBuffer(2.0)
        self._sub = rospy.Subscriber("soData", soMessage, self.callback)

    def callback(self, msg):
        '''
        :param data: message received from soData topic
        :return:
        '''
        #rospy.loginfo(msg)
        self.buffer.storeData(msg)

    def print_data(self):
        rospy.loginfo(self.buffer.getLastGradient())

    def get_gradient_distance(self, pose):
        self._gradpos = self.buffer.getLastGradient().p
        distance = [(self._gradpos.x - pose.x), (self._gradpos.y - pose.y)]
        return distance