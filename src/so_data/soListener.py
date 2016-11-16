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
    This class is the listener to request and receive data from
    the soData topic 
    '''
    def __init__(self, pose_sensor):
        '''
        Constructor
        Creates subscriber to receive data from soData 
        '''
        self.buffer = soBuffer.SoBuffer(2.0, pose_sensor)
        self._sub = rospy.Subscriber("soData", soMessage, self.callback)

    def callback(self, msg):
        '''
        :param data: message received from soData topic
        :return:
        '''
        #rospy.loginfo(msg)
        self.buffer.store_data(msg)

    def print_data(self):
        rospy.loginfo(self.buffer.get_data())

    def get_gradient_distance(self, pose):
        '''
        :param pose: current position of robot
        :return: x- and y-distance from robot to last received gradient multiplied with info (repulsive/attractive)
        '''
        self._gradpos = self.buffer.get_current_gradient()
        if self._gradpos:
            distance = [self._gradpos.direction * (self._gradpos.p.x - pose.x), self._gradpos.direction * (self._gradpos.p.y - pose.y)]
            return distance
        else:
            return [0.0, 0.0]

