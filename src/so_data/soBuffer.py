'''
Created on 07.11.2016

@author: kaiser
'''

import rospy
from collections import deque

class SoBuffer():
    '''
    This class is the buffer for received self-organization data
    '''
    def __init__(self, duration, permanent = False):
        '''
        :param duration: how long data is kept in buffer
        '''
        self._duration = duration
        #store data - use of deque to be able to add/delete from both ends
        self.data = deque([])
        self.permanent = permanent

    def store_data(self, msg):
        '''
        store received soMessage
        :param msg: soMessage
        :return:
        '''
        if not self.data or self.data[-1].stamp < msg.stamp:
            self.data.append(msg)

        # delete all outdated data if data is not stored permanently
        if not self.permanent:
            self.prune_buffer()

    def get_data(self):
        return self.data

    def get_last_gradient(self):
        '''
        :return: last received gradient
        '''
        if self.data:
            if self.permanent:
                return self.data[-1]
            elif rospy.Time.now() - self.data[0].stamp < rospy.Duration(self._duration):
                return self.data[-1]

    def prune_buffer(self):
        '''
        remove outdated data from buffer
        '''
        while self.data and rospy.Time.now() - self.data[0].stamp > rospy.Duration(self._duration):
            self.data.popleft()

    def aggregate_data(self):
        '''
        aggregation of data - keep closest information
        :return:
        '''
        return self.data

    def clear_buffer(self):
        '''
        delete all buffer elements
        :return:
        '''
        self.data.clear()