'''
Created on 07.11.2016

@author: kaiser
'''

import rospy
from collections import deque
from so_data.msg import Pose, soMessage
import numpy as np


class SoBuffer():
    '''
    This class is the buffer for received self-organization data
    '''
    def __init__(self, duration, pose_sensor = None, permanent = True, aggregation = True):
        '''
        :param duration: how long data is kept in buffer
        '''

        if pose_sensor:
            rospy.Subscriber(pose_sensor, Pose, self.pose_callback)

        self._duration = duration
        #store data - use of deque to be able to add/delete from both ends
        self.data = deque([])
        self._permanent = permanent
        self._current_pose = Pose()
        self._aggregation = aggregation
        self._current_gradient = None #soMessage()

    def pose_callback(self, pose):
        '''
        Callback for storing the updated position of the robot
        '''
        self._current_pose = pose

    def store_data(self, msg):
        '''
        store received soMessage
        :param msg: soMessage
        :return:
        '''

        if self.get_gradient_distance(msg.p) <= msg.diffusion: #only consider gradients within diffusion radius   #or self.data[-1].stamp < msg.stamp:
            self.data.append(msg)

        # delete all outdated data if data is not stored permanently
        if not self._permanent:
            self.prune_buffer()

        if self._aggregation:
            self.aggregate_min()

    def get_data(self):
        '''
        :return: buffer content
        '''
        return self.data

    def get_current_gradient(self):
        '''
        :return: last received gradient
        '''

        return self._current_gradient

        #if self.data:
        #    if self._permanent:
        #        return self.data[-1]
        #    elif rospy.Time.now() - self.data[-1].stamp < rospy.Duration(self._duration):
        #        return self.data[-1]


    def prune_buffer(self):
        '''
        remove outdated data from buffer
        '''
        while self.data and rospy.Time.now() - self.data[0].stamp > rospy.Duration(self._duration):
            self.data.popleft()


    def clear_buffer(self):
        '''
        delete all buffer elements
        :return:
        '''
        self.data.clear()

    def aggregate_data(self): #does not work like this in the whole setting maybe CHECK
        '''
        aggregation of data - keep info with closest source / both for repulsion and attraction
        :param pose: current position of robot
        :return:
        '''
        if self.data:
            closest_attractive = soMessage()
            closest_repulsive = soMessage()
            for element in self.data:
                if element.direction == 1.0:
                    if closest_attractive.direction == 0.0: #first element to be considered
                        closest_attractive = element
                        distance_attractive = self.get_gradient_distance(closest_attractive.p)
                    elif self.get_gradient_distance(element.p) < distance_attractive:
                        closest_attractive = element
                        distance_attractive = self.get_gradient_distance(closest_attractive.p)
                if element.direction == -1.0:
                    if closest_repulsive.direction == 0.0:
                        closest_repulsive = element
                        distance_repulsive = self.get_gradient_distance(closest_repulsive.p)
                    elif self.get_gradient_distance(element.p) < distance_repulsive:
                        closest_repulsive = element
                        distance_repulsive = self.get_gradient_distance(closest_repulsive.p)
            self.data.clear()

            #keep only aggregated elements
            if closest_attractive.direction == 1.0:
                self.data.append(closest_attractive)
            if closest_repulsive.direction == -1.0:
                self.data.append(closest_repulsive)


    def aggregate_min(self):
        '''
        keep only closest gradient information / direction
        :return:
        '''
        if self.data:
            for element in self.data:
                if self._current_gradient  and \
                            self.get_gradient_distance(element.p) < self.get_gradient_distance(self._current_gradient.p):
                        self._current_gradient = element
                elif not self._current_gradient:
                    self._current_gradient = element
            self.data.clear()




    def get_gradient_distance(self, gradpos):
        '''
        :param gradient pose
        :return: euclidian distance robot to last received gradient
        '''
        return np.linalg.norm([(gradpos.x - self._current_pose.x), (gradpos.y - self._current_pose.y)])
