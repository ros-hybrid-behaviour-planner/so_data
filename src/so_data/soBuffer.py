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
    def __init__(self, duration, pose_sensor, aggregation = True, evaporation_factor = 0.8, evaporation_time = 5):
        '''
        :param duration: how long data is kept in buffer
        '''

        rospy.Subscriber(pose_sensor, Pose, self.pose_callback)

        self._duration = duration
        #store data - use of deque to be able to add/delete from both ends
        self.data = deque([])
        self._current_gradient = soMessage()

        self._current_pose = Pose()
        self._aggregation = aggregation
        self._evaporation_factor = evaporation_factor #evaporation factor between [0, 1] - 0 means data is lost after 1 iteration, 1 means data is permanent
        self._evaporation_time = evaporation_time # delta time when evaporation is applied
        self._last_evaporation = rospy.Time.now() # last time evaporation was applied

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

        # delete outdated gradients
        if self.get_gradient_distance(self._current_gradient.p) >= self._current_gradient.diffusion:  # only consider gradients within diffusion radius
            self._current_gradient = soMessage()

        # remember all gradients within diffusion radius
        if self.get_gradient_distance(msg.p) <= msg.diffusion: #only consider gradients within diffusion radius   #or self.data[-1].stamp < msg.stamp:
            self.data.append(msg)

        # delete all outdated data if data is not stored permanently
        #if not self._permanent:
        #    self.prune_buffer()

    def get_data(self):
        '''
        :return: buffer content
        '''
        return self.data

    def get_current_gradient(self):
        '''
        :return: last received gradient
        '''

        # evaporate & aggregate data

        self.evaporation()

        if self._aggregation:
            self.aggregate_min()

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


    def aggregate_min(self):
        '''
        keep only closest gradient information / direction
        :return:
        '''
        if self.data:
            for element in self.data:
                #if self._current_gradient.diffusion != 0 and \
                #            self.get_gradient_distance(element.p) < self.get_gradient_distance(self._current_gradient.p):
                #        self._current_gradient = element

                if self._current_gradient.diffusion < element.diffusion and \
                            self.get_gradient_distance(element.p) <= self.get_gradient_distance(self._current_gradient.p):
                        self._current_gradient = element

                elif self._current_gradient.diffusion == 0.0: #no diffusion radius == no gradient
                    self._current_gradient = element

            self.data.clear()




    def get_gradient_distance(self, gradpos):
        '''
        :param gradient pose
        :return: euclidian distance robot to last received gradient
        '''
        return np.linalg.norm([(gradpos.x - self._current_pose.x), (gradpos.y - self._current_pose.y)])

    def evaporation(self):
        '''
        evaporate current stored gradient
        :return:
        '''
        diff = rospy.Time.now() - self._last_evaporation
        if diff > rospy.Duration(self._evaporation_time):
            #apply evaporation
            n = diff.secs // self._evaporation_time # no remainders
            self._current_gradient.diffusion *= self._evaporation_factor ** n

            #  check if robot left diffusion radius
            if self.get_gradient_distance(
                    self._current_gradient.p) >= self._current_gradient.diffusion:
                self._current_gradient = soMessage() #reset

            self._last_evaporation += rospy.Duration(n*self._evaporation_time) #rospy.Time.now()






# def aggregate_data(self): #does not work like this in the whole setting maybe CHECK
#    '''
#    aggregation of data - keep info with closest source / both for repulsion and attraction
#    :param pose: current position of robot
#    :return:
#    '''
#    if self.data:
#        closest_attractive = soMessage()
#        closest_repulsive = soMessage()
#        for element in self.data:
#            if element.direction == 1.0:
#                if closest_attractive.direction == 0.0: #first element to be considered
#                    closest_attractive = element
#                    distance_attractive = self.get_gradient_distance(closest_attractive.p)
#                elif self.get_gradient_distance(element.p) < distance_attractive:
#                    closest_attractive = element
#                    distance_attractive = self.get_gradient_distance(closest_attractive.p)
#            if element.direction == -1.0:
#                if closest_repulsive.direction == 0.0:
#                    closest_repulsive = element
#                    distance_repulsive = self.get_gradient_distance(closest_repulsive.p)
#                elif self.get_gradient_distance(element.p) < distance_repulsive:
#                    closest_repulsive = element
#                    distance_repulsive = self.get_gradient_distance(closest_repulsive.p)
#        self.data.clear()

#        #keep only aggregated elements
#        if closest_attractive.direction == 1.0:
#            self.data.append(closest_attractive)
#        if closest_repulsive.direction == -1.0:
#            self.data.append(closest_repulsive)
