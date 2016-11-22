'''
Created on 07.11.2016

@author: kaiser
'''

import rospy
from so_data.msg import Pose, soMessage
import numpy as np

class SoBuffer():
    '''
    This class is the buffer for received self-organization data
    '''
    def __init__(self, pose_sensor, aggregation = True, evaporation_factor = 0.8, evaporation_time = 5):
        '''
        :param duration: how long data is kept in buffer
        '''

        rospy.Subscriber(pose_sensor, Pose, self.pose_callback)

        self.data = []

        self._current_gradient = soMessage()

        self._current_pose = Pose()
        self._aggregation = aggregation
        self._evaporation_factor = evaporation_factor #evaporation factor between [0, 1] - 0 means data is lost after 1 iteration, 1 means data is permanent
        self._evaporation_time = evaporation_time # delta time when evaporation is applied
        self._last_evaporation = rospy.Time.now() # last time evaporation was applied
        self._min_diffusion = 1.0 # gradients with smaller diffusion radii will be deleted

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
        if self._current_gradient.diffusion != 0 and self.get_gradient_distance(self._current_gradient.p) >= self._current_gradient.diffusion:
            self._current_gradient = soMessage()

        # Check whether gradient at the same position exists, if yes keep gradient with bigger diffusion radius #TODO aggregate two gradients at the same position? e.g. combine

        # flag to indicate whether an element with same position was already found
        found = False

        for element in self.data:
            if element.p.x == msg.p.x and element.p.y == msg.p.y:
                if msg.diffusion >= element.diffusion:
                    del self.data[self.data.index(element)]
                    self.data.append(msg)
                found = True

        if not found:
            self.data.append(msg)


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
        if self._evaporation_factor != 1.0: #if factor == 1, the diffusion would stay the same
            self.evaporation()
            self.evaporate_buffer()

        if self._aggregation:
            self.aggregate_min()

        return self._current_gradient



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

                if self._current_gradient.diffusion < element.diffusion and \
                            self.get_gradient_distance(element.p) <= self.get_gradient_distance(self._current_gradient.p):
                        self._current_gradient = element

                elif self._current_gradient.diffusion == 0.0: #no diffusion radius == no gradient
                    self._current_gradient = element

                # in case of problems, try this: avoid interfering actions on data
                #self._current_gradient = deepcopy(self._current_gradient)


    def get_gradient_distance(self, gradpos):
        '''
        :param gradient pose
        :return: euclidian distance robot to last received gradient
        '''
        return np.linalg.norm([(gradpos.x - self._current_pose.x), (gradpos.y - self._current_pose.y)])

    def evaporation(self):
        '''
        evaporate current gradient
        :return:
        '''
        diff = rospy.Time.now() - self._current_gradient.stamp
        if diff >= rospy.Duration(self._evaporation_time):
            n = diff.secs // self._evaporation_time # no remainders

            # current stored gradient
            self._current_gradient.diffusion *= self._evaporation_factor ** n
            self._current_gradient.stamp += rospy.Duration(n*self._evaporation_time)

            #  check if robot left diffusion radius
            if self.get_gradient_distance(
                    self._current_gradient.p) >= self._current_gradient.diffusion:
                self._current_gradient = soMessage() #reset


    def evaporate_buffer(self):
        '''
        evaporate buffer data
        :return:
        '''
        for element in self.data:
            diff = rospy.Time.now() - element.stamp

            if diff >= rospy.Duration(self._evaporation_factor):
                n = diff.secs // self._evaporation_time
                element.diffusion *= self._evaporation_factor ** n
                element.stamp += rospy.Duration(n*self._evaporation_time)

                if element.diffusion < self._min_diffusion:
                    del self.data[self.data.value(element)]