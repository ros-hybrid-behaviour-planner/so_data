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
    def __init__(self, pose_sensor, aggregation=True, evaporation_factor=0.8, evaporation_time=5, min_diffusion=1.0):
        '''
        :param pose_sensor: topic name of the pose sensor
        :param aggregation: True/False - indicator if aggregation should be applied
        :param evaporation_factor: specifies how fast data evaporates, has to be between [0,1]
                (0 - data is lost after 1 iteration, 1 - data is stored permanently)
        :param evaporation_time: delta time when evaporation should be applied in secs
        :param min_diffusion: threshold, gradients with smaller diffusion radii will be deleted
        '''
        rospy.Subscriber(pose_sensor, Pose, self.pose_callback)

        self.data = []
        self._current_gradient = soMessage()
        self._current_pose = Pose()
        self._aggregation = aggregation
        self._evaporation_factor = evaporation_factor
        self._evaporation_time = evaporation_time
        self._min_diffusion = min_diffusion

    def pose_callback(self, pose):
        '''
        Callback for storing the updated position of the robot
        '''
        self._current_pose = pose

    def store_data(self, msg):
        '''
        store received soMessage
        :param msg: received gradient (soMessage)
        :return:
        '''

        # Check whether gradient at the same position already exists, if yes keep gradient with bigger diffusion radius

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

        # delete gradient when robot left it's diffusion radius
        if self._current_gradient.diffusion != 0 and self.get_gradient_distance(self._current_gradient.p) > self._current_gradient.diffusion:
            self._current_gradient = soMessage()

        # evaporate & aggregate data
        if self._evaporation_factor != 1.0: # factor of 1.0 means no evaporation
            self.evaporate_gradient()
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

    def get_gradient_distance(self, gradpos):
        '''
        :param gradient pose
        :return: euclidian distance robot to last received gradient
        '''
        return np.linalg.norm([(gradpos.x - self._current_pose.x), (gradpos.y - self._current_pose.y)])

    # AGGREGATION
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


    # EVAPORATION
    def evaporate_gradient(self):
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
                    self._current_gradient.p) > self._current_gradient.diffusion:
                self._current_gradient = soMessage() #reset

            # gradient smaller than minimum diffusion radius
            if self._current_gradient.diffusion < self._min_diffusion:
                self._current_gradient = soMessage()

    def evaporate_buffer(self):
        '''
        evaporate buffer data
        :return:
        '''
        for i in xrange(len(self.data) -1, -1, -1): # go in reverse order
            diff = rospy.Time.now() - self.data[i].stamp

            if diff >= rospy.Duration(self._evaporation_factor):
                n = diff.secs // self._evaporation_time
                self.data[i].diffusion *= self._evaporation_factor ** n
                self.data[i].stamp += rospy.Duration(n*self._evaporation_time)

                if self.data[i].diffusion < self._min_diffusion:
                    del self.data[i] # delete element from list