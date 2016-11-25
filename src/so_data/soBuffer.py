'''
Created on 07.11.2016

@author: kaiser
'''

import rospy
from so_data.msg import soMessage, Vector
import numpy as np

class SoBuffer():
    '''
    This class is the buffer for received self-organization data
    '''
    def __init__(self, aggregation=True, evaporation_factor=0.8, evaporation_time=5, min_diffusion=1.0, view_distance=2.0):
        '''
        :param aggregation: True/False - indicator if aggregation should be applied
        :param evaporation_factor: specifies how fast data evaporates, has to be between [0,1]
                (0 - data is lost after 1 iteration, 1 - data is stored permanently)
        :param evaporation_time: delta time when evaporation should be applied in secs
        :param min_diffusion: threshold, gradients with smaller diffusion radii will be deleted
        '''

        rospy.Subscriber('soData', soMessage, self.store_data)

        self.data = []
        self._current_gradient = Vector()
        self._aggregation = aggregation #aggregation has somehow always to be true, so change that maybe to different options
        self._evaporation_factor = evaporation_factor
        self._evaporation_time = evaporation_time
        self._min_diffusion = min_diffusion
        self._view_distance = view_distance


    def store_data(self, msg):
        '''
        store received soMessage
        :param msg: received gradient (soMessage)
        :return:
        '''

        # Check whether gradient at the same position already exists, if yes keep gradient with bigger diffusion radius

        # flag to indicate whether an element with same position was already found
        found = False

        # Aggregation of data with same content (position), but different diffusion radii
        for element in self.data:
            if element.p.x == msg.p.x and element.p.y == msg.p.y:
                if msg.diffusion >= element.diffusion: #keep data with max diffusion radius
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

    def get_current_gradient(self, pose):
        '''
        :parameter: pose: Pose Message with position of robot
        :return: last received gradient
        '''
        # evaporate & aggregate data
        if self._evaporation_factor != 1.0: # factor of 1.0 means no evaporation
            self.evaporate_buffer()

        if self._aggregation:
            self.aggregate_min(pose)

        return self._current_gradient

    def clear_buffer(self):
        '''
        delete all buffer elements
        :return:
        '''
        self.data.clear()

    def get_gradient_distance(self, gradpos, pose):
        '''
        :param gradpos: pose of the gradient to be investigated (Vector)
        :param pose: pose of the robot (Pose)
        :return: euclidian distance robot to last received gradient
        '''
        return np.linalg.norm([(gradpos.x - pose.x), (gradpos.y - pose.y)])

    # AGGREGATION
    def aggregate_min(self, pose): #TODO: unit test
        '''
        follow higher gradient values (= gradient with shortest relative distance)
        :return:
        '''
        tmp_grad = soMessage()
        gradients = []

        if self.data:
            for element in self.data:
                # check if gradient is within view of robot
                if self.get_gradient_distance(element.p, pose) <= element.diffusion + self._view_distance:
                    gradients.append(element)

        # find gradient with highest value ( = closest relative distance)
        if gradients:
            for gradient in gradients:
                if tmp_grad.diffusion != 0.0 and self.get_gradient_distance(gradient.p, pose)/gradient.diffusion <= \
                        self.get_gradient_distance(tmp_grad.p, pose)/tmp_grad.diffusion:
                    tmp_grad = gradient
                elif tmp_grad.diffusion == 0.0:
                    tmp_grad = gradient

        if tmp_grad.diffusion == 0.0:
            self._current_gradient = Vector()
        else:
            distance = Vector()
            if tmp_grad.attraction == 1:
                distance.x = tmp_grad.attraction * (tmp_grad.p.x - pose.x)
                distance.y = tmp_grad.attraction * (tmp_grad.p.y - pose.y)
            elif tmp_grad.attraction == -1: # similar to repulsion vector calculation in basic mechanisms
                dist = self.get_gradient_distance(tmp_grad.p, pose)
                if dist > 0:
                    distance.x = (tmp_grad.diffusion - dist) * ((pose.x - tmp_grad.p.x) / dist)
                    distance.y = (tmp_grad.diffusion - dist) * ((pose.y - tmp_grad.p.y) / dist)
                elif distance == 0:
                    # create random vector with length = repulsion radius
                    rand = np.random.random_sample()
                    distance.x = (2 * np.random.randint(2) - 1) * rand * tmp_grad.diffusion
                    distance.y = (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * tmp_grad.diffusion

            self._current_gradient = distance


    def aggregate_nearest_repulsion(self, pose): # TODO unit test
        '''
        aggregate nearest attractive gradient with repulsive gradients s.t. robot finds gradient source avoiding the
        repulsive gradient sources
        :param pose:
        :return:
        '''

        gradients = []
        vector_attraction = Vector()
        vector_repulsion = Vector()
        vector_gradient = Vector()

        if self.data:
            for element in self.data:
                #store all elements which are within reach of the robot
                # TODO only gradients which are in a certain view angle of the robot? (Simulation specific?!?)
                if self.get_gradient_distance(element.p, pose) <= element.diffusion + self._view_distance:
                      gradients.append(element)

        if gradients:
            tmp_grad = soMessage()
            for gradient in gradients:
                    # find nearest attractive gradient
                    if gradient.attraction == 1:
                        if tmp_grad.diffusion != 0.0 and \
                                self.get_gradient_distance(gradient.p, pose)/gradient.diffusion < \
                                                self.get_gradient_distance(tmp_grad.p, pose)/tmp_grad.diffusion:
                            vector_attraction.x = gradient.p.x - pose.x
                            vector_attraction.y = gradient.p.y - pose.y
                            tmp_grad = gradient

                    # aggregate repulsive gradients
                    if gradient.attraction == -1:
                        # based on repulsion vector of the paper
                        dist = self.get_gradient_distance(gradient.p, pose)
                        if dist > 0.0 and dist <= gradient.diffusion:
                            #in case that distance is 0, don't add repulsion vector, as moving in direction of attractive gradient will result in leaving repulsive gradient
                            vector_repulsion.x += (gradient.diffusion - dist) * ((pose.x - gradient.p.x) / dist)
                            vector_repulsion.y += (gradient.diffusion - dist) * ((pose.y - gradient.p.y) / dist)
                       # elif dist == 0.0: 
                       #     vector_repulsion.x += (2 * np.random.random_sample() - 1) * gradient.diffusion
                       #     vector_repulsion.y += (2 * np.random.random_sample() - 1) * gradient.diffusion

        # vector addition to combine repulsion and attraction
        vector_gradient.x = vector_attraction.x + vector_repulsion.x
        vector_gradient.y = vector_attraction.y + vector_repulsion.y

        #TODO: vectors in opposite directions (repulsion and attraction) - how to handle this one

        self._current_gradient = vector_gradient


    # EVAPORATION
    def evaporate_buffer(self):
        '''
        evaporate buffer data
        :return:
        '''
        for i in xrange(len(self.data) -1, -1, -1): # go in reverse order
            diff = rospy.Time.now() - self.data[i].stamp

            if diff >= rospy.Duration(self._evaporation_time):
                n = diff.secs // self._evaporation_time
                self.data[i].diffusion *= self._evaporation_factor ** n
                self.data[i].stamp += rospy.Duration(n*self._evaporation_time)

                if self.data[i].diffusion < self._min_diffusion:
                    del self.data[i] # delete element from list