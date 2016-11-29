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

        self.aggregate_nearest_repulsion(pose)

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
        sets current gradient to direction vector (length <= 1)
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
                distance.x = tmp_grad.attraction * (tmp_grad.p.x - pose.x) / tmp_grad.diffusion
                distance.y = tmp_grad.attraction * (tmp_grad.p.y - pose.y) / tmp_grad.diffusion
            elif tmp_grad.attraction == -1: # similar to repulsion vector calculation in basic mechanisms
                dist = self.get_gradient_distance(tmp_grad.p, pose)
                if dist > 0:
                    distance.x = (tmp_grad.diffusion - dist) * ((pose.x - tmp_grad.p.x) / dist) / tmp_grad.diffusion
                    distance.y = (tmp_grad.diffusion - dist) * ((pose.y - tmp_grad.p.y) / dist) / tmp_grad.diffusion
                elif distance == 0:
                    # create random vector with length = repulsion radius
                    rand = np.random.random_sample()
                    distance.x = ((2 * np.random.randint(2) - 1) * rand * tmp_grad.diffusion) / tmp_grad.diffusion
                    distance.y = (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * tmp_grad.diffusion / tmp_grad.diffusion

            self._current_gradient = distance



    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        """ Returns the directed angle in radians between vectors 'v1' and 'v2' - only working in 2D!::

                >>> angle_between((1, 0), (0, 1))
                1.5707963267948966
                >>> angle_between((0, 1), (1, 0))
                -1.5707963267948966
                >>> angle_between((1, 0), (1, 0))
                0.0
                >>> angle_between((1, 0), (-1, 0))
                3.141592653589793
        """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        # det needs square matrix as input!
        angle = np.math.atan2(np.linalg.det([v1_u, v2_u]), np.dot(v1_u, v2_u))

        if np.isnan(angle):
            return 0.0

        return angle

    def aggregate_nearest_repulsion(self, pose): # TODO unit test
        '''
        aggregate nearest attractive gradient with repulsive gradients s.t. robot finds gradient source avoiding the
        repulsive gradient sources
        :param pose:
        :return:
        '''

        gradients_attractive = []
        gradients_repulsive = []
        vector_attraction = Vector()
        vector_repulsion = Vector()

        if self.data:
            for element in self.data:
                #store all elements which are within reach of the robot
                if self.get_gradient_distance(element.p, pose) <= element.diffusion + self._view_distance:
                    if element.attraction == 1:
                        gradients_attractive.append(element)
                    elif element.attraction == -1:
                        gradients_repulsive.append(element)

        if gradients_attractive:
            tmp_grad = soMessage()
            for gradient in gradients_attractive:
                    # find nearest attractive gradient
                    if tmp_grad.diffusion != 0.0 and \
                            self.get_gradient_distance(gradient.p, pose)/gradient.diffusion < \
                                             self.get_gradient_distance(tmp_grad.p, pose)/tmp_grad.diffusion:
                        vector_attraction.x = (gradient.p.x - pose.x) / gradient.diffusion
                        vector_attraction.y = (gradient.p.y - pose.y) / gradient.diffusion
                        tmp_grad = gradient
                    elif tmp_grad.diffusion == 0.0:
                        vector_attraction.x = (gradient.p.x - pose.x) / gradient.diffusion
                        vector_attraction.y = (gradient.p.y - pose.y) / gradient.diffusion
                        tmp_grad = gradient

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                count = 0
                # based on repulsion vector of the paper
                dist = self.get_gradient_distance(gradient.p, pose)
                if 0.0 < dist <= gradient.diffusion:

                    tmp = Vector()
                    tmp.x = (gradient.diffusion - dist) * ((pose.x - gradient.p.x) / dist)
                    tmp.y = (gradient.diffusion - dist) * ((pose.y - gradient.p.y) / dist)

                    if np.pi - np.pi/6 <= self.angle_between([vector_attraction.x, vector_attraction.y],
                                             [tmp.x, tmp.y]) <= np.pi + np.pi/6:
                        angle = np.arccos(dist/gradient.diffusion)
                        tmp.x += (pose.x - gradient.p.x)
                        tmp.y += (pose.y - gradient.p.y)

                        vector_repulsion.x += (tmp.x * np.cos(angle) - tmp.y * np.sin(angle)) / gradient.diffusion
                        vector_repulsion.y += (tmp.x * np.sin(angle) + tmp.y * np.cos(angle)) / gradient.diffusion
                    elif np.linalg.norm([tmp.x, tmp.y]) == 0.0:
                        angle = np.arccos(dist / gradient.diffusion)
                        tmp.x += (pose.x - gradient.p.x)
                        tmp.y += (pose.y - gradient.p.y)

                        vector_repulsion.x += (tmp.x * np.cos(angle) - tmp.y * np.sin(angle)) / gradient.diffusion
                        vector_repulsion.y += (tmp.x * np.sin(angle) + tmp.y * np.cos(angle)) / gradient.diffusion
                    else:
                        vector_repulsion.x += tmp.x / gradient.diffusion
                        vector_repulsion.y += tmp.y / gradient.diffusion
                    count += 1

                # try to move s.t. repulsive gradient is avoided completely when nearby repulsive gradient is sensed
                elif dist > gradient.diffusion: #TODO: only if gradient is in same direction as attractive goal
                    deltax = gradient.p.x - pose.x
                    deltay = gradient.p.y - pose.y

                    angle = np.arccos(dist/gradient.diffusion)

                    if np.absolute(pose.theta) < angle:
                        if self.angle_between([deltax, deltay], [vector_attraction.x, vector_attraction.y]) < 0.0:
                            angle *= -1
                        vector_repulsion.x += (deltax * np.cos(angle) - deltay * np.sin(angle)) / dist
                        vector_repulsion.y += (deltax * np.sin(angle) + deltay * np.cos(angle)) / dist

                        count += 1

            # ensure repulsive gradient length [0, 1]
            if count > 0:
                vector_repulsion.x /= count
                vector_repulsion.y /= count

            # only one repulsive gradient with dist robot - gradient == 0, add random vector
            if count == 0 and tmp_grad.diffusion == 0.0:
                vector_repulsion.x += (2 * np.random.random_sample() - 1) * gradient.diffusion / gradient.diffusion
                vector_repulsion.y += (2 * np.random.random_sample() - 1) * gradient.diffusion / gradient.diffusion


            # vector addition to combine repulsion and attraction
        # f np.linalg.norm([vector_attraction.x, vector_attraction.y]) > 1 - np.linalg.norm([vector_repulsion.x, vector_repulsion.y]):

        vector_attraction.x += vector_repulsion.x
        vector_attraction.y += vector_repulsion.y

        self._current_gradient = vector_attraction


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