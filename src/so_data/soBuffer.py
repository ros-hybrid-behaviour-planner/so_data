'''
Created on 07.11.2016

@author: kaiser
'''

import rospy
from so_data.msg import soMessage, Vector
import numpy as np
import copy


class SoBuffer():
    '''
    This class is the buffer for received self-organization data
    '''
    def __init__(self, aggregation=True, evaporation_factor=0.8, evaporation_time=5, min_diffusion=1.0,
                 view_distance=2.0, id=''):
        '''
        :param aggregation: True/False - indicator if aggregation should be applied
        :param evaporation_factor: specifies how fast data evaporates, has to be between [0,1]
                (0 - data is lost after 1 iteration, 1 - data is stored permanently)
        :param evaporation_time: delta time when evaporation should be applied in secs
        :param min_diffusion: threshold, gradients with smaller diffusion radii will be deleted
        '''

        rospy.Subscriber('soData', soMessage, self.store_data)

        self.data = []
        self.own_pos = [] # store own last positions
        self._current_gradient = Vector()
        self._aggregation = aggregation #aggregation has somehow always to be true, so change that maybe to different options
        self._evaporation_factor = evaporation_factor
        self._evaporation_time = evaporation_time
        self._min_diffusion = min_diffusion
        self._view_distance = view_distance
        self._id = id

    def store_data(self, msg):
        '''
        store received soMessage using evaporation and aggregation
        :param msg: received gradient (soMessage)
        :return:
        '''

        # Check whether gradient at the same position already exists, if yes keep gradient with bigger diffusion radius

        # evaporate & aggregate data
        if self._evaporation_factor != 1.0: # factor of 1.0 means no evaporation
            self.evaporate_buffer()

        # store own position data (last two values)
        # ToDo Add check that received data is newer than stored data
        if msg.header.frame_id == self._id:
            self.own_pos.append(msg)
            if len(self.own_pos) > 2:
                del self.own_pos[0]
        else:
        # Aggregation of data with same content (position), but different diffusion radii
            for element in self.data:
                if msg.header.frame_id:
                    if element.header.frame_id == msg.header.frame_id:
                        del self.data[self.data.index(element)]
                elif not msg.header.frame_id and element.p.x == msg.p.x and element.p.y == msg.p.y:
                    if msg.diffusion >= element.diffusion: #keep data with max diffusion radius
                        del self.data[self.data.index(element)]
            self.data.append(msg)

    def get_data(self):
        '''
        :return: buffer content
        '''
        return self.data

    def get_current_gradient(self, pose):
        '''
        :parameter: pose: Pose Message with position of robot
        :return: current gradient vector to follow based on settings
        '''
        # evaporate & aggregate data
        #if self._evaporation_factor != 1.0: # factor of 1.0 means no evaporation
        #    self.evaporate_buffer()

        #self.aggregate_nearest_repulsion(pose)
        self.aggregate_min(pose)

        return self._current_gradient

    def get_gradient_distance(self, gradpos, pose):
        '''
        :param gradpos: pose of the gradient to be investigated (Vector)
        :param pose: pose of the robot (Pose)
        :return: euclidian distance robot to last received gradient
        '''
        return np.linalg.norm([(gradpos.x - pose.x), (gradpos.y - pose.y)])

    def calc_attractive_gradient(self, gradient, pose):
        '''
        :param gradient: position of the goal
        :param pose: position of the robot
        :return: attractive vector
        '''

        v = Vector()

        # distance goal - agent
        d = self.get_gradient_distance(gradient.p, pose)
        # angle between agent and goal
        angle = np.math.atan2((gradient.p.y - pose.y), (gradient.p.x - pose.x))

        if d < gradient.goal_radius:
            v.x = 0
            v.y = 0
        elif gradient.goal_radius <= d <= gradient.goal_radius + gradient.diffusion:
            v.x = (d - gradient.goal_radius) * np.cos(angle)
            v.y = (d - gradient.goal_radius) * np.sin(angle)
        elif d > gradient.goal_radius + gradient.diffusion:
            v.x = gradient.diffusion * np.cos(angle)
            v.y = gradient.diffusion * np.sin(angle)

        return v

    def calc_repulsive_gradient(self, gradient, pose):
        '''
        :param gradient: position of the goal
        :param pose: position of the robot
        :return: repulsive vector
        '''
        v = Vector()

        # distance goal - agent
        d = self.get_gradient_distance(gradient.p, pose)
        # angle between agent and goal
        angle = np.math.atan2((gradient.p.y - pose.y), (gradient.p.x - pose.x))

        if d < gradient.goal_radius:
            v.x = -1 * np.sign(np.cos(angle)) * np.inf
            v.y = -1 * np.sign(np.sin(angle)) * np.inf
        elif gradient.goal_radius <= d <= gradient.goal_radius + gradient.diffusion:
            v.x = -1 * (gradient.goal_radius + gradient.diffusion - d) * np.cos(angle)
            v.y = -1 * (gradient.goal_radius + gradient.diffusion - d) * np.sin(angle)
        elif d > gradient.goal_radius + gradient.diffusion:
            v.x = 0
            v.y = 0

        return v


    # AGGREGATION
    def aggregate_min(self, pose): #TODO: unit test
        '''
        follow higher gradient values (= gradient with shortest relative distance)
        sets current gradient to direction vector (length <= 1)
        '''

        gradients = []
        tmp_att = 0
        tmp_grad = Vector()

        if self.data:
            for element in self.data:
                # check if gradient is within view of robot
                if self.get_gradient_distance(element.p, pose) <= element.diffusion + element.goal_radius \
                        + self._view_distance:
                    gradients.append(element)


        # find gradient with highest value ( = closest relative distance)
        if gradients:
            for gradient in gradients:
                if gradient.attraction == 1:
                    grad = self.calc_attractive_gradient(gradient, pose)
                else:
                    grad = self.calc_repulsive_gradient(gradient, pose)

                if grad.x == np.inf or grad.x == -1 * np.inf:
                    att = np.inf
                else:
                    att = np.linalg.norm([grad.x, grad.y])

                if att > tmp_att:
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        rand = np.random.random_sample()
                        tmp_grad.x = (2 * np.random.randint(2) - 1) * rand * (gradient.goal_radius + gradient.diffusion)
                        tmp_grad.y = (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * \
                                     (gradient.goal_radius + gradient.diffusion)
                    else:
                        tmp_grad = grad

                    tmp_att = att

            self._current_gradient = tmp_grad



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


    def aggregate_nearest_repulsion(self, pose): # TODO unit test!!!
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
        tmp_att = 0

        if self.data:
            for element in self.data:
                #store all elements which are within reach of the robot
                if self.get_gradient_distance(element.p, pose) <= element.diffusion + self._view_distance:
                    if element.attraction == 1:
                        gradients_attractive.append(element)
                    elif element.attraction == -1:
                        gradients_repulsive.append(element)

        if gradients_attractive:
            for gradient in gradients_attractive:
                    # find nearest attractive gradient
                    grad = self.calc_attractive_gradient(gradient, pose)
                    att = np.linalg.norm([grad.x, grad.y])
                    if att > tmp_att:
                        vector_attraction.x = grad.x
                        vector_attraction.y = grad.y
                        tmp_att = att


        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                grad = self.calc_repulsive_gradient(gradient, pose)
                # robot position is within obstacle radius, inf can't be handled as direction --> add vector which brings robot to the boarder of the obstacle
                if grad.x == np.inf or grad.x == -1 * np.inf:
                        rand = np.random.random_sample()
                        vector_repulsion.x += (2 * np.random.randint(2) - 1) * rand * (gradient.goal_radius + gradient.diffusion)
                        vector_repulsion.y += (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * (gradient.goal_radius + gradient.diffusion)
                else:
                    vector_repulsion.x += grad.x
                    vector_repulsion.y += grad.y

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
            diff = rospy.Time.now() - self.data[i].header.stamp

            if diff >= rospy.Duration(self._evaporation_time):
                n = diff.secs // self._evaporation_time
                self.data[i].diffusion *= self._evaporation_factor ** n
                self.data[i].header.stamp += rospy.Duration(n*self._evaporation_time)

                if self.data[i].diffusion < self._min_diffusion:
                    del self.data[i] # delete element from list