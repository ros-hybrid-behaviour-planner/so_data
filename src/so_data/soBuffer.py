""" Created on 07.11.2016
.. module:: soBuffer
.. moduleauthor:: kaiser
"""

import rospy
from so_data.msg import soMessage
import numpy as np
import calc
from geometry_msgs.msg import Vector3



class SoBuffer():
    """
    This class is the buffer for received self-organization data
    """
    def __init__(self, aggregation=True, evaporation_factor=1.0, evaporation_time=5, min_diffusion=1.0,
                 view_distance=2.0, id='', result='all', collision_avoidance='repulsion'):
        """
        :param aggregation: indicator if aggregation should be applied
        :type aggregation: bool
        :param evaporation_factor: specifies how fast data evaporates, has to be between [0,1]
                (0 - data is lost after 1 iteration, 1 - data is stored permanently)
        :type: evaporation_factor: float [0,1]
        :param evaporation_time: delta time when evaporation should be applied in secs
        :type evaporation_time: int
        :param min_diffusion: threshold, gradients with smaller diffusion radii will be deleted
        :type min_diffusion: float
        :param result: specifies vector which should be returned;
                options: * all = return vector considering all vectors of potential field
                         * max = return vector with max attraction / repulsion
                         * near = return vector to nearest attractive vector avoiding obstacles
        :type result: str.
        :param collision_avoidance
                options: * gradient = gradient / potential field approach to realize collision avoidance between neighbors
                         * repulsion = repulsion vector calculation based on Fernandez-Marquez et al.
        :type collision_avoidance: str.
        """

        rospy.Subscriber('soData', soMessage, self.store_data)
        self.data = [] # store incoming
        self.own_pos = [] # store own last positions
        self.neighbors = {} # empty dict
        self._current_gradient = Vector3()
        self._aggregation = aggregation #aggregation has somehow always to be true, so change that maybe to different options
        self._evaporation_factor = evaporation_factor
        self._evaporation_time = evaporation_time
        self._min_diffusion = min_diffusion
        self._view_distance = view_distance
        self._collision_avoidance = collision_avoidance
        self._id = id
        self._result = result

    def store_data(self, msg):
        """
        store received soMessage using evaporation and aggregation
        :param msg: received gradient (soMessage)
        :return:
        """

        # Check whether gradient at the same position already exists, if yes keep gradient with bigger diffusion radius

        # evaporate & aggregate data
        if self._evaporation_factor != 1.0: # factor of 1.0 means no evaporation
            self.evaporate_buffer()

        # store own position data (last two values)
        if msg.header.frame_id == self._id:
            self.own_pos.append(msg)
            if len(self.own_pos) > 2:
                del self.own_pos[0]
        elif msg.header.frame_id:
            #Aggregation of data with same content (position), but different diffusion radii
            if msg.header.frame_id[:5] == 'robot': # store neighbors as key-value-pair
                if msg.header.frame_id in self.neighbors:
                    #check if data is newer
                    if msg.header.stamp > self.neighbors[msg.header.frame_id][-1].header.stamp:
                        self.neighbors[msg.header.frame_id].append(msg)
                    if len(self.neighbors[msg.header.frame_id]) > 2:
                        del self.neighbors[msg.header.frame_id][0]
                else:
                    self.neighbors[msg.header.frame_id] = [msg]

        # ToDo Add check that received data is newer than stored data
        elif not msg.header.frame_id:
            found = False
            if self.data:
                for i in xrange(len(self.data) - 1, -1, -1):
                    if self.data[i].p.x == msg.p.x and self.data[i].p.y == msg.p.y:
                        found = True
                        if msg.diffusion >= self.data[i].diffusion: #keep data with max diffusion radius
                            del self.data[i]
                            self.data.append(msg)
                        found = True
            else:
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
        :return: current gradient vector to follow based on settings
        '''
        # evaporate & aggregate data
        #if self._evaporation_factor != 1.0: # factor of 1.0 means no evaporation
        #    self.evaporate_buffer()

        # vector based on gradients
        if self._result == 'near':
            self.aggregate_nearest_repulsion(pose)
        elif self._result == 'max':
            self.aggregate_max(pose)
        elif self._result == 'all':
            self.aggregate_all(pose)

        # Collision Avoidance between neighbors
        if self._collision_avoidance == 'gradient':
            collision = self.gradient_repulsion(pose)
            self._current_gradient.x += collision.x
            self._current_gradient.y += collision.y
        elif self._collision_avoidance == 'repulsion':
            collision = self.repulsion_vector(pose)
            self._current_gradient.x += collision.x
            self._current_gradient.y += collision.y

        return self._current_gradient


    # Collision avoidance between neighbors
    def gradient_repulsion(self, pose):
        '''
        returns repulsion vector (collision avoidance between neighbors) based on potential field approach
        :param pose:
        :return:
        '''
        repulsion = Vector3()
        if self.neighbors:
            for val in self.neighbors.values():
                # check if neighbor is in sight
                if calc.get_gradient_distance(val[-1].p, pose) <= val[-1].diffusion + val[-1].goal_radius \
                        + self._view_distance:
                    grad = self.calc_repulsive_gradient(val[-1], pose)
                    # two robots are at the same position
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        rand = np.random.random_sample()
                        repulsion.x += (2 * np.random.randint(2) - 1) * rand * (val[-1].goal_radius + val[-1].diffusion)
                        repulsion.y += (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * \
                                   (val[-1].goal_radius + val[-1].diffusion)
                    else:
                        repulsion.x += grad.x
                        repulsion.y += grad.y

        return repulsion

    def repulsion_vector(self, pose):
        """
        return a repulsion vector based on formula presented by Fernandez-Marquez et al.
        :param ownpos (Pose.msg),
         repulsion radius is set to view_distance
        :return repulsion vector
        """
        # initialize vector
        m = Vector3()

        if self.neighbors:
            for val in self.neighbors.values():
                distance = calc.get_gradient_distance(val[-1].p, pose)
                if calc.get_gradient_distance(val[-1].p, pose) < self._view_distance:
                    # only robots within repulsion
                    if distance != 0:
                        diff = self._view_distance - distance
                        m.x += (pose.x - val[-1].p.x) * diff / distance
                        m.y += (pose.y - val[-1].p.y) * diff / distance
                    elif distance == 0:
                        # create random vector with length = repulsion radius
                        rand = np.random.random_sample()
                        m.x += (2 * np.random.randint(2) - 1) * rand * self._view_distance
                        m.y += (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * self._view_distance

        # adjust vector length to be within view Distance
        norm = np.linalg.norm([m.x, m.y])
        if norm > self._view_distance:
            m.x = (val[-1].diffusion + val[-1].goal_radius) * m.x / norm
            m.y = (val[-1].diffusion + val[-1].goal_radius) * m.y / norm

        return m


    # AGGREGATION - build potential field
    def aggregate_max(self, pose): #TODO: unit test
        '''
        follow higher gradient values (= gradient with shortest relative distance)
        sets current gradient to direction vector (length <= 1)
        '''

        gradients = []
        tmp_att = 0
        tmp_grad = Vector3()

        if self.data:
            for element in self.data:
                # check if gradient is within view of robot
                if calc.get_gradient_distance(element.p, pose) <= element.diffusion + element.goal_radius \
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


    def aggregate_nearest_repulsion(self, pose): # TODO unit test!!!
        '''
        aggregate nearest attractive gradient with repulsive gradients s.t. robot finds gradient source avoiding the
        repulsive gradient sources
        :param pose:
        :return:
        '''

        gradients_attractive = []
        gradients_repulsive = []
        vector_attraction = Vector3()
        vector_repulsion = Vector3()
        tmp_att = 0

        if self.data:
            for element in self.data:
                #store all elements which are within reach of the robot
                if calc.get_gradient_distance(element.p, pose) <= element.diffusion + element.goal_radius \
                        + self._view_distance:
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
                    vector_repulsion.y += (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * \
                                              (gradient.goal_radius + gradient.diffusion)
                else:
                    vector_repulsion.x += grad.x
                    vector_repulsion.y += grad.y

        vector_attraction.x += vector_repulsion.x
        vector_attraction.y += vector_repulsion.y

        self._current_gradient = vector_attraction

    def aggregate_all(self, pose):  # TODO unit test!!!
        '''
        aggregate all vectors
        :param pose:
        :return:
        '''

        gradients_attractive = []
        gradients_repulsive = []
        vector_attraction = Vector3()
        vector_repulsion = Vector3()

        if self.data:
            for element in self.data:
                # store all elements which are within reach of the robot
                if calc.get_gradient_distance(element.p, pose) <= element.diffusion + element.goal_radius \
                        + self._view_distance:
                    if element.attraction == 1:
                        gradients_attractive.append(element)
                    elif element.attraction == -1:
                        gradients_repulsive.append(element)

        if gradients_attractive:
            for gradient in gradients_attractive:
                # find nearest attractive gradient
                grad = self.calc_attractive_gradient(gradient, pose)
                vector_attraction.x += grad.x
                vector_attraction.y += grad.y

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                grad = self.calc_repulsive_gradient(gradient, pose)
                # robot position is within obstacle radius, inf can't be handled as direction --> add vector which brings robot to the boarder of the obstacle
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    rand = np.random.random_sample()
                    vector_repulsion.x += (2 * np.random.randint(2) - 1) * rand * (
                    gradient.goal_radius + gradient.diffusion)
                    vector_repulsion.y += (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * (
                    gradient.goal_radius + gradient.diffusion)
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


    # Potential field calculations
    def calc_attractive_gradient(self, gradient, pose):
        '''
        :param gradient: position of the goal
        :param pose: position of the robot
        :return: attractive vector
        '''

        v = Vector3()

        # distance goal - agent
        d = calc.get_gradient_distance(gradient.p, pose)
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
        v = Vector3()

        # distance goal - agent
        d = calc.get_gradient_distance(gradient.p, pose)
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

