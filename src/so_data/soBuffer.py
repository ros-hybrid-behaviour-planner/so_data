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
    def __init__(self, aggregation='max', evaporation_factor=1.0, evaporation_time=5, min_diffusion=1.0,
                 view_distance=2.0, id='', result='near', collision_avoidance='gradient'):
        """
        :param aggregation: indicator which kind of aggregation should be applied
                options: * min = keep gradients with minimum diffusion radius
                         * max = keep gradients with maximum diffusion radius
                         * avg = combine gradients / average
        :type aggregation: str.
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

        self._data = [] # store incoming
        self._own_pos = [] # store own last positions
        self._neighbors = {} # empty dict
        self._current_gradient = Vector3()

        # options
        if aggregation != 'min' and aggregation != 'max' and aggregation != 'avg' and aggregation != 'newest':
            rospy.logerr("Wrong aggregation type in soBuffer. Set to max.")
            self._aggregation = 'max'
        else:
            self._aggregation = aggregation

        self._evaporation_factor = evaporation_factor
        self._evaporation_time = evaporation_time
        self._min_diffusion = min_diffusion
        self._view_distance = view_distance
        if collision_avoidance != 'repulsion' and collision_avoidance != 'gradient' and collision_avoidance != '':
            rospy.logerr("No valid option for collision avoidance entered. Set to gradient.")
            self._collision_avoidance = 'gradient'
        else:
            self._collision_avoidance = collision_avoidance

        self._id = id

        if result != 'all' and result != 'max' and result != 'near':
            rospy.logerr("Wrong return type in soBuffer. Set to near.")
            self._result = 'near'
        else:
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
            self._evaporate_buffer()

        # store own position data (last two values)
        if self._id and msg.header.frame_id == self._id:
            self._own_pos.append(msg)
            if len(self._own_pos) > 2:
                del self._own_pos[0]
        elif msg.header.frame_id:
            #Aggregation of data with same content (position), but different diffusion radii
            if msg.header.frame_id[:5] == 'robot': # store neighbors as key-value-pair
                if msg.header.frame_id in self._neighbors:
                    #check if data is newer
                    if msg.header.stamp > self._neighbors[msg.header.frame_id][-1].header.stamp:
                        self._neighbors[msg.header.frame_id].append(msg)
                    if len(self._neighbors[msg.header.frame_id]) > 2:
                        del self._neighbors[msg.header.frame_id][0]
                else:
                    self._neighbors[msg.header.frame_id] = [msg]

        # ToDo Add check that received data is newer than stored data
        elif not msg.header.frame_id:
            found = False
            if self._data:
                for i in xrange(len(self._data) - 1, -1, -1):
                    if self._data[i].p.x == msg.p.x and self._data[i].p.y == msg.p.y:
                        found = True
                        if self._aggregation == 'max': # keep data with max diffusion radius
                            if msg.diffusion + msg.goal_radius >= self._data[i].diffusion + self._data[i].goal_radius:
                                del self._data[i]
                                self._data.append(msg)
                        elif self._aggregation == 'min': # keep data with min diffusion radius
                            if msg.diffusion + msg.goal_radius <= self._data[i].diffusion + self._data[i]. goal_radius:
                                del self._data[i]
                                self._data.append(msg)
                        elif self._aggregation == 'avg':
                            # attraction is the same direction
                            if msg.attraction == self._data[i].attraction:
                                msg.diffusion = (msg.diffusion + self._data[i].diffusion) / 2
                                msg.goal_radius = (msg.goal_radius + self._data[i].goal_radius) / 2
                            else:
                                # change sign
                                if self._data[i].diffusion + self._data[i].goal_radius > msg.diffusion + msg.goal_radius:
                                    msg.attraction *= -1
                                msg.diffusion = np.absolute(msg.diffusion - self._data[i].diffusion)
                                msg.goal_radius = np.absolute(msg.goal_radius - self._data[i].goal_radius)
                            del self._data[i]
                            if msg.diffusion >= self._min_diffusion:
                                self._data.append(msg)
                        elif self._aggregation == 'newest': # keep last received gradient at one position
                            if msg.header.stamp >= self._data[i].header.stamp:
                                del self._data[i]
                                self._data.append(msg)
            else:
                self._data.append(msg)
                found = True

            if not found:
                self._data.append(msg)


    def get_data(self):
        """
        :return buffer content
        """
        return self._data

    def get_current_gradient(self, pose):
        """
        :param pose: Pose Message with position of robot (geometry msgs Pose)
        :return current gradient vector to follow based on settings
        """
        # distance vector based on gradients - merges available information
        if self._result == 'near':
            self._aggregate_nearest_repulsion(pose)
        elif self._result == 'max':
            self._aggregate_max(pose)
        elif self._result == 'all':
            self._aggregate_all(pose)

        # Collision Avoidance between neighbors
        if self._collision_avoidance == 'gradient':
            collision = self._gradient_repulsion(pose)
            self._current_gradient.x += collision.x
            self._current_gradient.y += collision.y
        elif self._collision_avoidance == 'repulsion':
            collision = self._repulsion_vector()
            self._current_gradient.x += collision.x
            self._current_gradient.y += collision.y

        return self._current_gradient


    # Collision avoidance between neighbors
    def _gradient_repulsion(self, pose):
        '''
        returns repulsion vector (collision avoidance between neighbors) based on potential field approach
        :param pose:
        :return:
        '''
        repulsion = Vector3()
        if self._neighbors:
            for val in self._neighbors.values():
                # check if neighbor is in sight
                if calc.get_gradient_distance(val[-1].p, pose) <= val[-1].diffusion + val[-1].goal_radius \
                        + self._view_distance:
                    grad = self._calc_repulsive_gradient(val[-1], pose)

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

    def _repulsion_vector(self):
        """
        return a repulsion vector based on formula presented by Fernandez-Marquez et al., use of received gradients (p)
        for calculation
        repulsion radius is set to view_distance
        :return repulsion vector
        """
        # initialize vector
        m = Vector3()

        if self._neighbors and self._own_pos:
            for val in self._neighbors.values():
                distance = calc.get_gradient_distance(val[-1].p, self._own_pos[-1].p)
                if calc.get_gradient_distance(val[-1].p, self._own_pos[-1].p) < self._view_distance:
                    # only robots within repulsion
                    if distance != 0:
                        diff = self._view_distance - distance
                        m.x += (self._own_pos[-1].p.x - val[-1].p.x) * diff / distance
                        m.y += (self._own_pos[-1].p.y - val[-1].p.y) * diff / distance
                        m.z += (self._own_pos[-1].p.z - val[-1].p.z) * diff / distance
                    elif distance == 0:
                        # create random vector with length = repulsion radius
                        rand = np.random.random_sample()
                        m.x += (2 * np.random.randint(2) - 1) * rand * self._view_distance
                        m.y += (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * self._view_distance
                        m.z += 0
        return m


    # AGGREGATION - build potential field (merging of information)
    def _aggregate_max(self, pose): #TODO: unit test
        """
        follow higher gradient values (= gradient with shortest relative distance)
        sets current gradient to direction vector (length <= 1)
        """

        gradients = []
        tmp_att = 0
        tmp_grad = Vector3()

        if self._data:
            for element in self._data:
                # check if gradient is within view of robot
                if calc.get_gradient_distance(element.p, pose) <= element.diffusion + element.goal_radius \
                        + self._view_distance:
                    gradients.append(element)

        # find gradient with highest value ( = closest relative distance)
        if gradients:
            for gradient in gradients:
                if gradient.attraction == 1:
                    grad = self._calc_attractive_gradient(gradient, pose)
                else:
                    grad = self._calc_repulsive_gradient(gradient, pose)

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


    def _aggregate_nearest_repulsion(self, pose): # TODO unit test!!!
        """
        aggregate nearest attractive gradient with repulsive gradients s.t. robot finds gradient source avoiding the
        repulsive gradient sources
        :param pose:
        :return
        """

        gradients_attractive = []
        gradients_repulsive = []
        vector_attraction = Vector3()
        vector_repulsion = Vector3()
        tmp_att = 0

        if self._data:
            for element in self._data:
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
                grad = self._calc_attractive_gradient(gradient, pose)
                att = np.linalg.norm([grad.x, grad.y])
                if att > tmp_att:
                    vector_attraction.x = grad.x
                    vector_attraction.y = grad.y
                    tmp_att = att

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                grad = self._calc_repulsive_gradient(gradient, pose)
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

    def _aggregate_all(self, pose):  # TODO unit test!!!
        """
        aggregate all vectors
        :param pose:
        :return:
        """

        gradients_attractive = []
        gradients_repulsive = []
        vector_attraction = Vector3()
        vector_repulsion = Vector3()

        if self._data:
            for element in self._data:
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
                grad = self._calc_attractive_gradient(gradient, pose)
                vector_attraction.x += grad.x
                vector_attraction.y += grad.y

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                grad = self._calc_repulsive_gradient(gradient, pose)
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
    # TODO: think about integration of goal_radius (only in check of minimum diffusion radius, obviously)
    def _evaporate_buffer(self):
        '''
        evaporate buffer data
        :return:
        '''
        for i in xrange(len(self._data) -1, -1, -1): # go in reverse order
            diff = rospy.Time.now() - self._data[i].header.stamp

            if diff >= rospy.Duration(self._evaporation_time):
                n = diff.secs // self._evaporation_time
                self._data[i].diffusion *= self._evaporation_factor ** n
                self._data[i].header.stamp += rospy.Duration(n*self._evaporation_time)

                if self._data[i].diffusion < self._min_diffusion:
                    del self._data[i] # delete element from list

    # Potential field calculations based on Balch and Hybinette Paper (doi:10.1109/ROBOT.2000.844042)
    def _calc_attractive_gradient(self, gradient, pose):
        """
        :param gradient: position of the goal
        :param pose: position of the robot
        :return: attractive vector
        """
        v = Vector3()

        # distance goal - agent
        tmp = Vector3()
        tmp.x = gradient.p.x - pose.x
        tmp.y = gradient.p.y - pose.y
        tmp.z = gradient.p.z - pose.z

        d = np.linalg.norm([tmp.x, tmp.y, tmp.z])
        if d <= gradient.goal_radius:
            v.x = 0
            v.y = 0
            v.z = 0
        elif gradient.goal_radius < d <= gradient.goal_radius + gradient.diffusion:
            # calculate norm vector for direction
            tmp.x /= d
            tmp.y /= d
            tmp.z /= d
            # calculate magnitude of vector
            magnitude = (d - gradient.goal_radius) / gradient.diffusion
            # calculate attraction vector
            v.x = magnitude * tmp.x
            v.y = magnitude * tmp.y
            v.z = magnitude * tmp.z
        elif d > gradient.goal_radius + gradient.diffusion:
            # calculate norm vector for direction
            tmp.x /= d
            tmp.y /= d
            tmp.z /= d
            # calculate attraction vector
            v.x = 1.0 * tmp.x
            v.y = 1.0 * tmp.y
            v.z = 1.0 * tmp.z

        return v

    def _calc_repulsive_gradient(self, gradient, pose):
        """
        :param gradient: position of the goal
        :param pose: position of the robot
        :return: repulsive vector
        """
        v = Vector3()

        # distance goal - agent

        tmp = Vector3()
        tmp.x = pose.x - gradient.p.x
        tmp.y = pose.y - gradient.p.y
        tmp.z = pose.z - gradient.p.z

        d = np.linalg.norm([tmp.x, tmp.y, tmp.z])

        if d <= gradient.goal_radius: #infinitely large repulsion
            # calculate norm vector for direction
            tmp.x /= d
            tmp.y /= d
            tmp.z /= d
            # calculate repulsion vector
            v.x = np.inf * tmp.x
            v.y = np.inf * tmp.x
            v.z = np.inf * tmp.x
        elif gradient.goal_radius < d <= gradient.diffusion + gradient.goal_radius:
            # calculate norm vector for direction
            tmp.x /= d
            tmp.y /= d
            tmp.z /= d
            # calculate magnitude of vector
            magnitude = (gradient.diffusion + gradient.goal_radius - d) / gradient.diffusion
            # calculate repulsion vector
            v.x = magnitude * tmp.x
            v.y = magnitude * tmp.y
            v.z = magnitude * tmp.z
        elif d > gradient.diffusion + gradient.goal_radius:
            v.x = 0
            v.y = 0
            v.y = 0

        return v

