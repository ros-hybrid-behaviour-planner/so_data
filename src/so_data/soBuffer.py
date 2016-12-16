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
    def __init__(self, aggregation='max', min_diffusion=0.1,
                 view_distance=2.0, id='', result='reach', collision_avoidance='', repulsion_radius=2.0,
                 store_neighbors = True, neighbor_storage_size=2, framestorage=[]):
        """
        :param aggregation: indicator which kind of aggregation should be applied
                options: * min = keep gradients with minimum diffusion radius
                         * max = keep gradients with maximum diffusion radius
                         * avg = combine gradients / average
        :type aggregation: str.
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
        :param repulsion_radius: how strong repulsion is
        :param store_neighbors: specifies whether data about neighbors is stored or not
        :type store_neighbors: bool
        :param neighbor_storage_size: how many gradient messages per neighbor will be stored; robot frame ID considered
                                      as being "robotX" with X being the robot's ID
        :type neighbor_storage_size: int [0, inf.]
        :param framestorage: list of frame IDs which should be stored
                options: * [] = all frame IDs will be stored
                         * [key1, key2, ...] = only gradients which have one of the specified frame IDs will be stored
                         * 'None' has to be specified as the key for gradients without frameID
        :type framestorage: list of strings
        :param id: should have the form 'robotX' with X being the robot's id; frame ID's in this form are considered
                   as robot position data
        """

        rospy.Subscriber('soData', soMessage, self.store_data)

        self._data = {} # store incoming, dict with frameIds and arrays of data
        # initialise dictionary with frame IDs/keys
        if framestorage:
            for id in framestorage:
                self._data[id] = []
        self._frames = framestorage

        self._own_pos = [] # store own last positions
        self._neighbors = {} # empty dict
        self._current_gradient = Vector3()

        # options
        if aggregation != 'min' and aggregation != 'max' and aggregation != 'avg' and aggregation != 'newest':
            rospy.logerr("Wrong aggregation type in soBuffer. Set to max.")
            self._aggregation = 'max'
        else:
            self._aggregation = aggregation

        self._store_neighbors = store_neighbors
        self._neighbor_storage_size = neighbor_storage_size
        self._min_diffusion = min_diffusion
        self._repulsion_radius = repulsion_radius
        self._view_distance = view_distance
        if collision_avoidance != 'repulsion' and collision_avoidance != 'gradient' and collision_avoidance != '':
            rospy.logerr("No valid option for collision avoidance entered. Set to gradient.")
            self._collision_avoidance = 'gradient'
        else:
            self._collision_avoidance = collision_avoidance

        self._id = id

        if result != 'all' and result != 'max' and result != 'near' and result != 'reach':
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
        # aggregate data
        # store msgs with no frame id with dictionary key 'None'
        if not msg.header.frame_id:
            msg.header.frame_id = 'None'

        # no frames specified - store everything; add key to dict
        if not self._frames and msg.header.frame_id not in self._data and msg.header.frame_id[:5] != 'robot':
            self._data[msg.header.frame_id] = []

        #store own position and neighbor data
        if msg.header.frame_id[:5] == 'robot':
            if self._store_neighbors:
                if self._id and msg.header.frame_id == self._id:
                    self._own_pos.append(msg)
                    if len(self._own_pos) > self._neighbor_storage_size:
                        del self._own_pos[0]
                elif msg.header.frame_id in self._neighbors:
                    #check if data is newer
                    if msg.header.stamp > self._neighbors[msg.header.frame_id][-1].header.stamp:
                        self._neighbors[msg.header.frame_id].append(msg)
                    if len(self._neighbors[msg.header.frame_id]) > self._neighbor_storage_size:
                        del self._neighbors[msg.header.frame_id][0]
                else:
                    self._neighbors[msg.header.frame_id] = [msg]
        else: # Aggregation of data with same content (position), but different diffusion radii
            # check if data with frame_id should be stored

            # evaporate stored data (only non-neighbor data)
            self._evaporate_buffer()

            # evaporate received data (only non-neighbor data)
            msg = self._evaporate_msg(msg)
            if not msg:
                return

            if msg.header.frame_id in self._data:
                found = False
                if self._data[msg.header.frame_id]:
                    for i in xrange(len(self._data[msg.header.frame_id]) - 1, -1, -1):
                        if self._data[msg.header.frame_id][i].p.x == msg.p.x and \
                                        self._data[msg.header.frame_id][i].p.y == msg.p.y:
                            found = True
                            if self._aggregation == 'max': # keep data with max diffusion radius
                                if msg.diffusion + msg.goal_radius >= self._data[msg.header.frame_id][i].diffusion \
                                        + self._data[msg.header.frame_id][i].goal_radius:
                                    del self._data[msg.header.frame_id][i]
                                    self._data[msg.header.frame_id].append(msg)
                            elif self._aggregation == 'min': # keep data with min diffusion radius
                                if msg.diffusion + msg.goal_radius <= self._data[msg.header.frame_id][i].diffusion \
                                        + self._data[msg.header.frame_id][i].goal_radius:
                                    del self._data[msg.header.frame_id][i]
                                    self._data[msg.header.frame_id].append(msg)
                            elif self._aggregation == 'avg':
                                # attraction is the same direction
                                if msg.attraction == self._data[msg.header.frame_id][i].attraction:
                                    msg.diffusion = (msg.diffusion + self._data[msg.header.frame_id][i].diffusion)/2
                                    msg.goal_radius = (msg.goal_radius +
                                                       self._data[msg.header.frame_id][i].goal_radius)/2
                                else:
                                    # change sign
                                    if self._data[msg.header.frame_id][i].diffusion + \
                                            self._data[msg.header.frame_id][i].goal_radius \
                                            > msg.diffusion + msg.goal_radius:
                                        msg.attraction *= -1
                                    msg.diffusion = np.absolute(msg.diffusion
                                                                - self._data[msg.header.frame_id][i].diffusion)
                                    msg.goal_radius = np.absolute(msg.goal_radius -
                                                                  self._data[msg.header.frame_id][i].goal_radius)
                                del self._data[msg.header.frame_id][i]
                                # store average element as long as it has a goal radius or a diffision radius larger
                                # than the minimum required diffusion
                                if msg.diffusion >= self._min_diffusion or msg.goal_radius != 0.0:
                                    self._data[msg.header.frame_id].append(msg)
                            elif self._aggregation == 'newest': # keep last received gradient at one position
                                if msg.header.stamp >= self._data[msg.header.frame_id][i].header.stamp:
                                    del self._data[msg.header.frame_id][i]
                                    self._data[msg.header.frame_id].append(msg)
                else:
                    self._data[msg.header.frame_id].append(msg)
                    found = True

                if not found:
                    self._data[msg.header.frame_id].append(msg)

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

        # apply evaporation before proceeding with calculations
        self._evaporate_buffer()

        # distance vector based on gradients - merges available information
        if self._result == 'near':
            self._aggregate_nearest_repulsion(pose)
        elif self._result == 'max':
            self._aggregate_max(pose)
        elif self._result == 'all':
            self._aggregate_all(pose)
        elif self._result == 'reach':
            self._aggregate_nearest_ge(pose)

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

    def get_goal_reached(self, pose, frameids=[]):
        """
        determines whether nearest attractive gradient was reached - especially for return == reach option
        returns True in case that gradient was reached or no gradient to be reached was found
        False otherweise
        :param pose: Pose Message with position of robot
        :return: True/False (bool)
        """
        gradients_attractive = []
        tmp_att = -1

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._data.keys()

        for fid in frameids:
            if fid in self._data:
                for element in self._data[fid]:
                    if calc.get_gradient_distance(element.p, pose) <= element.diffusion + element.goal_radius \
                            + self._view_distance:
                        if element.attraction == 1:
                            gradients_attractive.append(element)

        if gradients_attractive:
            for gradient in gradients_attractive:
                # find nearest attractive gradient
                tmp = Vector3()
                tmp.x = gradient.p.x - pose.x
                tmp.y = gradient.p.y - pose.y
                tmp.z = gradient.p.z - pose.z

                # attraction = distance to goal area
                att = np.linalg.norm([tmp.x, tmp.y, tmp.z]) - gradient.goal_radius
                if att < 0.0:
                    att = 0.0

                if att > tmp_att:
                    tmp_att = att
                elif att == 0.0:
                    return True
        else:
                return True

        if tmp_att == 0.0:
            return True
        else:
            return False

    # Collision avoidance between neighbors
    def _gradient_repulsion(self, pose):
        """
        returns repulsion vector (collision avoidance between neighbors) based on potential field approach
        considers all neighbours that have a gradient reaching inside view distance / communication range of agent
        :param pose:
        :return:
        """
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
                        repulsion.x += (2 * np.random.randint(2) - 1) * rand * self._repulsion_radius
                        repulsion.y += (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * self._repulsion_radius
                    else:
                        repulsion.x += grad.x
                        repulsion.y += grad.y

        # limit repulsion vector to repulsion radius
        d = calc.vector_length(repulsion)
        if d > self._repulsion_radius:
            repulsion.x *= self._repulsion_radius / d
            repulsion.y *= self._repulsion_radius / d
            repulsion.z *= self._repulsion_radius / d

        return repulsion

    def _repulsion_vector(self):
        """
        return a repulsion vector based on formula presented by Fernandez-Marquez et al., use of received gradients (p)
        for calculation
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
                        diff = self._repulsion_radius - distance
                        m.x += (self._own_pos[-1].p.x - val[-1].p.x) * diff / distance
                        m.y += (self._own_pos[-1].p.y - val[-1].p.y) * diff / distance
                        m.z += (self._own_pos[-1].p.z - val[-1].p.z) * diff / distance
                    elif distance == 0:
                        # create random vector with length = repulsion radius
                        rand = np.random.random_sample()
                        m.x += (2 * np.random.randint(2) - 1) * rand * self._repulsion_radius
                        m.y += (2 * np.random.randint(2) - 1) * np.sqrt(1 - rand) * self._repulsion_radius
                        m.z += 0

        # max repulsion vector length = repulsion radius of robot
        d = np.linalg.norm([m.x, m.y, m.z])
        if d > self._repulsion_radius:
            m.x *= self._repulsion_radius / d
            m.y *= self._repulsion_radius / d
            m.z *= self._repulsion_radius / d

        return m

    # AGGREGATION - build potential field (merging of information)
    def _aggregate_max(self, pose, frameids=[]): #TODO: unit test
        """
        follow higher gradient values (= gradient with shortest relative distance)
        sets current gradient to direction vector (length <= 1)
        """

        gradients = []
        tmp_att = 0
        tmp_grad = Vector3()

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._data.keys()

        for fid in frameids:
            if fid in self._data:
                for element in self._data[fid]:
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

    def _aggregate_nearest_repulsion(self, pose, frameids=[]): # TODO unit test!!!
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

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._data.keys()

        for fid in frameids:
            if fid in self._data:
                for element in self._data[fid]:
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

    def _aggregate_nearest_ge(self, pose, frameids=[]): # TODO unit test!!!
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
        tmp_att = -1
        attractive_gradient = soMessage()

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._data.keys()

        for fid in frameids:
            if fid in self._data:
                for element in self._data[fid]:
                    if calc.get_gradient_distance(element.p, pose) <= element.diffusion + element.goal_radius \
                            + self._view_distance:
                        if element.attraction == 1:
                            gradients_attractive.append(element)
                        elif element.attraction == -1:
                            gradients_repulsive.append(element)

        if gradients_attractive:
            for gradient in gradients_attractive:
                # find nearest attractive gradient
                grad = self._calc_attractive_gradient_ge(gradient, pose)
                att = np.linalg.norm([grad.x, grad.y])
                if att > tmp_att:
                    vector_attraction.x = grad.x
                    vector_attraction.y = grad.y
                    tmp_att = att
                    attractive_gradient = gradient

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                grad = self._calc_repulsive_gradient_ge(gradient, attractive_gradient, pose)
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

    def _aggregate_all(self, pose, frameids=[]):  # TODO unit test!!!
        """
        aggregate all vectors
        :param pose:
        :return:
        """

        gradients_attractive = []
        gradients_repulsive = []
        vector_attraction = Vector3()
        vector_repulsion = Vector3()

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._data.keys()

        for fid in frameids:
            if fid in self._data:
                for element in self._data[fid]:
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
    def _evaporate_buffer(self):
        """
        evaporate buffer data stored in self._data
        neighbor data is not evaporated as it is considered being fixed
        :return:
        """
        for fid in self._data:
            if fid in self._data and self._data[fid]:
                for i in xrange(len(self._data[fid]) -1, -1, -1): # go in reverse order
                    if self._data[fid][i].ev_time > 0:
                        diff = rospy.Time.now() - self._data[fid][i].header.stamp
                        if diff >= rospy.Duration(self._data[fid][i].ev_time):
                            n = diff.secs // self._data[fid][i].ev_time
                            self._data[fid][i].diffusion *= self._data[fid][i].ev_factor ** n
                            self._data[fid][i].header.stamp += rospy.Duration(n*self._data[fid][i].ev_time)
                    else:  # delta t for evaporation = 0 and evaporation applies, set diffusion immediately to 0
                        if self._data[fid][i].ev_factor < 1.0:
                            self._data[fid][i].diffusion = 0.0

                    # in case that gradient concentration is lower than minimum and no goal_radius exists, delete data
                    if self._data[fid][i].goal_radius == 0.0 and self._data[fid][i].diffusion < self._min_diffusion:
                        del self._data[fid][i] # remove element

    def _evaporate_msg(self, msg):
        """
        evaporate a single message
        :param msg: gradient message
        :return: evaporated message
        """
        if msg.ev_time > 0:
            diff = rospy.Time.now() - msg.header.stamp
            if diff >= rospy.Duration(msg.ev_time):
                n = diff.secs // msg.ev_time
                msg.diffusion *= msg.ev_factor ** n
                msg.header.stamp += rospy.Duration(n*msg.ev_time)
        else:  # delta t for evaporation = 0 and evaporation applies, set diffusion immediately to 0
            if msg.ev_factor < 1.0:
                msg.diffusion = 0

        if msg.diffusion >= self._min_diffusion or msg.goal_radius != 0.0:
            return msg

    # Potential field calculations based on Balch and Hybinette Paper (doi:10.1109/ROBOT.2000.844042)
    @staticmethod
    def _calc_attractive_gradient(gradient, pose):
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

    @staticmethod
    def _calc_repulsive_gradient(gradient, pose):
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


    # second version of gradients based on Ge & Cui
    @staticmethod
    def _calc_attractive_gradient_ge(gradient, pose):
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
        if d <= gradient.goal_radius + gradient.diffusion:
            v.x = tmp.x
            v.y = tmp.y
            v.z = tmp.z
        elif d > gradient.goal_radius + gradient.diffusion:
            v.x = (tmp.x / d) * (gradient.goal_radius + gradient.diffusion)
            v.y = (tmp.y / d) * (gradient.goal_radius + gradient.diffusion)
            v.z = (tmp.z / d) * (gradient.goal_radius + gradient.diffusion)

        return v

    @staticmethod
    def _calc_repulsive_gradient_ge(gradient, goal, pose):
        """
        :param gradient: position of the goal
        :param pose: position of the robot
        :return: repulsive vector
        distance of influence of obstacle = goal_radius + diffusion
        """
        v = Vector3()

        # distance obstacle - agent
        tmp = Vector3()
        tmp.x = pose.x - gradient.p.x
        tmp.y = pose.y - gradient.p.y
        tmp.z = pose.z - gradient.p.z

        d = np.linalg.norm([tmp.x, tmp.y, tmp.z])
        if d <= gradient.goal_radius:
            v.x = -1.0 * np.inf
            v.y = -1.0 * np.inf
            v.z = -1.0 * np.inf
        elif gradient.goal_radius < d <= gradient.goal_radius + gradient.diffusion:
            # unit vector obstacle - agent
            tmp.x /= d
            tmp.y /= d
            tmp.z /= d
            # distance repulsive gradient - goal
            d_goal = calc.get_gradient_distance(pose, goal.p)
            # unit vector agent - goal
            ag = Vector3()
            ag.x = (goal.p.x - pose.x) / d_goal
            ag.y = (goal.p.y - pose.y) / d_goal
            ag.z = (goal.p.z - pose.z) / d_goal
            # closest distance to obstacle  - diffusion
            d_obs_diff = (1.0/(d - gradient.goal_radius)) - (1.0/gradient.diffusion)
            n = 1.0
            f_rep1 = d_obs_diff * ((d_goal**n) / ((d - gradient.goal_radius)**n))
            f_rep2 = (n/2.0) * np.square(d_obs_diff) * (d_goal ** (n-1))
            v.x = f_rep1 * tmp.x + f_rep2 * ag.x
            v.y = f_rep1 * tmp.y + f_rep2 * ag.y
            v.z = f_rep1 * tmp.z + f_rep2 * ag.z

        elif d > gradient.goal_radius + gradient.diffusion:
            v.x = 0
            v.y = 0
            v.z = 0

        return v
