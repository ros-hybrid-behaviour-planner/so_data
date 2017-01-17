""" Created on 07.11.2016
.. module:: soBuffer
.. moduleauthor:: kaiser
"""

from __future__ import \
    division  # force floating point division when using plain /
import rospy
from so_data.msg import soMessage
import numpy as np
import calc
from geometry_msgs.msg import Vector3
import flocking
import collections
import random


class SoBuffer(object):
    """
    This class is the buffer for received self-organization data
    """

    def __init__(self, aggregation={'DEFAULT': 'max'},
                 aggregation_distance=1.0, min_diffusion=0.1,
                 view_distance=1.5, id='', result='near',
                 collision_avoidance='',
                 moving_storage_size=2, store_all=True,
                 framestorage=[], threshold=2, a=1.0,
                 b=1.0, h=0.5, epsilon=1.0, max_acceleration=1.0,
                 max_velocity=1.0, quorum_moving=True, quorum_static=False,
                 min_velocity=0.1):

        """
        :param aggregation: indicator which kind of aggregation should be
        applied per frameID at a gradient center / within aggregation_distance
        of gradient center. "DEFAULT" used for gradients without own
        aggregation option.
                options: * min = keep gradients with minimum diffusion radius
                                 + goal radius
                         * max = keep gradients with maximum diffusion radius
                                 + goal radius
                         * avg = combine gradients / average
                         * newest = store newest received gradient
        :type aggregation: dictionary - key: frameID value: aggregation option

        :param aggregation_distance: radius in which gradient data is
        aggregated
        :type aggregation_distance: float

        :param min_diffusion: threshold, gradients with smaller diffusion
        radius will be deleted (when goal_radius != 0)
        :type min_diffusion: float

        :param view_distance: radius in which agent can sense gradients;
        should be >= goal_radius of own gradient
        :type view_distance: float

        :param id: agent's id, gradients with this id are stored in
        self._own_pos
        :type id: str

        :param result: specifies vector which should be returned
        (gradients within view distance considered);
                options: * all = return vector considering all vectors of
                          potential field
                         * max = return vector with max attraction / repulsion
                         * near = return vector to nearest attractive vector
                          avoiding obstacles
                         * reach = return vector which enables to reach nearest
                          attractive gradient
                         * avoid = return vector leading away form all
                          gradients
        :type result: str

        :param collision_avoidance: avoidance of neighbors
                options: * gradient = potential field approach to realize
                          collision avoidance between neighbors
                         * repulsion = repulsion vector calculation based on
                          Fernandez-Marquez et al.
        :type collision_avoidance: str

        :param moving_storage_size: how many gradient messages per moving
         gradient will be stored, set 0 not to store any neighbor gradients
        :type moving_storage_size: int [0, inf]

        :param store_all: defines whether all frameIDs will be stored or only
                    the frameIDs defined in framestorage
        :param framestorage: list of frame IDs which should be stored,
                        applies both for moving and for static gradients
                options: * [] = no gradients will be stored
                         * [key1, key2, ...] = only gradients which have one of
                          the specified frame IDs will be stored
                         * 'None' has to be specified as the key for gradients
                         without frameID
        :type framestorage: list of strings

        :param threshold: quorum sensing threshold to be passed to return True
        :type threshold: int

        :param quorum_moving: consider moving gradients (True) or not (False)
                            in quorum decision
        :type quorum_moving: bool

        :param quorum_static: consider static gradients (True) or not (False)
                            in quorum decision
        :type quorum_static: bool

        :param max_velocity: maximum velocity of robot (= length of returned
        vector); used in get_current_gradient and flocking
        :param min_velocity:  minimum velocity of robot (= length of returned
        vector); used in get_current_gradient and flocking

        flocking parameters
        :param a: action function parameter
        :param b: action function parameter
                    0 < a <= b; c = |a-b|/np.sqrt(4ab)
        :param h: parameter (0,1) specifying boundaries of bump function
        :param epsilon: sigma norm parameter (0,1)
        :param max_acceleration: maximum acceleration of robot (flocking)
        """

        # STORE DATA
        self._static = {}  # static gradient storage, dict
        self._own_pos = []  # own positions storage
        self._moving = {}  # moving gradients storage, e.g. robots, dict

        self._store_all = store_all
        self._frames = framestorage

        self._aggregation = aggregation
        self._aggregation_distance = aggregation_distance
        self._min_diffusion = min_diffusion
        self._id = id  # fixed, no setter
        self._moving_storage_size = moving_storage_size

        # RETURN AGGREGATED DATA
        self._view_distance = view_distance
        self.result = result

        if collision_avoidance != 'repulsion' and \
                        collision_avoidance != 'gradient' and \
                        collision_avoidance != '' and \
                        collision_avoidance != 'reach':
            rospy.logerr("No valid option for collision avoidance entered. "
                         "Set to gradient.")
            self.collision_avoidance = 'gradient'
        else:
            self.collision_avoidance = collision_avoidance

        # quorum
        self.threshold = threshold
        self.quorum_moving = quorum_moving
        self.quorum_static = quorum_static

        # flocking
        self.a = a
        self.b = b
        self.h = h
        self.epsilon = epsilon
        self.max_acceleration = max_acceleration
        self.max_velocity = max_velocity

        self.min_velocity = min_velocity

        rospy.Subscriber('soData', soMessage, self.store_data)

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

        # no frames specified - store everything; add key to dict s.t. it
        # will be stored
        # otherwise it results in self._data[msg.header.frame_id] == None
        #if not self._frames and msg.header.frame_id not in self._data and \
        #                msg.header.frame_id[:5] != 'robot':
        #    self._data[msg.header.frame_id] = []

        # check if received msg should be stored
        if not self._store_all:
            if msg.header.frame_id not in self._frames:
                return

        # store own position and neighbor / moving agents data
        # if msg.header.frame_id[:5] == 'robot':
        if msg.moving:
            if self._moving_storage_size > 0:
                if self._id and msg.header.frame_id == self._id:
                    # check if data is newer
                    if self._own_pos:  # own data already stored
                        if msg.header.stamp > self._own_pos[-1].header.stamp:
                            self._own_pos.append(msg)
                    else:  # no own position stored so far
                        self._own_pos.append(msg)
                    # maximum length of stored own gradients exceeded
                    if len(self._own_pos) > self._moving_storage_size:
                        del self._own_pos[0]
                elif msg.header.frame_id in self._moving:
                    # check if data is newer
                    if msg.header.stamp > \
                            self._moving[msg.header.frame_id][-1]. \
                                    header.stamp:
                        self._moving[msg.header.frame_id].append(msg)
                    # maximum length of stored neighbor gradients exceeded
                    if len(self._moving[msg.header.frame_id]) > \
                            self._moving_storage_size:
                        del self._moving[msg.header.frame_id][0]
                else:
                    self._moving[msg.header.frame_id] = [msg]
        else:  # Aggregation of data with same content (position), but
               # different diffusion radii

            # set aggregation value for received message based on frameID
            if msg.header.frame_id in self._aggregation.keys():
                aggregation = self._aggregation[msg.header.frame_id]
            elif 'DEFAULT' in self._aggregation.keys():  # only default value
                # is specified
                aggregation = self._aggregation['DEFAULT']
            else:
                rospy.logerr("No DEFAULT value specified for aggregation!")
                return

            # evaporate stored data (only non-neighbor data)
            self._evaporate_buffer()

            # evaporate received data (only non-neighbor data)
            msg = self._evaporate_msg(msg)
            if not msg:  # evaporation let to disappearance of the message
                return

            if msg.header.frame_id in self._static:
                # indicates whether data point at same position / within
                # aggregation radius is already stored
                view = {}

                # find all points which are in aggregation range of message
                for i in xrange(len(self._static[msg.header.frame_id])
                                        - 1, -1, -1):
                    distance = calc.get_gradient_distance(
                        self._static[msg.header.frame_id][i].p, msg.p)
                    # data point lies within aggregation distance
                    if distance <= self._aggregation_distance:
                        view[i] = distance

                # find minimum value of gradient centers within view
                if view:
                    found = True
                    min_val = min(view.values())  # get minimum distance
                    result = [j for j, v in view.items() if v == min_val]

                    # several with same distance - random value of list
                    if len(result) > 1:
                        k = random.choice(result)
                    # only one minimum - use found index
                    else:
                        k = result[0]

                    if aggregation == 'max':  # keep data with max reach
                        if msg.diffusion + msg.goal_radius >= \
                                        self._static[msg.header.frame_id][k]. \
                                                diffusion \
                                        + self._static[msg.header.frame_id][k]. \
                                        goal_radius:
                            del self._static[msg.header.frame_id][k]
                            self._static[msg.header.frame_id].append(msg)
                    elif aggregation == 'min':  # keep data with min reach
                        if msg.diffusion + msg.goal_radius <= \
                                        self._static[msg.header.frame_id][k]. \
                                                diffusion \
                                        + self._static[msg.header.frame_id][k]. \
                                        goal_radius:
                            del self._static[msg.header.frame_id][k]
                            self._static[msg.header.frame_id].append(msg)
                    elif aggregation == 'avg':
                        # attraction is the same direction
                        if msg.attraction == \
                                self._static[msg.header.frame_id][k]. \
                                        attraction:
                            msg.diffusion = (msg.diffusion +
                                             self._static[
                                                 msg.header.frame_id][k].
                                             diffusion) / 2.0
                            msg.goal_radius = (msg.goal_radius +
                                               self._static[
                                                   msg.header.frame_id][k].
                                               goal_radius) / 2.0
                        else:
                           # change sign
                            if self._static[msg.header.frame_id][
                                k].diffusion + \
                                    self._static[msg.header.frame_id][
                                        k].goal_radius \
                                    > msg.diffusion + msg.goal_radius:
                                msg.attraction *= -1
                            msg.diffusion = np.absolute(msg.diffusion -
                                                        self._static[
                                                            msg.header.frame_id][
                                                            k].diffusion)
                            msg.goal_radius = np.absolute(msg.goal_radius -
                                                           self._static[
                                                              msg.header.frame_id][
                                                              k].goal_radius)

                        msg.p.x = (msg.p.x +
                                   self._static[msg.header.frame_id][k].p.x) \
                                  / 2.0
                        msg.p.y = (msg.p.y +
                                   self._static[msg.header.frame_id][k].p.y) \
                                  / 2.0
                        msg.p.z = (msg.p.z +
                                   self._static[msg.header.frame_id][k].p.z) \
                                  / 2.0
                        del self._static[msg.header.frame_id][k]
                        # store average element as long as it has a goal
                        # radius or a diffusion radius larger
                        # than the minimum required diffusion
                        if msg.diffusion >= self._min_diffusion or \
                                        msg.goal_radius != 0.0:
                            self._static[msg.header.frame_id].append(msg)
                    elif aggregation == 'newest':  # keep last received
                       # gradient at one position
                        if msg.header.stamp >= \
                                self._static[msg.header.frame_id][k]. \
                                       header.stamp:
                            del self._static[msg.header.frame_id][k]
                            self._static[msg.header.frame_id].append(msg)

                else:
                    self._static[msg.header.frame_id].append(msg)
            else:
                self._static[msg.header.frame_id] = [msg]

    def get_current_gradient(self, frameids=[]):  # TODO unit test anpassen?
        """
        returns movement vector based on gradients & with or without collision
        avoidance
        :param pose: Pose Message with position of robot (geometry msgs Pose)
        :return current gradient vector to follow based on settings
        """
        # apply evaporation before proceeding with calculations
        self._evaporate_buffer()

        # result vector
        result = Vector3()

        # distance vector based on gradients - merges available information
        if self.result == 'near':
            result = self._aggregate_nearest_repulsion(frameids=frameids)
        elif self.result == 'max':
            result = self._aggregate_max(frameids=frameids)
        elif self.result == 'all':
            result = self._aggregate_all(frameids=frameids)
        elif self.result == 'reach':
            result = self._aggregate_nearest_ge(frameids=frameids)
        elif self.result == 'avoid':
            result = self._aggregate_avoid_all(frameids=frameids)
        elif self.result == 'flocking':
            result = self.flocking()

        # collision avoidance / consider moving gradients
        if self.collision_avoidance == 'gradient':
            collision = self._gradient_repulsion()
            result.x += collision.x
            result.y += collision.y
            result.z += collision.z
        elif self.collision_avoidance == 'repulsion':
            collision = self._repulsion_vector()
            result.x += collision.x
            result.y += collision.y
            result.z += collision.z

        # adjust length to be max within view_distance
        d = calc.vector_length(result)
        if d > self.max_velocity:
            result = calc.unit_vector3(result)
            result.x *= self.max_velocity
            result.y *= self.max_velocity
            result.z *= self.max_velocity
        elif 0 < d < self.min_velocity:
            result = calc.unit_vector3(result)
            result.x *= self.min_velocity
            result.y *= self.min_velocity
            result.z *= self.min_velocity

        return result

    def get_attractive_gradients_view(self, frameids=[]):
        """
        :return:
        """
        flag = False

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._static.keys()

        for fid in frameids:
            if fid in self._static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        if element.attraction == 1:
                            flag = True

        return flag

    def get_goal_reached(self, frameids=[]):
        """
        determines whether nearest attractive gradient was reached - especially
        for return == reach option
        returns True in case that gradient was reached
        False otherwise
        :return: True/False (bool)
        """
        gradients_attractive = []

        tmp_att = np.inf  # attractive gradient calculations return values
        # between 0 and 1

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._static.keys()

        for fid in frameids:
            if fid in self._static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        if element.attraction == 1:
                            gradients_attractive.append(element)

        if gradients_attractive:
            for gradient in gradients_attractive:
                # attraction = distance to goal area
                grad = self._calc_attractive_gradient(gradient)
                att = np.linalg.norm([grad.x, grad.y, grad.z])
                # attraction smaller means distance closer - normalized value
                # considered regarding diffusion radius + goal_radius
                if att < tmp_att:
                    tmp_att = att

        if tmp_att == 0.0:
            return True
        else:
            return False

    def get_attractive_distance(self, frameids=[]):   # TODO test
        """
        :return:  returns distance to closest attractive gradient
        no attractive gradients: returns np.inf
        """
        gradients_attractive = []

        tmp_att = np.inf  # attractive gradient calculations return values
        # between 0 and 1

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._static.keys()

        for fid in frameids:
            if fid in self._static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        if element.attraction == 1:
                            gradients_attractive.append(element)

        tmp_grad = soMessage()
        d = np.inf

        if gradients_attractive:
            for gradient in gradients_attractive:
                # find nearest attractive gradient
                grad = self._calc_attractive_gradient(gradient)
                att = np.linalg.norm([grad.x, grad.y, grad.z])
                # attraction decreases with being closer to gradient source
                # / goal area
                if att < tmp_att:
                    tmp_grad = gradient
                    tmp_att = att

            # distance from center to center - goal_radius agent "HW size")
            # - goal_radius gradient ("size of goal" - agent just
            # has to reach this area somehow)
            d = calc.get_gradient_distance(self._own_pos[-1].p, tmp_grad.p) \
                - self._own_pos[-1].goal_radius - tmp_grad.goal_radius

            if d < 0:
                d = 0

        return d

    def get_no_potential(self, frameids=[]):
        """
        determines whether there is still some attraction/repulsion
        :param frameids: frameIDs of gradients to be considered in calculation
        :return: True/False (bool)
        """

        # calculate gradient vector
        vector = self.get_current_gradient(frameids)
        d = calc.vector_length(vector)

        # no gradient vector to follow --> "goal reached / repulsion avoided"
        if d == 0.0:
            return True
        else:
            return False

    def get_neighbors_bool(self):
        """
        :return: True (no neighbors within view / leading to repulsive
        behaviour), False (neighbors within view / leading to repulsive
        behaviour)
        """
        flag = True

        d = 0.0

        if self.collision_avoidance == 'repulsion':
            d = calc.vector_length(self._repulsion_vector())
        elif self.collision_avoidance == 'gradient':
            d = calc.vector_length(self._gradient_repulsion())
        elif self.result == 'flocking':
            d = calc.vector_length(self.flocking())

        if d > 0:
            flag = False

        return flag

    # Collision avoidance between neighbors
    def _gradient_repulsion(self):
        """
        returns repulsion vector (collision avoidance between neighbors)
         based on potential field approach
        considers all neighbours that have a gradient reaching inside view
        distance / communication range of agent
        considers both repulsive and attractive moving gradients
        :return: repulsion vector
        """
        repulsion = Vector3()

        # no data available
        if not self._own_pos:
            return repulsion

        # repulsion radius of robot, <= view_distance w
        repulsion_radius = self._own_pos[-1].diffusion + self._own_pos[
            -1].goal_radius

        if self._moving:
            for val in self._moving.values():
                # check if neighbor is in sight
                if calc.get_gradient_distance(val[-1].p,
                                              self._own_pos[-1].p) <= val[-1]\
                        .diffusion + val[-1].goal_radius + self._view_distance:
                    # distinguish between attractive and repulsive gradients
                    if val[-1].attraction == -1:
                        grad = self._calc_repulsive_gradient(val[-1])
                    else:
                        grad = self._calc_attractive_gradient(val[-1])

                    # two robots are at the same position
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        tmp = np.random.rand(1, 3)
                        tmp /= np.linalg.norm(tmp)
                        tmp *= repulsion_radius

                        repulsion.x += (2 * np.random.randint(2) - 1) * tmp[0][
                            0]
                        repulsion.y += (2 * np.random.randint(2) - 1) * tmp[0][
                            1]
                        repulsion.z += (2 * np.random.randint(2) - 1) * tmp[0][
                            2]

                    else:
                        repulsion.x += grad.x
                        repulsion.y += grad.y
                        repulsion.z += grad.z

        # limit repulsion vector length to repulsion radius
        d = calc.vector_length(repulsion)
        if d > repulsion_radius:
            repulsion.x *= repulsion_radius / d
            repulsion.y *= repulsion_radius / d
            repulsion.z *= repulsion_radius / d

        return repulsion

    def _repulsion_vector(self):
        """
        return a repulsion vector based on formula presented by
        Fernandez-Marquez et al., use of received gradients (p)
        for calculation
        only for repulsive moving gradients
        :return repulsion vector
        """
        # initialize vector
        m = Vector3()

        # no data available
        if not self._own_pos:
            return m

        repulsion_radius = self._own_pos[-1].diffusion + self._own_pos[
            -1].goal_radius

        if self._moving and self._own_pos:
            for val in self._moving.values():
                if val[-1].attraction == -1:
                    # shortest distance
                    distance = calc.get_gradient_distance(val[-1].p,
                                                          self._own_pos[-1].p)\
                               - self._own_pos[-1].goal_radius
                    # agents within view
                    if distance < self._view_distance:
                        # only robots within repulsion
                        if distance != 0:
                            diff = repulsion_radius - distance
                            m.x += (self._own_pos[-1].p.x - val[
                                -1].p.x) * diff / distance
                            m.y += (self._own_pos[-1].p.y - val[
                                -1].p.y) * diff / distance
                            m.z += (self._own_pos[-1].p.z - val[
                                -1].p.z) * diff / distance
                        elif distance == 0:
                            # create random vector with length=repulsion radius
                            # create random vector with length (goal_radius +
                            # gradient.diffusion)
                            tmp = np.random.rand(1, 3)
                            tmp /= np.linalg.norm(tmp)
                            tmp *= repulsion_radius

                            m.x += (2 * np.random.randint(2) - 1) * tmp[0][0]
                            m.y += (2 * np.random.randint(2) - 1) * tmp[0][1]
                            m.z += (2 * np.random.randint(2) - 1) * tmp[0][2]

        # max repulsion vector length = repulsion radius of robot
        d = np.linalg.norm([m.x, m.y, m.z])
        if d > repulsion_radius:
            m.x *= repulsion_radius / d
            m.y *= repulsion_radius / d
            m.z *= repulsion_radius / d

        return m

    # AGGREGATION - build potential field (merging of information)
    def _aggregate_max(self, frameids=[]):
        """
        follow higher gradient values (= gradient with shortest relative
        distance to gradient source)
        sets current gradient to direction vector (length <= 1)
        :param pose: current position of agent
        :param frameids: frameIDs of gradients to be considered in calculation
        :return gradient vector
        """

        gradients = []
        tmp_att = -1
        tmp_grad = Vector3()

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._static.keys()

        for fid in frameids:
            if fid in self._static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        gradients.append(element)

        # find gradient with highest value ( = closest relative distance)
        if gradients:
            for gradient in gradients:
                if gradient.attraction == 1:
                    grad = self._calc_attractive_gradient(gradient)
                else:
                    grad = self._calc_repulsive_gradient(gradient)

                if grad.x == np.inf or grad.x == -1 * np.inf:
                    att = np.inf
                else:
                    att = np.linalg.norm([grad.x, grad.y, grad.z])
                    # inverse attraction for attractive gradients as attraction
                    #  decreases with getting closer to the
                    # goal zone of the gradient
                    if gradient.attraction == 1:
                        att = 1 - att

                if att > tmp_att:
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length (goal_radius +
                        # gradient.diffusion)
                        tmp = np.random.rand(1, 3)
                        tmp /= np.linalg.norm(tmp)
                        tmp *= (gradient.goal_radius + gradient.diffusion)

                        tmp_grad.x = (2 * np.random.randint(2) - 1) * tmp[0][0]
                        tmp_grad.y = (2 * np.random.randint(2) - 1) * tmp[0][1]
                        tmp_grad.z = (2 * np.random.randint(2) - 1) * tmp[0][2]
                    else:
                        tmp_grad = grad

                    tmp_att = att

        return tmp_grad

    def _aggregate_nearest_repulsion(self, frameids=[]):
        """
        aggregate nearest attractive gradient with repulsive gradients s.t.
        robot finds gradient source avoiding the
        repulsive gradient sources
        :param pose: current agent's pose
        :param frameids: frameids of gradients to be considered in the
        calculation
        :return gradient vector
        """

        gradients_attractive = []
        gradients_repulsive = []
        vector_attraction = Vector3()
        vector_repulsion = Vector3()
        tmp_att = np.inf

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._static.keys()

        for fid in frameids:
            if fid in self._static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        if element.attraction == 1:
                            gradients_attractive.append(element)
                        elif element.attraction == -1:
                            gradients_repulsive.append(element)

        if gradients_attractive:
            for gradient in gradients_attractive:
                # find nearest attractive gradient
                grad = self._calc_attractive_gradient(gradient)
                att = np.linalg.norm([grad.x, grad.y, grad.z])
                # attraction decreases with being closer to gradient source
                # / goal area
                if att < tmp_att:
                    vector_attraction.x = grad.x
                    vector_attraction.y = grad.y
                    vector_attraction.z = grad.z
                    tmp_att = att

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                grad = self._calc_repulsive_gradient(gradient)
                # robot position is within obstacle radius, inf can't be
                # handled as direction
                # --> add vector which brings robot to the boarder of the
                # obstacle
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length (goal_radius +
                    # gradient.diffusion)
                    tmp = np.random.rand(1, 3)
                    tmp /= np.linalg.norm(tmp)
                    tmp *= (gradient.goal_radius + gradient.diffusion)

                    vector_repulsion.x += (2 * np.random.randint(2) - 1) * \
                                          tmp[0][0]
                    vector_repulsion.y += (2 * np.random.randint(2) - 1) * \
                                          tmp[0][1]
                    vector_repulsion.z += (2 * np.random.randint(2) - 1) * \
                                          tmp[0][2]
                else:
                    vector_repulsion.x += grad.x
                    vector_repulsion.y += grad.y
                    vector_repulsion.z += grad.z

        vector_attraction.x += vector_repulsion.x
        vector_attraction.y += vector_repulsion.y
        vector_attraction.z += vector_repulsion.z

        return vector_attraction

    def _aggregate_nearest_ge(self, frameids=[]):
        """
        aggregate nearest attractive gradient with repulsive gradients s.t.
        robot finds gradient source avoiding the
        repulsive gradient sources based on Ge & Cui
        requires at least one attractive gradient to be sensed
        :param pose: current robot pose
        :param frameids: frameIDs of gradients to be considered in calculation
        :return gradient vector
        """
        gradients_attractive = []
        gradients_repulsive = []
        vector_attraction = Vector3()
        vector_repulsion = Vector3()
        tmp_att = 2
        attractive_gradient = soMessage()

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._static.keys()
        if self.collision_avoidance == 'reach':
            frameids += self._moving.keys()

        for fid in frameids:
            if fid in self._static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        if element.attraction == 1:
                            gradients_attractive.append(element)
                        elif element.attraction == -1:
                            gradients_repulsive.append(element)

            if fid in self._moving:
                if self._moving[fid][-1].attraction == 1:
                    gradients_attractive.append(self._moving[fid][-1])
                elif self._moving[fid][-1].attraction == -1:
                    gradients_repulsive.append(self._moving[fid][-1])

        if gradients_attractive:
            for gradient in gradients_attractive:
                # find nearest attractive gradient
                grad = self._calc_attractive_gradient(gradient)
                # returns value between 0 and 1
                att = np.linalg.norm([grad.x, grad.y, grad.z])
                # attraction smaller means distance closer - normalized value
                # taken regarding
                # diffusion radius + goal_radius
                if att < tmp_att:
                    grad = self._calc_attractive_gradient_ge(gradient)
                    vector_attraction.x = grad.x
                    vector_attraction.y = grad.y
                    vector_attraction.z = grad.z
                    tmp_att = att
                    attractive_gradient = gradient
        # no attractive gradients available, return zero vector
        else:
            return vector_attraction

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                grad = self._calc_repulsive_gradient_ge(gradient,
                                                        attractive_gradient)
                # robot position is within obstacle goal radius, inf can't be
                # handled as direction
                # --> add vector which brings robot to the boarder of the
                # obstacle
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length (goal_radius +
                    # gradient.diffusion)
                    tmp = np.random.rand(1, 3)
                    tmp /= np.linalg.norm(tmp)
                    tmp *= (gradient.goal_radius + gradient.diffusion)

                    vector_repulsion.x += (2 * np.random.randint(2) - 1) * \
                                          tmp[0][0]
                    vector_repulsion.y += (2 * np.random.randint(2) - 1) * \
                                          tmp[0][1]
                    vector_repulsion.z += (2 * np.random.randint(2) - 1) * \
                                          tmp[0][2]
                else:
                    vector_repulsion.x += grad.x
                    vector_repulsion.y += grad.y
                    vector_repulsion.z += grad.z

        vector_attraction.x += vector_repulsion.x
        vector_attraction.y += vector_repulsion.y
        vector_attraction.z += vector_repulsion.z

        return vector_attraction

    def _aggregate_all(self, frameids=[]):
        """
        aggregate all vectors within view distance
        :param pose: current robot position
        :param frameids: frameIDs of gradients to be considered in calculation
        :return: gradient vector
        """

        gradients_attractive = []
        gradients_repulsive = []
        vector_attraction = Vector3()
        vector_repulsion = Vector3()

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._static.keys()

        for fid in frameids:
            if fid in self._static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        if element.attraction == 1:
                            gradients_attractive.append(element)
                        elif element.attraction == -1:
                            gradients_repulsive.append(element)

        if gradients_attractive:
            for gradient in gradients_attractive:
                # sum up all attractive gradients
                grad = self._calc_attractive_gradient(gradient)
                vector_attraction.x += grad.x
                vector_attraction.y += grad.y
                vector_attraction.z += grad.z

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                grad = self._calc_repulsive_gradient(gradient)
                # robot position is within obstacle radius, inf can't be
                # handled as direction
                # --> add vector which brings robot to the boarder of the
                # obstacle
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length (goal_radius +
                    # gradient.diffusion)
                    tmp = np.random.rand(1, 3)
                    tmp /= np.linalg.norm(tmp)
                    tmp *= (gradient.goal_radius + gradient.diffusion)

                    vector_repulsion.x += (2 * np.random.randint(2) - 1) * \
                                          tmp[0][0]
                    vector_repulsion.y += (2 * np.random.randint(2) - 1) * \
                                          tmp[0][1]
                    vector_repulsion.z += (2 * np.random.randint(2) - 1) * \
                                          tmp[0][2]
                else:
                    vector_repulsion.x += grad.x
                    vector_repulsion.y += grad.y
                    vector_repulsion.z += grad.z

        vector_attraction.x += vector_repulsion.x
        vector_attraction.y += vector_repulsion.y
        vector_attraction.z += vector_repulsion.z

        return vector_attraction

    def _aggregate_avoid_all(self, frameids=[]):
        """
        calculate vector which avoids all gradients within view distance
        :param pose: current pose of agent
        :param frameids: frameIDs of gradients to be considered in calculation
        :return gradient vector
        """
        gradients = []
        v = Vector3()

        # if no frameids are specified, use all data stored in buffer
        if not frameids:
            frameids = self._static.keys()

        for fid in frameids:
            if fid in self._static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        gradients.append(element)

        if gradients:
            for gradient in gradients:
                grad = self._calc_repulsive_gradient(gradient)
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length (goal_radius +
                    # gradient.diffusion)
                    tmp = np.random.rand(1, 3)
                    tmp /= np.linalg.norm(tmp)
                    tmp *= (gradient.goal_radius + gradient.diffusion)

                    v.x += (2 * np.random.randint(2) - 1) * tmp[0][0]
                    v.y += (2 * np.random.randint(2) - 1) * tmp[0][1]
                    v.z += (2 * np.random.randint(2) - 1) * tmp[0][2]
                else:
                    v.x += grad.x
                    v.y += grad.y
                    v.z += grad.z

        return v

    # EVAPORATION
    def _evaporate_buffer(self):
        """
        evaporate buffer data stored in self._static
        neighbor data is not evaporated as it is considered being fixed
        :return:
        """
        # interate through keys
        for fid in self._static:
            if self._static[fid]:  # array not empty
                for i in xrange(len(self._static[fid]) - 1, -1,
                                -1):  # go in reverse order
                    if self._static[fid][i].ev_time > 0:
                        diff = rospy.Time.now() - self._static[fid][
                            i].header.stamp
                        if diff >= rospy.Duration(self._static[fid][i].ev_time):
                            n = diff.secs // self._static[fid][i].ev_time
                            self._static[fid][i].diffusion *= self._static[fid][
                                                                i].ev_factor \
                                                            ** n
                            self._static[fid][i].header.stamp += rospy.Duration(
                                n * self._static[fid][i].ev_time)
                    else:  # delta t for evaporation = 0 and evaporation
                        # applies, set diffusion immediately to 0
                        if self._static[fid][i].ev_factor < 1.0:
                            self._static[fid][i].diffusion = 0.0

                    # in case that gradient concentration is lower than
                    # minimum and no goal_radius exists, delete data
                    if self._static[fid][i].goal_radius == 0.0 and \
                                    self._static[fid][
                                        i].diffusion < self._min_diffusion:
                        del self._static[fid][i]  # remove element

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
                msg.header.stamp += rospy.Duration(n * msg.ev_time)
        else:  # delta t for evaporation = 0 and evaporation applies,
            # set diffusion immediately to 0
            if msg.ev_factor < 1.0:
                msg.diffusion = 0

        if msg.diffusion >= self._min_diffusion or msg.goal_radius != 0.0:
            return msg

    # Potential field calculations based on Balch and Hybinette Paper
    # (doi:10.1109/ROBOT.2000.844042)
    def _calc_attractive_gradient(self, gradient):
        """
        :param gradient: position of the goal
        :param pose: position of the robot
        :return: attractive vector / norm vector
        """
        v = Vector3()

        # distance goal - agent
        tmp = Vector3()
        tmp.x = gradient.p.x - self._own_pos[-1].p.x
        tmp.y = gradient.p.y - self._own_pos[-1].p.y
        tmp.z = gradient.p.z - self._own_pos[-1].p.z

        # shortest distance considered (goal_radius of agent == size of agent)
        d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) \
            - self._own_pos[-1].goal_radius

        if d <= gradient.goal_radius:
            v.x = 0
            v.y = 0
            v.z = 0
        elif gradient.goal_radius < d <= gradient.goal_radius + \
                gradient.diffusion:
            # calculate magnitude of vector
            magnitude = (d - gradient.goal_radius) / gradient.diffusion
            v.x = magnitude * (tmp.x / d)
            v.y = magnitude * (tmp.y / d)
            v.z = magnitude * (tmp.z / d)
        elif d > gradient.goal_radius + gradient.diffusion:
            # calculate attraction vector
            v.x = 1.0 * (tmp.x / d)
            v.y = 1.0 * (tmp.y / d)
            v.z = 1.0 * (tmp.z / d)

        return v

    def _calc_repulsive_gradient(self, gradient):
        """
        :param gradient: position of the goal
        :param pose: position of the robot
        :return: repulsive vector
        """
        v = Vector3()

        # distance goal - agent
        tmp = Vector3()
        tmp.x = self._own_pos[-1].p.x - gradient.p.x
        tmp.y = self._own_pos[-1].p.y - gradient.p.y
        tmp.z = self._own_pos[-1].p.z - gradient.p.z

        # shortest distance considered (goal_radius of agent == size of agent)
        d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) \
            - self._own_pos[-1].goal_radius

        if d <= gradient.goal_radius:  # infinitely large repulsion
            v = Vector3(np.inf, np.inf, np.inf)
            # calculate norm vector for direction
            if d != 0:
                tmp.x /= d
                tmp.y /= d
                tmp.z /= d
            # calculate repulsion vector / adjust sign/direction
            if tmp.x != 0.0:
                v.x *= tmp.x
            if tmp.y != 0.0:
                v.y *= tmp.y
            if tmp.z != 0.0:
                v.z *= tmp.z

        elif gradient.goal_radius < d <= gradient.diffusion + \
                gradient.goal_radius:
            # calculate norm vector for direction
            tmp.x /= d
            tmp.y /= d
            tmp.z /= d
            # calculate magnitude of vector
            magnitude = (
                        gradient.diffusion + gradient.goal_radius - d) / \
                        gradient.diffusion
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
    def _calc_attractive_gradient_ge(self, gradient):
        """
        calculate attractive gradient based on Ge & Cui - no normalization of
        vectors!
        normalized version same as _calc_attractive_gradient
        :param gradient: position of the goal
        :param pose: position of the robot
        :return: attractive vector
        """
        v = Vector3()

        # distance goal - agent
        tmp = Vector3()
        tmp.x = gradient.p.x - self._own_pos[-1].p.x
        tmp.y = gradient.p.y - self._own_pos[-1].p.y
        tmp.z = gradient.p.z - self._own_pos[-1].p.z

        # shortest distance considered (goal_radius of agent == size of agent)
        d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) \
            - self._own_pos[-1].goal_radius

        if d <= gradient.goal_radius + gradient.diffusion:
            v.x = tmp.x
            v.y = tmp.y
            v.z = tmp.z
        elif d > gradient.goal_radius + gradient.diffusion:
            v.x = (tmp.x / d) * (gradient.goal_radius + gradient.diffusion)
            v.y = (tmp.y / d) * (gradient.goal_radius + gradient.diffusion)
            v.z = (tmp.z / d) * (gradient.goal_radius + gradient.diffusion)

        return v

    def _calc_repulsive_gradient_ge(self, gradient, goal):
        """
        :param gradient: position of the goal
        :param pose: position of the robot
        :return: repulsive vector
        distance of influence of obstacle = goal_radius + diffusion
        """
        v = Vector3()

        # distance obstacle - agent
        tmp = Vector3()
        tmp.x = self._own_pos[-1].p.x - gradient.p.x
        tmp.y = self._own_pos[-1].p.y - gradient.p.y
        tmp.z = self._own_pos[-1].p.z - gradient.p.z

        # shortest distance considered (goal_radius of agent == size of agent)
        d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) \
            - self._own_pos[-1].goal_radius

        if d <= gradient.goal_radius:
            v = Vector3(np.inf, np.inf, np.inf)
            # calculate norm vector for direction
            if d != 0.0:
                tmp.x /= d
                tmp.y /= d
                tmp.z /= d
            # calculate repulsion vector - adjust sign/direction
            if tmp.x != 0.0:
                v.x *= tmp.x
            if tmp.y != 0.0:
                v.y *= tmp.y
            if tmp.z != 0.0:
                v.z *= tmp.z
        elif gradient.goal_radius < d <= gradient.goal_radius + \
                gradient.diffusion:
            # unit vector obstacle - agent
            tmp.x /= d
            tmp.y /= d
            tmp.z /= d
            # distance repulsive gradient - goal
            d_goal = calc.get_gradient_distance(self._own_pos[-1].p, goal.p)
            # unit vector agent - goal
            ag = Vector3()
            ag.x = (goal.p.x - self._own_pos[-1].p.x) / d_goal
            ag.y = (goal.p.y - self._own_pos[-1].p.y) / d_goal
            ag.z = (goal.p.z - self._own_pos[-1].p.z) / d_goal
            # closest distance to obstacle  - diffusion
            d_obs_diff = (1.0 / (d - gradient.goal_radius)) - (
            1.0 / gradient.diffusion)
            # parameters
            n = 1.0
            eta = 1.0
            # weighting rep1 and rep2
            f_rep1 = eta * d_obs_diff * (
            (d_goal ** n) / ((d - gradient.goal_radius) ** n))
            f_rep2 = eta * (n / 2.0) * np.square(d_obs_diff) * (
            d_goal ** (n - 1))
            v.x = f_rep1 * tmp.x + f_rep2 * ag.x
            v.y = f_rep1 * tmp.y + f_rep2 * ag.y
            v.z = f_rep1 * tmp.z + f_rep2 * ag.z
        elif d > gradient.goal_radius + gradient.diffusion:
            v.x = 0
            v.y = 0
            v.z = 0

        return v

    # QUORUM SENSING: DENSITY FUNCTION
    def quorum(self):
        """
        calculates agent density within view
        :return: True (threshold passed), False (threshold not passed)
        """
        count = 0
        if self.quorum_moving:
            if self._moving:
                for val in self._moving.values():  # returns list
                                                # of neighbor positions
                    # check if neighbor is in sight
                    if calc.get_gradient_distance(val[-1].p,
                                                  self._own_pos[-1].p) <= val[
                        -1].diffusion + val[-1].goal_radius \
                            + self._view_distance:
                        count += 1.0
        if self.quorum_static:
            for fid in self._static:
                if self._static[fid]:
                    for val in self._static[fid]:  # returns list
                        # of neighbor positions
                        # check if neighbor is in sight
                        if calc.get_gradient_distance(val.p,
                                                      self._own_pos[-1].p) <= \
                                                val.diffusion + \
                                                val.goal_radius \
                                        + self._view_distance:
                            count += 1.0

        if count >= self.threshold:
                return True
        else:
            return False

    def quorum_list(self):
            """
            returns gradients within view
            :return: list
            """
            view = []

            if self.quorum_moving:
                if self._moving:
                    for val in self._moving.values():  # returns list
                        # of neighbor positions
                        # check if neighbor is in sight
                        if calc.get_gradient_distance(val[-1].p,
                                                      self._own_pos[-1].p) <= \
                                                val[
                                                    -1].diffusion + val[
                                            -1].goal_radius \
                                        + self._view_distance:
                            view.append(val[-1])

            if self.quorum_static:
                for fid in self._static:
                    if self._static[fid]:
                        for val in self._static[fid]:  # returns list
                            # of neighbor positions
                            # check if neighbor is in sight
                            if calc.get_gradient_distance(val.p,
                                                          self._own_pos[-1
                                                          ].p) <= \
                                                    val.diffusion + \
                                                    val.goal_radius \
                                            + self._view_distance:
                                view.append(val)

            return view

    # FLOCKING
    # TODO: unittest
    def flocking(self):
        """

        :return:
        """
        view = []

        # own position - we need minimum two values to calculate velocities
        if len(self._own_pos) >= 2:
            pose = self._own_pos[-1].p
        else:
            return

        # neighbors of agent
        if self._moving:
            for val in self._moving:
                # check if neighbor is in sight
                if calc.get_gradient_distance(self._moving[val][-1].p,
                                              pose) <= self._moving[val][
                    -1].diffusion + self._moving[val][-1].goal_radius \
                        + self._view_distance:
                    view.append(self._moving[val])

        # create array of tuples with neighbor position - neighbor velocity &
        # own pos & velocity (p, v)
        Boid = collections.namedtuple('Boid', ['p', 'v'])

        agent = Boid(self._own_pos[-1].p,
                     flocking.agent_velocity(self._own_pos[-1],
                                             self._own_pos[-2]))

        neighbors = []
        for neighbor in view:
            if len(neighbor) >= 2:  # at least 2 datapoints are available
                neighbors.append(Boid(neighbor[-1].p,
                                      flocking.agent_velocity(neighbor[-1],
                                                              neighbor[-2])))

        if self._own_pos:
            repulsion_radius = self._own_pos[-1].diffusion + self._own_pos[
                -1].goal_radius
        else:
            repulsion_radius = self._view_distance

        # calculate new velocity based on steering force
        # find out how to, probably like this:
        # velocity to be set = current vel + flocking steering force

        acceleration = flocking.flocking_vector(neighbors, agent,
                                                         self.epsilon,
                                                         self.a, self.b,
                                                         repulsion_radius,
                                                         self._view_distance,
                                                         self.h)

        if calc.vector_length(acceleration) > self.max_acceleration:
            acceleration = calc.unit_vector3(acceleration)
            acceleration.x *= self.max_acceleration
            acceleration.y *= self.max_acceleration
            acceleration.z *= self.max_acceleration

        velocity = calc.add_vectors(agent.v, acceleration)

        if calc.vector_length(velocity) > self.max_velocity:
            velocity = calc.unit_vector3(velocity)
            velocity.x *= self.max_velocity
            velocity.y *= self.max_velocity
            velocity.z *= self.max_velocity

        return velocity
