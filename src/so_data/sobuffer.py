""" Created on 07.11.2016
.. module:: soBuffer
.. moduleauthor:: kaiser

Module for receiving, storing and manipulating gradient data
"""

from __future__ import \
    division  # force floating point division when using plain /
import rospy
from so_data.msg import SoMessage
import numpy as np
import calc
from geometry_msgs.msg import Vector3
import flocking
import flockingrey
import collections
import random


# ENUMERATIONS
class RESULT(object):
    """
    Enumeration specifying result options for movement patterns
    * near = return vector to nearest attractive vector avoiding obstacles
    * all = return vector considering all vectors of potential field
    * max = return vector with max attraction / repulsion
    * reach = return vector which enables to reach nearest attractive gradient
    * avoid = return vector leading away form all gradients
    * flocking: return flocking vector based on Olfati-Saber
    * flockingrey: return flocking vector based on Reynolds
    * collision = return vector avoiding all repulsive gradients
    """
    NEAR = 0
    ALL = 1
    MAX = 2
    REACH = 3
    AVOID = 4
    FLOCKING = 5
    FLOCKINGREY = 6
    COLLISION = 7


class DECISION(object):
    """
    Enumeration specifying result options for decision patterns
    * morph: return list for morphogenesis pattern (morphogenesis frame)
    * gossip: return list for gossip pattern (gossip frame)
    * quorum: return list for quorum pattern (pose frame of neighbors)
    """
    MORPH = 0
    GOSSIP = 1
    QUORUM = 2


class AGGREGATION(object):
    """
    Enumeration specifying aggregation options
    * min = keep gradients with minimum diffusion radius + goal radius
    * max = keep gradients with maximum diffusion radius + goal radius
    * avg = combine gradients / average
    * new = store newest received gradient
    * newparent = store neweste received gradient per parent frame
    """
    MIN = 0
    MAX = 1
    AVG = 2
    NEW = 3
    NEWPARENT = 4


class REPULSION(object):
    """
    Enumeration specifying collision avoidance options
    * gradient = potential field approach to realize collision avoidance
                between neighbors
    * repulsion = repulsion vector calculation based on Fernandez-Marquez et al.
    * reach = moving vectors are considered for repulsive vector calculation
              in Ge & Cui approach (only for result == reach)
    """
    GRADIENT = 0
    REPULSION = 1
    REACH = 2


class STATE(object):
    """
    Enumeration containing states for morphogenesis
    * None = no state
    * center = robot is barycenter of its group of robots
    """
    NONE = "None"
    CENTER = "Center"
    RED = "red"
    BLUE = "blue"


class SoBuffer(object):
    """
    This class is the buffer for received self-organization data
    """

    def __init__(self, aggregation=None, aggregation_distance=1.0,
                 min_diffusion=0.1, view_distance=1.5, id='', result=None,
                 repulsion=None, moving_storage_size=2,
                 store_all=True, framestorage=None, threshold=2, a=1.0,
                 b=1.0, h=0.5, epsilon=1.0, max_acceleration=1.0,
                 max_velocity=1.0, result_moving=True, result_static=True,
                 min_velocity=0.1, state=STATE.NONE, key=None,
                 morph_frame='morphogenesis', pose_frame='robot',
                 chem_frames=None, gossip_frame='gossip', gossip_key='',
                 decision=None, phase=None):

        """
        :param aggregation: indicator which kind of aggregation should be
        applied per frameID at a gradient center / within aggregation_distance
        of gradient center. "DEFAULT" used for gradients without own
        aggregation option.
        :type aggregation: dictionary - key:
            frameID value: aggregation option (enum AGGREGATION)

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
        :type result: enum (RESULT)

        :param collision_avoidance: avoidance of neighbors
        :type collision_avoidance: enum (COLLISION)

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

        :param result_moving: consider moving gradients (True) or not (False)
                            in quorum decision
        :type result_moving: bool

        :param result_static: consider static gradients (True) or not (False)
                            in quorum decision
        :type result_static: bool

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

        # handle mutable parameters
        if framestorage is None:
            self._frames = []
        else:
            self._frames = framestorage

        if aggregation is None:
            self._aggregation = {'DEFAULT': AGGREGATION.MAX}
        else:
            self._aggregation = aggregation

        self._aggregation_distance = aggregation_distance
        self._min_diffusion = min_diffusion
        self._id = id
        # frame specifying agent / neighbor data
        self._pose_frame = pose_frame
        self._moving_storage_size = moving_storage_size

        # RETURN AGGREGATED DATA
        self._view_distance = view_distance
        if result is None:
            self.result = [RESULT.NEAR]
        else:
            self.result = result
        self.result_moving = result_moving
        self.result_static = result_static

        # Collision Avoidance
        self.repulsion = repulsion

        # quorum
        self.threshold = threshold

        # flocking
        self.a = a
        self.b = b
        self.h = h
        self.epsilon = epsilon
        self.max_acceleration = max_acceleration
        self.max_velocity = max_velocity

        self.min_velocity = min_velocity

        self.decision = decision

        # morphogenesis
        self.state = state
        self._morph_values = {}
        # payload data key
        self.key = key
        # frame ID for morphogenetic gradients
        self.morph_frame = morph_frame

        # chemotaxis frames
        if chem_frames is None:
            self.chem_frames = []
        else:
            self.chem_frames = chem_frames

        # gossip
        self.gossip_frame = gossip_frame
        self.gossip_key = gossip_key
        self.gossip_values = {}

        # indicate in which phase robot is
        self.phase = phase

        rospy.Subscriber('so_data', SoMessage, self.store_data)

    def store_data(self, msg):
        """
        store received soMessage using evaporation and aggregation
        :param msg: received gradient (soMessage)
        :return:
        """

        # store msgs with no frame id with dictionary key 'None'
        if not msg.header.frame_id:
            msg.header.frame_id = 'None'

        if not msg.parent_frame:
            msg.parent_frame = 'None'

        # check if received msg should be stored
        if not self._store_all:
            if msg.header.frame_id not in self._frames:
                return

        # Evaporation
        # evaporate stored data
        self._evaporate_buffer()
        # evaporate received data
        msg = self._evaporate_msg(msg)
        if not msg:  # evaporation let to disappearance of the message
            return

        # store own position and neighbor / moving agents data
        if msg.moving:
            self.store_moving(msg)
        # aggregate and store static gradient data
        else:
            self.store_static(msg)

    def store_moving(self, msg):
        """
        method to store moving gradients
        :param msg: SoMessage to be stored
        """
        if self._moving_storage_size > 0:
            # own position data
            if self._id and msg.header.frame_id == self._pose_frame and \
                            msg.parent_frame == self._id:
                # check if data is newer
                if self._own_pos:  # own data already stored
                    if msg.header.stamp > self._own_pos[-1].header.stamp:
                        self._own_pos.append(msg)
                else:  # no own position stored so far
                    self._own_pos.append(msg)
                # maximum length of stored own gradients exceeded
                if len(self._own_pos) > self._moving_storage_size:
                    del self._own_pos[0]
            # neighbor data
            elif msg.header.frame_id in self._moving:
                if msg.parent_frame in self._moving[msg.header.frame_id]:
                    # check if data is newer
                    if msg.header.stamp > \
                            self._moving[msg.header.frame_id][msg.parent_frame][-1].header.stamp:
                        self._moving[msg.header.frame_id][msg.parent_frame].append(msg)
                    # maximum length of stored neighbor gradients exceeded
                    if len(self._moving[msg.header.frame_id][msg.parent_frame]) > \
                            self._moving_storage_size:
                        del self._moving[msg.header.frame_id][msg.parent_frame][0]
                else:
                    self._moving[msg.header.frame_id][msg.parent_frame] = [msg]
            else:
                self._moving[msg.header.frame_id] = {}
                self._moving[msg.header.frame_id][msg.parent_frame] = [msg]

    def store_static(self, msg):
        """
        method to store static gradients
        :param msg: received SoMessage
        """
        # set aggregation option
        aggregation = self.aggregation_option(msg.header.frame_id)
        if aggregation is None:
            return

        if msg.header.frame_id in self._static:

            # aggregation based on parent frame
            if aggregation == AGGREGATION.NEWPARENT:
                self.aggregation_newparent(msg)

            # aggregation based on position
            else:
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

                # find minimum distance to gradient centers within aggregation
                # distance
                if view:
                    min_val = min(view.values())  # get minimum distance
                    result = [j for j, v in view.items() if v == min_val]

                    # several with same distance - random value of list
                    if len(result) > 1:
                        k = random.choice(result)
                    # only one minimum - use found index
                    else:
                        k = result[0]

                    # keep data with max reach
                    if aggregation == AGGREGATION.MAX:
                        self.aggregation_max(msg, k)
                    # keep data with min reach
                    elif aggregation == AGGREGATION.MIN:
                        self.aggregation_min(msg, k)
                    # keep average gradient
                    elif aggregation == AGGREGATION.AVG:
                        self.aggregation_average(msg, k)
                    # keep last received gradient at one position
                    elif aggregation == AGGREGATION.NEW:
                        self.aggregation_new(msg, k)
                else:
                    self._static[msg.header.frame_id].append(msg)
        else:
            self._static[msg.header.frame_id] = [msg]

    def aggregation_option(self, frame_id):
        """
        determines aggregation option to be used
        :param frame_id:
        :return:
        """
        # set aggregation option for received message based on frameID
        if frame_id in self._aggregation.keys():
            return self._aggregation[frame_id]
        elif 'DEFAULT' in self._aggregation.keys():  # only default value
            # is specified
            return self._aggregation['DEFAULT']
        else:
            rospy.logerr("No DEFAULT value specified for aggregation!")
            return

    def aggregation_newparent(self, msg):
        """
        stores newest message per parent
        :param msg: received gradient message
        """
        found = False
        for i in range(0, len(self._static[msg.header.frame_id])):
            if self._static[msg.header.frame_id][i].parent_frame == \
                    msg.parent_frame:
                found = True
                if msg.header.stamp >= \
                        self._static[msg.header.frame_id][i].header.stamp:
                    del self._static[msg.header.frame_id][i]
                    self._static[msg.header.frame_id].append(msg)
                    return

        if not found:
            self._static[msg.header.frame_id].append(msg)

    def aggregation_new(self, msg, k):
        """
        stores newest gradient within aggregation distance in self._static
        :param msg: received gradient message
        :param k: index of closest data point within aggregation distance
        """
        if msg.header.stamp >= self._static[msg.header.frame_id][
            k].header.stamp:
            del self._static[msg.header.frame_id][k]
            self._static[msg.header.frame_id].append(msg)

    def aggregation_min(self, msg, k):
        """
        stores min gradient within aggregation distance in self._static
        :param msg: received gradient message
        :param k: index of closest data point within aggregation distance
        """
        if msg.diffusion + msg.goal_radius <= \
                        self._static[msg.header.frame_id][k].diffusion + \
                        self._static[msg.header.frame_id][k].goal_radius:
            del self._static[msg.header.frame_id][k]
            self._static[msg.header.frame_id].append(msg)

    def aggregation_max(self, msg, k):
        """
        stores max gradient within aggregation distance in self._static
        :param msg: received gradient message
        :param k: index of closest data point within aggregation distance
        """
        if msg.diffusion + msg.goal_radius >= \
                        self._static[msg.header.frame_id][k].diffusion + \
                        self._static[msg.header.frame_id][k].goal_radius:
            del self._static[msg.header.frame_id][k]
            self._static[msg.header.frame_id].append(msg)

    def aggregation_average(self, msg, k):
        """
        stores average gradient in self._static
        :param msg: received gradient message
        :param k: index of closest data point within aggregation distance
        """
        # attraction is the same direction
        if msg.attraction == self._static[msg.header.frame_id][k].attraction:
            msg.diffusion = (msg.diffusion + self._static[msg.header.frame_id][
                k].diffusion) / 2.0
            msg.goal_radius = (msg.goal_radius +
                               self._static[msg.header.frame_id][
                                   k].goal_radius) / 2.0
        else:
            # change sign
            if self._static[msg.header.frame_id][k].diffusion + \
                    self._static[msg.header.frame_id][k].goal_radius \
                    > msg.diffusion + msg.goal_radius:
                msg.attraction *= -1
            msg.diffusion = np.absolute(msg.diffusion -
                                        self._static[msg.header.frame_id][k].
                                        diffusion)
            msg.goal_radius = np.absolute(msg.goal_radius -
                                          self._static[msg.header.frame_id][k].
                                          goal_radius)

        msg.p.x = (msg.p.x + self._static[msg.header.frame_id][k].p.x) / 2.0
        msg.p.y = (msg.p.y + self._static[msg.header.frame_id][k].p.y) / 2.0
        msg.p.z = (msg.p.z + self._static[msg.header.frame_id][k].p.z) / 2.0
        del self._static[msg.header.frame_id][k]
        # store average element as long as it has a goal radius or the
        # diffusion radius is larger than the minimum required diffusion
        if msg.diffusion >= self._min_diffusion or msg.goal_radius != 0.0:
            self._static[msg.header.frame_id].append(msg)

    def get_current_gradient(self):
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
        if RESULT.NEAR in self.result:
            result = calc.add_vectors(result, self.result_near())
        if RESULT.MAX in self.result:
            result = calc.add_vectors(result, self.result_max())
        if RESULT.ALL in self.result:
            result = calc.add_vectors(result, self.result_all())
        if RESULT.REACH in self.result:
            result = calc.add_vectors(result, self.result_reach())
        if RESULT.AVOID in self.result:
            result = calc.add_vectors(result, self.result_avoid())
        if RESULT.FLOCKING in self.result:
            result = calc.add_vectors(result, self.result_flocking())
        if RESULT.FLOCKINGREY in self.result:
            result = calc.add_vectors(result, self.result_flockingrey())
        if RESULT.COLLISION in self.result:
            result = calc.add_vectors(result, self.result_collision())

        # collision avoidance / consider neighbor gradients
        if self.repulsion == REPULSION.GRADIENT:
            result = calc.add_vectors(result, self.repulsion_gradient())
        elif self.repulsion == REPULSION.REPULSION:
            collision = self.repulsion_repulsion()
            result = calc.add_vectors(result, collision)

        # adjust length to be max within view_distance
        d = calc.vector_length(result)
        if d > self.max_velocity:
            result = calc.adjust_length(result, self.max_velocity)
        elif 0 < d < self.min_velocity:
            result = calc.adjust_length(result, self.min_velocity)

        return result

    def get_attractive_gradients_view(self):
        """
        :return: True (attractive gradient within view),
                False (no attractive gradients within view)
        """
        # if no frameids are specified, use all data stored in buffer
        frameids = []
        if not self.chem_frames:
            if self.result_static:
                frameids += self._static.keys()
            if self.result_moving:
                frameids += self._moving.keys()
        else:
            frameids = self.chem_frames

        # check if moving and / or static attractive gradients are
        # within view distance
        for fid in frameids:
            if fid in self._static and self.result_static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        if element.attraction == 1:
                            return True

            if self.result_moving and fid in self._moving:
                for pid in self._moving[fid]:
                    if calc.get_gradient_distance(self._moving[fid][pid][-1].p,
                                                  self._own_pos[-1].p) \
                            <= self._moving[fid][pid][-1].diffusion + \
                                    self._moving[fid][pid][
                                        -1].goal_radius + self._view_distance:
                        if self._moving[fid][pid][-1].attraction == 1:
                            return True

        return False

    def get_own_pose(self):
        """
        :return: robots last position
        """
        return self._own_pos[-1]

    def attractive_gradients(self, frameids):
        """
        function determines which attractive gradients are currently within
         view distance
        :param frameids: frame IDs to be considered looking for attractive
        gradients
        :return: list of attractive gradients within view distance
        """
        # check if moving and / or static attractive gradients are
        # within view distance
        gradients_attractive = []

        if not self._own_pos:
            return gradients_attractive

        for fid in frameids:
            # static gradients
            if fid in self._static and self.result_static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        if element.attraction == 1:
                            gradients_attractive.append(element)

            # moving gradients
            if self.result_moving and fid in self._moving:
                for pid in self._moving[fid].keys():
                    if calc.get_gradient_distance(self._moving[fid][pid][-1].p,
                                                  self._own_pos[-1].p) \
                            <= self._moving[fid][pid][-1].diffusion + \
                                    self._moving[fid][pid][
                                        -1].goal_radius + self._view_distance:
                        if self._moving[fid][pid][-1].attraction == 1:
                            gradients_attractive.append(self._moving[fid][pid]
                                                        [-1])

        return gradients_attractive

    def repulsive_gradients(self, frameids):
        """
        function determines which repulsive gradients are currently within
         view distance
        :param frameids: frame IDs to be considered looking for repulsive
        gradients
        :return: list of repulsive gradients within view distance
        """
        # check if moving and / or static attractive gradients are
        # within view distance
        gradients_repulsive = []
        for fid in frameids:
            # static gradients
            if fid in self._static and self.result_static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        if element.attraction == -1:
                            gradients_repulsive.append(element)

            # moving gradients
            if self.result_moving and fid in self._moving and self._moving[
                fid]:
                if calc.get_gradient_distance(self._moving[fid][-1].p,
                                              self._own_pos[-1].p) \
                        <= self._moving[fid][-1].diffusion + \
                                self._moving[fid][
                                    -1].goal_radius + self._view_distance:
                    if self._moving[fid][-1].attraction == 1:
                        gradients_repulsive.append(self._moving[fid][-1])

        return gradients_repulsive

    def gradients(self, frameids):
        """
        function determines all gradients within view distance
        :param frameids: frame IDs to be considered looking for gradients
        :return: list of gradients
        """
        gradients_repulsive = []
        gradients_attractive = []

        # no own position available
        if not self._own_pos:
            return [[],[]]

        for fid in frameids:
            # static gradients
            if fid in self._static and self.result_static:
                for element in self._static[fid]:
                    if calc.get_gradient_distance(element.p, self._own_pos[
                        -1].p) <= element.diffusion + \
                            element.goal_radius + self._view_distance:
                        if element.attraction == -1:
                            gradients_repulsive.append(element)
                        else:
                            gradients_attractive.append(element)

            # moving gradients
            if self.result_moving and fid in self._moving and \
                    self._moving[fid]:
                for pid in self._moving[fid].keys():
                    if calc.get_gradient_distance(self._moving[fid][pid][-1].p,
                                                 self._own_pos[-1].p) \
                            <= self._moving[fid][pid][-1].diffusion + \
                                    self._moving[fid][pid][
                                        -1].goal_radius + self._view_distance:
                        if self._moving[fid][pid][-1].attraction == -1:
                            gradients_repulsive.append(self._moving[fid][pid][-1])
                        else:
                            gradients_attractive.append(self._moving[fid][pid][-1])

        return [gradients_attractive, gradients_repulsive]

    def get_goal_reached(self):
        """
        determines whether nearest attractive gradient was reached - especially
        for return == reach option
        returns True in case that gradient was reached
        False otherwise
        :return: True/False (bool)
        """
        tmp_att = np.inf  # attractive gradient calculation return values [0,1]
        frameids = []

        # if no frameids are specified, use all data stored in buffer
        if not self.chem_frames:
            if self.result_static:
                frameids += self._static.keys()
            if self.result_moving:
                frameids += self._moving.keys()
        else:
            frameids = self.chem_frames

        gradients_attractive = self.attractive_gradients(frameids)

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

    def get_attractive_distance(self):
        """
        :return: returns distance to closest attractive gradient
        (min attraction)
        no attractive gradients: returns np.inf
        """
        tmp_att = np.inf  # attractive gradient calculations return values
        # between 0 and 1
        frameids = []

        # if no frameids are specified, use all data stored in buffer
        if not self.chem_frames:
            if self.result_static:
                frameids += self._static.keys()
            if self.result_moving:
                frameids += self._moving.keys()
        else:
            frameids = self.chem_frames

        gradients_attractive = self.attractive_gradients(frameids)

        tmp_grad = SoMessage()
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

    def get_attraction_bool(self):
        """
        determines whether there is still some attraction/repulsion
        :return: True (potential) /False (no potential) (bool)
        """

        # calculate gradient vector
        vector = self.get_current_gradient()
        d = calc.vector_length(vector)

        # no gradient vector to follow --> "goal reached / repulsion avoided"
        if d == 0.0:
            return False
        else:
            return True

    def get_neighbors_bool(self):
        """
        :return: True (no neighbors within view / not leading to repulsive
        behaviour), False (neighbors within view / leading to repulsive
        behaviour)
        """
        flag = True

        d = 0.0

        if self.repulsion == REPULSION.REPULSION:
            d = calc.vector_length(self.repulsion_repulsion())
        elif self.repulsion == REPULSION.GRADIENT:
            d = calc.vector_length(self.repulsion_gradient())

        if d > 0:
            flag = False

        return flag

    # Collision avoidance between neighbors
    def repulsion_gradient(self):
        """
        returns repulsion vector (collision avoidance between neighbors)
         based on potential field approach
        considers all neighbours that have a gradient reaching inside view
        distance / communication range of agent
        considers only neighbor data of moving gradients (self._pose_frame)
        :return: repulsion vector
        """
        repulsion = Vector3()

        # no data available
        if not self._own_pos:
            return repulsion

        # repulsion radius of robot, <= view_distance w
        repulsion_radius = self._own_pos[-1].diffusion + self._own_pos[
            -1].goal_radius

        if self._pose_frame in self._moving:
            for pid in self._moving[self._pose_frame].keys():
                if self._moving[self._pose_frame][pid]:
                    val = self._moving[self._pose_frame][pid][-1]
                    # check if neighbor is in sight
                    if val and calc.get_gradient_distance(val.p,
                                                      self._own_pos[-1].p) \
                        <= val.diffusion + val.goal_radius \
                            + self._view_distance:
                            grad = self._calc_repulsive_gradient(val)

                    # two robots are at the same position
                    if grad.x == np.inf or grad.x == -1 * np.inf:
                        # create random vector with length=repulsion radius
                        dv = calc.delta_vector(self._own_pos[-1].p, val.p)

                        if calc.vector_length(dv) == 0:
                            repulsion = calc.add_vectors(repulsion,
                                                         calc.random_vector(
                                                             repulsion_radius))
                        else:
                            repulsion = calc.add_vectors(repulsion,
                                                         calc.adjust_length(
                                                             dv,
                                                             repulsion_radius))
                    else:
                        repulsion = calc.add_vectors(repulsion, grad)

        # limit repulsion vector length to repulsion radius
        d = calc.vector_length(repulsion)
        if d > repulsion_radius:
            repulsion = calc.adjust_length(repulsion, repulsion_radius)

        return repulsion

    def repulsion_repulsion(self):
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

        if self._pose_frame in self._moving.keys():
                for pid in self._moving[self._pose_frame].keys():
                    if self._moving[self._pose_frame][pid]:
                        val = self._moving[self._pose_frame][pid][-1]
                        # distance between agents shortest distance agent
                        # center to neighbor
                        distance = calc.get_gradient_distance(val.p,
                                        self._own_pos[-1].p) - val.goal_radius
                        # agents within view
                        if distance <= self._view_distance:
                            # only robots within repulsion
                            if distance > 0:
                                diff = repulsion_radius - distance
                                m.x += (self._own_pos[-1].p.x - val.p.x) * \
                                       diff / distance
                                m.y += (self._own_pos[-1].p.y - val.p.y) * \
                                       diff / distance
                                m.z += (self._own_pos[-1].p.z - val.p.z) * \
                                       diff / distance
                            elif distance <= 0:
                                # create random vector with length=
                                # repulsion radius
                                if distance == -val.goal_radius:
                                    m = calc.add_vectors(m, calc.random_vector(
                                                         repulsion_radius))
                                # use direction leading away if available
                                else:
                                    m = calc.add_vectors(m, calc.adjust_length(
                                        calc.delta_vector(self._own_pos[-1].p,
                                                          val.p),
                                        repulsion_radius))

        # max repulsion vector length = repulsion radius of robot
        d = np.linalg.norm([m.x, m.y, m.z])
        if d > repulsion_radius:
            m = calc.adjust_length(m, repulsion_radius)

        return m

    # AGGREGATION - build potential field (merging of information)
    def result_max(self):
        """
        follow higher gradient values (= gradient with shortest relative
        distance to gradient source)
        sets current gradient to direction vector (length <= 1)
        :return gradient vector
        """

        gradients = []
        tmp_att = -1
        tmp_grad = Vector3()
        frameids = []

        # if no frameids are specified, use all data stored in buffer
        if not self.chem_frames:
            if self.result_static:
                frameids += self._static.keys()
            if self.result_moving:
                frameids += self._moving.keys()
        else:
            frameids = self.chem_frames

        # find moving and / or static gradients within view distance
        gradients = self.gradients(frameids)
        gradients = gradients[0] + gradients[1]

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

                        dv = calc.delta_vector(self._own_pos[-1].p, gradient.p)

                        if calc.vector_length(dv) == 0:
                            tmp_grad = calc.random_vector(gradient.goal_radius
                                                  + gradient.diffusion)
                        else:
                            tmp_grad = calc.adjust_length(dv,
                                                          gradient.goal_radius
                                                          + gradient.diffusion)

                    else:
                        tmp_grad = grad

                    tmp_att = att

        return tmp_grad

    def result_collision(self):
        """
        aggregate gradients to avoid all repulsive gradients
        :return: repulsive gradient vector
        """
        vector_repulsion = Vector3()
        frameids = []

        # if no frameids are specified, use all data stored in buffer
        if not self.chem_frames:
            if self.result_static:
                frameids += self._static.keys()
            if self.result_moving:
                frameids += self._moving.keys()
        else:
            frameids = self.chem_frames

        gradients = self.repulsive_gradients(frameids)

        # aggregate repulsive gradients
        if gradients:
            for gradient in gradients:
                grad = self._calc_repulsive_gradient(gradient)
                # robot position is within obstacle radius, inf can't be
                # handled as direction
                # --> add vector which brings robot to the boarder of the
                # obstacle
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length (goal_radius +
                    # gradient.diffusion)
                    dv = calc.delta_vector(self._own_pos[-1].p, gradient.p)

                    if calc.vector_length(dv) == 0:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                calc.random_vector(
                                                    gradient.goal_radius +
                                                    gradient.diffusion))
                    else:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                    calc.adjust_length(dv,
                                                        gradient.goal_radius +
                                                        gradient.diffusion))
                else:
                    vector_repulsion = calc.add_vectors(vector_repulsion, grad)

        return vector_repulsion

    def result_near(self):
        """
        aggregate nearest attractive gradient with repulsive gradients s.t.
        robot finds gradient source avoiding the
        repulsive gradient sources
        :return gradient vector
        """

        vector_attraction = Vector3()
        vector_repulsion = Vector3()
        tmp_att = np.inf
        frameids = []

        # if no frameids are specified, use all data stored in buffer
        if not self.chem_frames:
            if self.result_static:
                frameids += self._static.keys()
            if self.result_moving:
                frameids += self._moving.keys()
        else:
            frameids = self.chem_frames

        # find attractive / repulsive gradients within view
        gradients = self.gradients(frameids)
        gradients_attractive = gradients[0]
        gradients_repulsive = gradients[1]

        if gradients_attractive:
            for gradient in gradients_attractive:
                # find nearest attractive gradient
                grad = self._calc_attractive_gradient(gradient)
                att = np.linalg.norm([grad.x, grad.y, grad.z])
                # attraction decreases with being closer to gradient source
                # / goal area
                if att < tmp_att:
                    vector_attraction = grad
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
                    dv = calc.delta_vector(self._own_pos[-1].p, gradient.p)

                    if calc.vector_length(dv) == 0:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                        calc.random_vector(
                                                        gradient.goal_radius
                                                        + gradient.diffusion))
                    else:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                            calc.adjust_length(
                                                                dv,
                                    gradient.goal_radius + gradient.diffusion))
                else:
                    vector_repulsion = calc.add_vectors(vector_repulsion, grad)

        return calc.add_vectors(vector_attraction, vector_repulsion)

    def result_reach(self):
        """
        aggregate nearest attractive gradient with repulsive gradients s.t.
        robot finds gradient source avoiding the
        repulsive gradient sources based on Ge & Cui
        requires at least one attractive gradient to be sensed
        :return gradient vector
        """
        vector_attraction = Vector3()
        vector_repulsion = Vector3()
        tmp_att = np.inf
        attractive_gradient = None
        frameids = []

        # if no frameids are specified, use all data stored in buffer
        if not self.chem_frames:
            if self.result_static:
                frameids += self._static.keys()
            if self.result_moving or \
                            self.repulsion == REPULSION.REACH:
                frameids += self._moving.keys()
        else:
            frameids = self.chem_frames

        # find gradients within view distance
        gradients = self.gradients(frameids)
        gradients_attractive = gradients[0]
        gradients_repulsive = gradients[1]

        # moving gradients not considered in general, but collision avoidance
        # should be applied: add moving repulsive gradients to gradient set
        if not self.result_moving and \
                        self.repulsion == REPULSION.REACH:
            if self._pose_frame in self._moving.keys():
                for pid in self._moving[self._pose_frame].keys():
                    if self._moving[self._pose_frame][pid]:
                        val = self._moving[self._pose_frame][pid][-1]
                        if calc.get_gradient_distance(val.p,
                                                      self._own_pos[-1].p) <= \
                                                val.diffusion + \
                                                val.goal_radius + \
                                        self._view_distance:
                            gradients_repulsive.append(val)

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
                    vector_attraction = grad
                    tmp_att = att
                    attractive_gradient = gradient

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                if attractive_gradient:
                    grad = self._calc_repulsive_gradient_ge(gradient,
                                                            attractive_gradient)
                else:  # no attractive gradient nearby, apply repulsion only
                    grad = self._calc_repulsive_gradient(gradient)

                # robot position is within obstacle goal radius, inf can't be
                # handled as direction
                # --> add vector which brings robot to the boarder of the
                # obstacle
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length (goal_radius +
                    # gradient.diffusion)
                    dv = calc.delta_vector(self._own_pos[-1].p, gradient.p)

                    if calc.vector_length(dv) == 0:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                            calc.random_vector(
                                                        gradient.goal_radius +
                                                        gradient.diffusion))
                    else:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                            calc.adjust_length(
                                                                dv,
                                    gradient.goal_radius + gradient.diffusion))
                else:
                    vector_repulsion = calc.add_vectors(vector_repulsion, grad)

        # return sum of gradients
        return calc.add_vectors(vector_attraction, vector_repulsion)

    def result_all(self):
        """
        aggregate all vectors within view distance
        :return: gradient vector
        """

        vector_attraction = Vector3()
        vector_repulsion = Vector3()
        frameids = []

        # if no frameids are specified, use all data stored in buffer
        if not self.chem_frames:
            if self.result_static:
                frameids += self._static.keys()
            if self.result_moving:
                frameids += self._moving.keys()
        else:
            frameids = self.chem_frames

        # find gradients within view distance
        gradients = self.gradients(frameids)
        gradients_attractive = gradients[0]
        gradients_repulsive = gradients[1]

        if gradients_attractive:
            for gradient in gradients_attractive:
                # sum up all attractive gradients
                grad = self._calc_attractive_gradient(gradient)
                vector_attraction = calc.add_vectors(vector_attraction, grad)

        # aggregate repulsive gradients
        if gradients_repulsive:
            for gradient in gradients_repulsive:
                grad = self._calc_repulsive_gradient(gradient)
                # robot position is within obstacle radius, inf can't be
                # handled as direction
                # --> add vector which brings robot to the boarder of the
                # obstacle
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length=goal radius + diffusion
                    dv = calc.delta_vector(self._own_pos[-1].p, gradient.p)

                    if calc.vector_length(dv) == 0:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                            calc.random_vector(
                                                        gradient.goal_radius
                                                        + gradient.diffusion))
                    else:
                        vector_repulsion = calc.add_vectors(vector_repulsion,
                                                            calc.adjust_length(
                                                                dv,
                                    gradient.goal_radius + gradient.diffusion))
                else:
                    vector_repulsion = calc.add_vectors(vector_repulsion, grad)

        return calc.add_vectors(vector_attraction, vector_repulsion)

    def result_avoid(self):
        """
        calculate vector which avoids all gradients within view distance
        :return gradient vector
        """
        gradients = []
        v = Vector3()
        frameids = []

        # if no frameids are specified, use all data stored in buffer
        if not self.chem_frames:
            if self.result_static:
                frameids += self._static.keys()
            if self.result_moving:
                frameids += self._moving.keys()
        else:
            frameids = self.chem_frames

        # find moving and / or static gradients within view distance
        gradients = self.gradients(frameids)
        gradients = gradients[0] + gradients[1]

        if gradients:
            for gradient in gradients:
                grad = self._calc_repulsive_gradient(gradient)
                if grad.x == np.inf or grad.x == -1 * np.inf:
                    # create random vector with length (goal_radius +
                    # gradient.diffusion)
                    dv = calc.delta_vector(self._own_pos[-1].p, gradient.p)

                    if calc.vector_length(dv) == 0:
                        v = calc.add_vectors(v, calc.random_vector(
                                            gradient.goal_radius
                                            + gradient.diffusion))
                    else:
                        v = calc.add_vectors(v, calc.adjust_length(dv,
                                    gradient.goal_radius + gradient.diffusion))

                else:
                    v = calc.add_vectors(v, grad)

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
                # go in reverse order
                for i in xrange(len(self._static[fid]) - 1, -1, -1):
                    if self._static[fid][i].ev_time > 0:
                        diff = rospy.Time.now() - self._static[fid][
                            i].ev_stamp
                        if diff >= rospy.Duration(
                                self._static[fid][i].ev_time):
                            n = diff.secs // self._static[fid][i].ev_time
                            self._static[fid][i].diffusion *= \
                                self._static[fid][i].ev_factor ** n
                            self._static[fid][i].ev_stamp += rospy.Duration(
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

        for fid in self._moving:
            if self._moving[fid]:
                for pid in self._moving[fid].keys():
                    for i in xrange(len(self._moving[fid][pid]) - 1, -1, -1):
                        if self._moving[fid][pid][i].ev_time > 0:
                            diff = rospy.Time.now() - self._moving[fid][pid][
                                i].ev_stamp
                            if diff >= rospy.Duration(
                                    self._moving[fid][pid][i].ev_time):
                                n = diff.secs // self._moving[fid][pid][i].ev_time
                                self._moving[fid][pid][i].diffusion *= \
                                    self._moving[fid][pid][i].ev_factor ** n
                                self._moving[fid][pid][i].ev_stamp += rospy.Duration(
                                    n * self._moving[fid][pid][i].ev_time)
                        else:  # delta t for evaporation = 0 and evaporation
                            # applies, set diffusion immediately to 0
                            if self._moving[fid][pid][i].ev_factor < 1.0:
                                self._moving[fid][pid][i].diffusion = 0.0

                            # in case that gradient concentration is lower than
                            # minimum and no goal_radius exists, delete data
                        if self._moving[fid][pid][i].goal_radius == 0.0 and \
                                        self._moving[fid][pid][i].diffusion < \
                                        self._min_diffusion:
                            del self._moving[fid][pid][i]  # remove element

    def _evaporate_msg(self, msg):
        """
        evaporate a single message
        :param msg: gradient message
        :return: evaporated message
        """
        if msg.ev_time > 0:
            diff = rospy.Time.now() - msg.ev_stamp
            if diff >= rospy.Duration(msg.ev_time):
                n = diff.secs // msg.ev_time
                msg.diffusion *= msg.ev_factor ** n
                msg.ev_stamp += rospy.Duration(n * msg.ev_time)
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
        tmp = calc.delta_vector(gradient.p, self._own_pos[-1].p)

        # shortest distance considered (goal_radius of agent == size of agent)
        d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) \
            - self._own_pos[-1].goal_radius

        if d <= gradient.goal_radius:
            v = Vector3()
        elif gradient.goal_radius < d <= gradient.goal_radius + \
                gradient.diffusion:
            # calculate magnitude of vector
            magnitude = (d - gradient.goal_radius) / gradient.diffusion
            v = calc.adjust_length(tmp, magnitude)
        elif d > gradient.goal_radius + gradient.diffusion:
            # calculate attraction vector
            v = calc.adjust_length(tmp, 1.0)

        return v

    def _calc_repulsive_gradient(self, gradient):
        """
        :param gradient: position of the goal
        :return: repulsive vector
        """
        v = Vector3()

        # distance goal - agent
        tmp = calc.delta_vector(self._own_pos[-1].p, gradient.p)

        # shortest distance considered (goal_radius of agent == size of agent)
        d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) \
            - self._own_pos[-1].goal_radius

        if d <= gradient.goal_radius:  # infinitely large repulsion
            v = Vector3(np.inf, np.inf, np.inf)
            # calculate norm vector for direction
            tmp = calc.unit_vector3(tmp)

            # calculate repulsion vector / adjust sign/direction
            if tmp.x != 0.0:
                v.x *= tmp.x
            if tmp.y != 0.0:
                v.y *= tmp.y
            if tmp.z != 0.0:
                v.z *= tmp.z

        elif gradient.goal_radius < d <= gradient.diffusion + \
                gradient.goal_radius:
            # calculate magnitude of vector
            magnitude = (gradient.diffusion + gradient.goal_radius - d) / \
                        gradient.diffusion
            # calculate vector
            v = calc.adjust_length(tmp, magnitude)

        elif d > gradient.diffusion + gradient.goal_radius:
            v = Vector3()

        return v

    # second version of gradients based on Ge & Cui
    def _calc_attractive_gradient_ge(self, gradient):
        """
        calculate attractive gradient based on Ge & Cui - no normalization of
        vectors!
        normalized version same as _calc_attractive_gradient
        :param gradient: position of the goal
        :return: attractive vector
        """
        v = Vector3()

        # distance goal - agent
        tmp = calc.delta_vector(gradient.p, self._own_pos[-1].p)

        # shortest distance considered (goal_radius of agent == size of agent)
        d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) \
            - self._own_pos[-1].goal_radius

        if d <= gradient.goal_radius:
            v = Vector3()
        elif gradient.goal_radius < d <= \
                        gradient.goal_radius + gradient.diffusion:
            v = tmp
        elif d > gradient.goal_radius + gradient.diffusion:
            v = calc.adjust_length(tmp,
                                   (gradient.goal_radius + gradient.diffusion))

        return v

    def _calc_repulsive_gradient_ge(self, gradient, goal):
        """
        :param gradient: position of the goal
        :param goal: attractive gradient to be followed
        :return: repulsive vector
        distance of influence of obstacle = goal_radius + diffusion
        """
        v = Vector3()

        # distance obstacle - agent
        tmp = calc.delta_vector(self._own_pos[-1].p, gradient.p)

        # shortest distance considered (goal_radius of agent == size of agent)
        d = np.linalg.norm([tmp.x, tmp.y, tmp.z]) \
            - self._own_pos[-1].goal_radius

        if d <= gradient.goal_radius:
            v = Vector3(np.inf, np.inf, np.inf)
            # calculate norm vector for direction
            tmp = calc.unit_vector3(tmp)

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
            tmp = calc.unit_vector3(tmp)
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
            v = Vector3()

        return v

    # FLOCKING
    def result_flocking(self):
        """
        flocking calculations based on Olfati-Saber
        :return: movement vector
        """
        view = []

        # own position - we need minimum two values to calculate velocities
        if len(self._own_pos) >= 2:
            pose = self._own_pos[-1].p
        else:
            return

        # neighbors of agent
        if self._moving:
            for val in self._moving.keys():
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
            acceleration = calc.adjust_length(acceleration,
                                              self.max_acceleration)

        velocity = calc.add_vectors(agent.v, acceleration)

        if calc.vector_length(velocity) > self.max_velocity:
            velocity = calc.adjust_length(velocity, self.max_velocity)

        return velocity

    def result_flockingrey(self):
        """
        flocking approach based on Reynolds
        :return: movement vector
        """
        view = []

        # at least one position of agent is available
        if len(self._own_pos) == 0:
            return Vector3()

        # neighbors of agent
        if self._pose_frame in self._moving.keys():
            for pid in self._moving[self._pose_frame].keys():
                if self._moving[self._pose_frame][pid]:
                    val = self._moving[self._pose_frame][pid][-1]
                    # check if neighbor is in sight
                    if calc.get_gradient_distance(val.p, self._own_pos[-1].p) \
                            <= val.diffusion + val.goal_radius \
                                    + self._view_distance:
                        view.append(val)

        # calculate flocking vector
        mov = flockingrey.separation(self._own_pos[-1], view)
        mov = calc.add_vectors(mov,
                               flockingrey.cohesion(self._own_pos[-1], view))
        mov = calc.add_vectors(mov,
                               flockingrey.alignment(self._own_pos[-1], view))

        # set maximum velocity
        if calc.vector_length(mov) > self.max_velocity:
            mov = calc.adjust_length(mov, self.max_velocity)

        return mov


    # TODO testing
    # Gossip & Morphogenesis & Quorum sensing

    def get_decision(self):

        result = None

        if self.decision == DECISION.GOSSIP:
            result = self.decision_list(self.gossip_frame)

            # store last used values for calculation
            self.gossip_values.clear()
            for el in result:
                self.gossip_values[el.parent_frame] = el

        elif self.decision == DECISION.MORPH:
            result = self.decision_list(self.morph_frame)

            # store last used values for calculation
            self._morph_values.clear()
            for el in result:
                self._morph_values[el.parent_frame] = el

        elif self.decision == DECISION.QUORUM:
            result = self.decision_list(self._pose_frame)

        return result

    def decision_list(self, frame):
        """
        returns list of gossip gradients within view, the one with own id is
        excluded
        :return: list of gradients
        """
        view = []
        frame = frame

        if frame in self._moving.keys():
            for pid in self._moving[frame]:
                if pid != self._id and self._moving[frame][pid]:
                    val = self._moving[frame][pid][-1]
                    d = calc.get_gradient_distance(self._own_pos[-1].p, val.p)
                    # gradient in sight
                    if d <= val.diffusion + val.goal_radius + \
                            self._view_distance:
                        view.append(val)
        return view

    # Gossip
    def gossip_changed(self):
        """
        returns true when no data is available / data has changed; false
        otherwise
        :return: bool
        """
        # no gossip data available --> start process
        if not self.gossip_frame in self._moving.keys():
            return True

        for pid in self._moving[self.gossip_frame]:
            # new value
            if pid != self._id and self._moving[self.gossip_frame][pid]:
                val = self._moving[self.gossip_frame][pid][-1]
                d = calc.get_gradient_distance(self._own_pos[-1].p, val.p)
                if d <= val.diffusion + val.goal_radius + self._view_distance:
                    # new value
                    if pid not in self.gossip_values.keys():
                        return True

                    keys = [i.key for i in self._moving[self.gossip_frame][pid]
                    [-1].payload]
                    index = keys.index(self.gossip_key)
                    ntmp = float(self._moving[self.gossip_frame][pid][-1].
                                 payload[index].value)

                    if float(self.gossip_values[pid].payload[index].value) \
                            != ntmp:
                        return True

        return False

    # Morphogenesis
    def morph_changed(self):
        """
        checks if morphogenetic data was changed
        :return: bool - True (data changed or no data available),
                        False (otherwise)
        """

        # no morph data available --> start process
        if self.morph_frame not in self._moving.keys():
            return True

        # update necessary when own position has changed
        if self._id in self._moving[self.morph_frame].keys():
            p = self._moving[self.morph_frame][self._id][-1].p
            p_cur = self._own_pos[-1].p
            if p.x != p_cur.x or p.y != p_cur.y or p.z != p_cur.z:
                return True

        # check if last used data has changed
        for pid in self._moving[self.morph_frame].keys():
            if pid != self._id:
                if pid not in self._morph_values.keys():
                    return True

                keys = [i.key for i in self._moving[self.morph_frame][pid][-1].
                        payload]
                index = keys.index(self.key)

                if index is not None and \
                                float(self._moving[self.morph_frame][pid][-1].
                                              payload[index].value) != \
                        float(self._morph_values[pid].payload[index].value):
                    return True

        # no data changed
        return False

    # QUORUM SENSING: DENSITY FUNCTION
    def quorum(self):
        """
        calculates agent density within view; only considers agent data
        :return: True (threshold passed), False (threshold not passed)
        """
        count = 0

        if self._pose_frame in self._moving.keys():
            for pid in self._moving[self._pose_frame].keys():
                if self._moving[self._pose_frame][pid]:
                    val = self._moving[self._pose_frame][pid][-1]
                    # check if neighbor is in sight
                    if calc.get_gradient_distance(val.p, self._own_pos[-1].p) \
                        <= val.diffusion + val.goal_radius + \
                        self._view_distance:
                        count += 1

        if count >= self.threshold:
            return True
        else:
            return False

    @property
    def id(self):
        return self._id
