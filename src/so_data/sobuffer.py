""" Created on 07.11.2016

@author: kaiser

Module for receiving, storing and manipulating gradient data
offers basic functionalities: receiving (spreading), evaporation, aggregation,
                              gradients
"""

from __future__ import division

import copy
import numpy as np
import random
import threading
import yaml

import rospy
import tf.transformations
from geometry_msgs.msg import Vector3

import calc
import gradient
from so_data.msg import SoMessage


class AGGREGATION(object):
    """
    Enumeration specifying aggregation options
    * min = keep gradients with minimum diffusion radius + goal radius
    * max = keep gradients with maximum diffusion radius + goal radius
    * avg = combine gradients / average
    * new = store newest received gradient
    * newparent = store newest received gradient per parent frame
    * newframe = stores last received message per header frame
    """
    MIN = 'min'
    MAX = 'max'
    AVG = 'avg'
    NEW = 'new'
    NEWPARENT = 'newparent'
    NEWFRAME = 'newframe'


class SoBuffer(object):
    """
    This class is the buffer for received self-organization data
    """

    def __init__(self, aggregation=None, aggregation_distance=1.0,
                 min_diffusion=0.1, view_distance=1.5, id='',
                 moving_storage_size=2, store_all=True, framestorage=None,
                 pose_frame='robot', ev_thread=False, ev_time=1):

        """
        :param aggregation: indicator which kind of aggregation should be
        applied per frameID at a gradient center / within aggregation_distance
        of gradient center. "DEFAULT" used for gradients without own
        aggregation option.
        :type aggregation: dictionary - key:frameID
              value: aggregation option (enum AGGREGATION)

        :param aggregation_distance: radius in which gradient data is
        aggregated
        :type aggregation_distance: float

        :param min_diffusion: threshold, gradients with smaller diffusion
        radius will be deleted (only if goal_radius == 0)
        :type min_diffusion: float

        :param view_distance: radius in which agent can sense gradients;
        should be >= goal_radius of own gradient
        :type view_distance: float

        :param id: agent's id, gradients with this id are stored in
        self._own_pos
        :type id: str

        :param moving_storage_size: how many gradient messages per moving
         gradient will be stored, set 0 not to store any neighbor gradients
        :type moving_storage_size: int [0, inf]

        :param store_all: defines whether all frameIDs will be stored or only
                    the frameIDs defined in framestorage
        :type store_all: bool

        :param framestorage: list of frame IDs which should be stored,
                        applies both for moving and for static gradients
                options: * [] = no gradients will be stored
                         * [key1, key2, ...] = only gradients which have one of
                          the specified frame IDs will be stored
                         * 'None' has to be specified as the key for gradients
                         without frameID
        :type framestorage: list of strings

        :param pose_frame: frame which indicates agent data (neighbors)
        :type pose_frame: str.

        :param ev_thread: trigger evaporation by thread
        :type ev_thread: bool

        :param ev_time: delta t in sec when evaporation should be triggered
        :type ev_time: float
        """

        # lock for evaporation
        self._listeners = []
        self.last_id = 0
        self.lock = threading.Lock()

        # STORE DATA
        self._static = {}  # static gradient storage, dict
        self._own_pos = []  # own positions storage
        self._moving = {}  # moving gradients storage, dict of dicts

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

        # frame specifying agent / neighbor data
        self.pose_frame = pose_frame
        self._id = id  # own ID
        self._moving_storage_size = moving_storage_size

        # view distance of agent
        self._view_distance = view_distance

        # threading
        self.ev_thread = ev_thread
        self.ev_time = ev_time
        if ev_thread:
            self._evaporate_buffer()

        self.init_subscriber()

    def init_subscriber(self):
        rospy.Subscriber('so_data', SoMessage, self._store_data_callback)

    def _store_data_callback(self, msg):
        """
        store received soMessage using evaporation and aggregation
        :param msg: received gradient (soMessage)
        """
        deserialize_msg_payload(msg)

        self.store_data(msg, rospy.Time.now())

    def store_data(self, msg, time):
        """
        store received soMessage using evaporation and aggregation
        :param msg: received gradient (soMessage)
        :param time: ros time stamp when the msg was received
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
                if msg.header.frame_id != self.pose_frame:
                    return
                elif msg.parent_frame != self.id:
                    return

        # Evaporation
        # evaporate stored data
        if not self.ev_thread:
            self._evaporate_buffer(time=time)

        # evaporate received data
        msg = self._evaporate_msg(msg, time)

        if not msg:  # evaporation let to disappearance of the message
            return

        with self.lock:
            # store own position and neighbor / moving agents data
            if msg.moving:
                self.store_moving(msg)
            # aggregate and store static gradient data
            else:
                self.store_static(msg)

        for frames, callback in self._listeners:
            if msg.header.frame_id in frames:
                callback(msg)

    def store_moving(self, msg):
        """
        method to store moving gradients
        :param msg: SoMessage to be stored
        """
        if self._moving_storage_size > 0:
            # own position data
            if self._id and msg.header.frame_id == self.pose_frame and \
                            msg.parent_frame == self._id:
                last_position = self.get_last_position()
                # check if data is newer
                if last_position is None or msg.header.stamp > last_position.header.stamp:
                    self._own_pos.append(msg)
                # maximum length of stored own gradients exceeded
                if len(self._own_pos) > self._moving_storage_size:
                    del self._own_pos[0]

            # neighbor data
            elif msg.header.frame_id in self._moving:
                if msg.parent_frame in self._moving[msg.header.frame_id]:
                    # check if data is newer
                    if msg.header.stamp > \
                            self._moving[msg.header.frame_id] \
                                    [msg.parent_frame][-1].header.stamp:
                        self._moving[msg.header.frame_id][msg.parent_frame]. \
                            append(msg)
                    # maximum length of stored neighbor gradients exceeded
                    if len(self._moving[msg.header.frame_id][msg.parent_frame]) \
                            > self._moving_storage_size:
                        del self._moving[msg.header.frame_id][msg.parent_frame][0]
                else:
                    self._moving[msg.header.frame_id][msg.parent_frame] = [msg]
            else:
                self._moving[msg.header.frame_id] = {}
                self._moving[msg.header.frame_id][msg.parent_frame] = [msg]

    def get_last_position(self):
        if len(self.own_pos) == 0:
            return None

        pos_ = self.own_pos[-1]
        return copy.deepcopy(pos_)

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
            elif aggregation == AGGREGATION.NEWFRAME:
                self.aggregation_newframe(msg)

            # aggregation based on position
            else:
                # indicates whether data point at same position / within
                # aggregation radius is already stored
                view = {}

                # find all points which are in aggregation range of message
                for i in xrange(len(self._static[msg.header.frame_id]) - 1, -1, -1):
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
        :param frame_id: frame ID for which aggregation option is searched
        :return: aggregation option
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

    def aggregation_newframe(self, msg):
        """
        stores newest message per frame ID
        :param msg: received gradient message
        """

        if msg.header.stamp >= self._static[msg.header.frame_id][-1].\
            header.stamp:
            self._static[msg.header.frame_id] = [msg]

    def aggregation_newparent(self, msg):
        """
        stores newest message per parent frame ID
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

    def get_own_pose(self):
        """
        :return: robots last position
        """
        if self._own_pos:
            return self.get_last_position()
        else:
            return

    def gradients(self, frameids=None, static=True, moving=True, time=None):
        """
        function determines all gradients within view distance
        :param frameids: frame IDs to be considered looking for gradients, None considers all frameids.
        :param static: consider static gradients
        :param moving: consider moving gradients
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: list of gradients []
        """
        if not self.ev_thread:
            self._evaporate_buffer(time=time)

        gradients = []

        # if view distance is infinite, return list of all gradients
        if self._view_distance == np.inf:
            return self.all_gradients(frameids, static, moving)

        # determine frames to consider
        if not frameids:
            frameids = []
            if static:
                frameids += self._static.keys()
            if moving:
                frameids += self._moving.keys()

        pose = self.get_last_position()

        # no own position available
        if pose is None:
            return gradients

        if static:
            staticbuffer = copy.deepcopy(self._static)
        if moving:
            movingbuffer = copy.deepcopy(self._moving)

        for fid in frameids:
            # static gradients
            if static and fid in staticbuffer.keys():
                for element in staticbuffer[fid]:
                    if calc.get_gradient_distance(element.p, pose.p) <= \
                                            element.diffusion + \
                                            element.goal_radius + \
                                    self._view_distance:
                            gradients.append(element)

            # moving gradients
            if moving and fid in movingbuffer.keys() \
                    and movingbuffer[fid]:
                for pid in movingbuffer[fid].keys():
                    if movingbuffer[fid][pid]:
                        element = movingbuffer[fid][pid][-1]
                        if calc.get_gradient_distance(element.p, pose.p) \
                            <= element.diffusion + element.goal_radius + \
                            self._view_distance:
                            gradients.append(element)

        return gradients

    def all_gradients(self, frameids=None, static=True, moving=True, time=None):
        """
        function returns all gradients as a list
        :param frameids: frame IDs to be considered looking for gradients, if None all static and moving are considered
                         if static and/or moving are True
        :param static: consider static gradients
        :param moving: consider moving gradients
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: list of gradients
        """
        # determine frames to consider
        if not frameids:
            frameids = []
            if static:
                frameids += self._static.keys()
            if moving:
                frameids += self._moving.keys()

        if not self.ev_thread:
            self._evaporate_buffer(time=time)

        gradients = []

        if static:
            staticbuffer = copy.deepcopy(self._static)
        if moving:
            movingbuffer = copy.deepcopy(self._moving)

        for fid in frameids:
            if static and fid in staticbuffer.keys():
                for element in staticbuffer[fid]:
                    gradients.append(element)

            # moving gradients
            if moving and fid in movingbuffer.keys() \
                    and movingbuffer[fid]:
                for pid in movingbuffer[fid].keys():
                    if movingbuffer[fid][pid]:
                        gradients.append(movingbuffer[fid][pid][-1])

        return gradients

    def repulsive_gradients(self, frameids=None, static=True, moving=True, time=None):
        """
        function determines which repulsive gradients are currently within
         view distance
        :param frameids: frame IDs to be considered looking for repulsive
        gradients
        :param static: consider static gradients
        :param moving: consider moving gradients
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: list of repulsive gradients within view distance
        """
        # check if moving and / or static attractive gradients are
        # within view distance

        if not self.ev_thread:
            self._evaporate_buffer(time=time)

        gradients_repulsive = []

        pose = self.get_last_position()

        # no own position available
        if pose is None:
            return []

        if not frameids:
            frameids = []
            if static:
                frameids += self._static.keys()
            if moving:
                frameids += self._moving.keys()

        if static:
            staticbuffer = copy.deepcopy(self._static)
        if moving:
            movingbuffer = copy.deepcopy(self._moving)

        for fid in frameids:
            # static gradients
            if static and fid in staticbuffer.keys():
                for element in staticbuffer[fid]:
                    if calc.get_gradient_distance(element.p, pose.p) <= \
                                            element.diffusion + \
                                            element.goal_radius + \
                                            self._view_distance:
                        if element.attraction == -1:
                            gradients_repulsive.append(element)

            # moving gradients
            if moving and fid in movingbuffer.keys() \
                    and movingbuffer[fid]:
                for pid in movingbuffer[fid].keys():
                    if movingbuffer[fid][pid]:
                        element = movingbuffer[fid][pid][-1]
                        if calc.get_gradient_distance(element.p, pose.p) \
                            <= element.diffusion + element.goal_radius + \
                               self._view_distance:
                            if element.attraction == -1:
                                gradients_repulsive.append(element)

        return gradients_repulsive

    def attractive_gradients(self, frameids=None, static=True, moving=True, time=None):
        """
        function determines which attractive gradients are currently within
         view distance
        :param frameids: frame IDs to be considered looking for attractive
                         gradients
        :param static: consider static gradients
        :param moving: consider moving gradients
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: list of attractive gradients within view distance
        """
        # check if moving and / or static attractive gradients are
        # within view distance
        if not self.ev_thread:
            self._evaporate_buffer(time=time)

        gradients_attractive = []

        pose = self.get_last_position()

        if pose is None:
            return gradients_attractive

        # determine frames to consider
        if not frameids:
            frameids = []
            if static:
                frameids += self._static.keys()
            if moving:
                frameids += self._moving.keys()

        if static:
            staticbuffer = copy.deepcopy(self._static)
        if moving:
            movingbuffer = copy.deepcopy(self._moving)

        for fid in frameids:
            # static gradients
            if static and fid in staticbuffer.keys():
                for element in staticbuffer[fid]:
                    if calc.get_gradient_distance(element.p, pose.p) <= \
                                            element.diffusion + \
                                            element.goal_radius + \
                                    self._view_distance:
                        if element.attraction == 1:
                            gradients_attractive.append(element)

            # moving gradients
            if moving and fid in movingbuffer.keys():
                for pid in movingbuffer[fid].keys():
                    if movingbuffer[fid][pid]:
                        element = movingbuffer[fid][pid][-1]
                        if calc.get_gradient_distance(element.p, pose.p) \
                            <= element.diffusion + element.goal_radius + \
                            self._view_distance:
                            if element.attraction == 1:
                                gradients_attractive.append(element)

        return gradients_attractive

    def max_attractive_gradient(self, frameids=None, static=True, moving=True, time=None):
        """
        Method to return relatively nearest attractive gradient
        (= min movement vector length)
        based on attraction values of Balch & Hybinette
        :param frameids: frames to consider
        :param static: consider static gradients
        :param moving: consider moving gradients
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: relatively nearest attractive gradient (Vector3)
        """

        pose = self.get_last_position()

        if pose is None:
            return []

        gradients = self.attractive_gradients(frameids, static, moving, time=time)
        tmp_grad = None
        tmp_att = np.inf

        if gradients:
            for grad in gradients:
                g = gradient.calc_attractive_gradient(grad, pose)
                att = calc.vector_length(g)
                # attraction decreases with being closer to gradient source
                # / goal area
                if att < tmp_att:
                    tmp_grad = grad
                    tmp_att = att

        return tmp_grad

    def min_attractive_gradient(self, frameids=None, static=True, moving=True, time=None):
        """
        Method to return relatively furthest gradient
        (= maximum attraction value length)
        based on attraction values of Balch & Hybinette
        :param frameids: frames to consider
        :param static: consider static gradients
        :param moving: consider moving gradients
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: gradient (Vector3)
        """

        pose = self.get_last_position()

        if pose is None:
            return []

        gradients = self.attractive_gradients(frameids, static, moving, time=time)
        tmp_grad = None
        tmp_att = 0

        if gradients:
            for grad in gradients:
                # gradient reach
                g = gradient.calc_attractive_gradient(grad, pose)
                att = calc.vector_length(g)
                if att > tmp_att:
                    tmp_grad = grad
                    tmp_att = att

        return tmp_grad

    def min_reach_attractive_gradient(self, frameids=None, static=True, moving=True, time= None):
        """
        Method to return gradient with minimum gradient reach
        :param frameids: frames to consider
        :param static: consider static gradients
        :param moving: consider moving gradients
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: gradient (Vector3)
        """

        if self.get_last_position() is None:
            return []

        gradients = self.attractive_gradients(frameids, static, moving, time=time)
        tmp_grad = None
        tmp_att = np.inf

        if gradients:
            for grad in gradients:
                # gradient reach
                att = grad.diffusion + grad.goal_radius
                if att < tmp_att:
                    tmp_grad = grad
                    tmp_att = att

        return tmp_grad

    def max_reach_attractive_gradient(self, frameids=None, static=True, moving=True, time=None):
        """
        Method to return gradient with maximum gradient reach
        :param frameids: frames to consider
        :param static: consider static gradients
        :param moving: consider moving gradients
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: gradient (Vector3)
        """

        if self.get_last_position() is None:
            return []

        gradients = self.attractive_gradients(frameids, static, moving, time)
        tmp_grad = None
        tmp_att = 0

        if gradients:
            for grad in gradients:
                # gradient reach
                att = grad.diffusion + grad.goal_radius
                if att > tmp_att:
                    tmp_grad = grad
                    tmp_att = att

        return tmp_grad

    def strongest_gradient(self, frameids=None, static=True, moving=True, time=None):
        """
        return gradient with strongest potential on robot
        :param frameids: frames to consider
        :param static: bool, consider static gradients
        :param moving: bool, consider moving gradients
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: soMessage gradient
        """
        tmp_grad = None

        if self.get_last_position() is None:
            return tmp_grad

        # all gradients within view distance
        gradients = self.gradients(frameids, static, moving, time=time)
        tmp_att = -1

        if gradients:
            for grad in gradients:
                if grad.attraction == 1:
                    g = gradient.calc_attractive_gradient(grad, self.get_last_position())
                else:
                    g = gradient.calc_repulsive_gradient(grad, self.get_last_position())

                if abs(g.x) == np.inf or abs(g.y) == np.inf or \
                                abs(g.z) == np.inf:
                    att = np.inf
                else:
                    att = calc.vector_length(g)
                    # attractive potential is defined inverse to repulsive
                    # potential --> adjust attractive value for comparison
                    if grad.attraction == 1:
                        att = 1 - att

                if att > tmp_att:
                    tmp_grad = grad
                    tmp_att = att

        return tmp_grad

    # Aggregation of data for Decision patterns
    def agent_list(self, frameids=None, time=None):
        """
        function determines all moving gradients within view distance with a
        certain frame ID, excluding all gradients from agent itself
        :param frame: frame ID of agent data
        :param moving: consider moving gradients
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: list of gradients
        """
        if not self.ev_thread:
            self._evaporate_buffer(time=time)

        gradients = []

        pose = self.get_last_position()

        # no own position available
        if pose is None:
            return gradients

        # determine frames to consider
        if not frameids:
            frameids = self._moving.keys()

        moving = copy.deepcopy(self._moving)

        for frame in frameids:
            if frame in moving.keys() and moving[frame]:
                for pid in moving[frame].keys():
                    if pid != self._id and moving[frame][pid]:
                        if calc.get_gradient_distance(moving[frame][pid][-1].p,
                                                      pose.p) \
                                <= moving[frame][pid][-1].diffusion + \
                                   moving[frame][pid][-1].goal_radius + \
                                   self._view_distance:
                            gradients.append(moving[frame][pid][-1])

        return gradients

    # Aggregation of data for Decision patterns
    def static_list_angle(self, frameids, view_angle_xy=None, view_angle_yz=np.pi, time=None):
        """
        function determines all static gradients within view distance with a
        certain frame ID & within a view angle,
        only static gradients as pheromones are deposited in environment,
        :param frameids: frame ID of agent data
        :param view_angle_xy: angle in which agent can see pheromones on
        x-y-plane (+/- from heading)
        :param view_angle_yz: angle in which agent can see pheromones on
        y-z-plane (+/- from heading)
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: list of gradients
        """
        if not self.ev_thread:
            self._evaporate_buffer(time=time)

        gradients = []

        # no own position available
        pose = self.get_last_position()

        if pose is None:
            rospy.logerr("No own positions available")
            return gradients

        # determine frames to consider
        if not frameids:
            frameids = self._static.keys()

        # heading vector
        heading = tf.transformations.quaternion_matrix([pose.q.x, pose.q.y,
                                                        pose.q.z, pose.q.w]).\
            dot([pose.direction.x, pose.direction.y, pose.direction.z, 1])

        heading = Vector3(heading[0], heading[1], heading[2])

        static = copy.deepcopy(self._static)

        for frame in frameids:
            if frame in static.keys():
                for element in static[frame]:
                    grad = calc.delta_vector(element.p, pose.p)
                    if calc.vector_length(grad) <= element.diffusion + element.goal_radius + self._view_distance:
                        if view_angle_xy is None or np.abs(calc.angle_between([grad.x, grad.y],
                                                     [heading.x, heading.y])) \
                                <= view_angle_xy:
                            if view_angle_xy is None or np.abs(calc.angle_between([grad.y, grad.z],
                                                         [heading.y,
                                                          heading.z])) \
                                    <= view_angle_yz:
                                gradients.append(element)

        return gradients

    # Aggregation of data for Decision patterns
    def agent_set(self, frameids=None, include_own=True, time=None):
        """
        function determines all moving gradients within view distance with a
        certain frame ID, excluding all gradients from agent itself
        :param frame: frame ID of agent data
        :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
        :return: list of gradients
        """
        if not self.ev_thread:
            self._evaporate_buffer(time=time)

        gradients = []

        pose = self.get_last_position()

        # no own position available
        if pose is None:
            return gradients

        # determine frames to consider
        if not frameids:
            frameids = self._moving.keys()

        moving = self._moving
        moving_keys = moving.keys()

        for frame in frameids:

            if frame in moving_keys and moving[frame]:

                moving_frame_keys = moving[frame].keys()
                for pid in moving_frame_keys:

                    if (include_own or pid != self._id) and moving[frame][pid]:

                        for g in moving[frame][pid]:

                            if calc.get_gradient_distance(g.p, pose.p) <= \
                                    g.diffusion + g.goal_radius + self._view_distance:

                                gradients.append(g)

        return gradients

    def register_listener(self, frames, callback):
        tuple = (frames, callback)
        self._listeners.append(tuple)
    # EVAPORATION
    def _evaporate_buffer(self, msg=None, time=None):
        """
        evaporate buffer data
        :param time: time stamp that is used as current time for the evaporation, if None rospy.Time.now() is used.
        :return:
        """
        if time is None:
            time = rospy.Time.now()

        if self.ev_thread:
            t = threading.Timer(self.ev_time, self._evaporate_buffer)
            t.daemon = True
            t.start()

        with self.lock:
            # iterate through keys
            for fid in self._static.keys():
                if self._static[fid]:  # array not empty
                    # go in reverse order
                    for i in xrange(len(self._static[fid]) - 1, -1, -1):
                        if self._static[fid][i].ev_time > 0 and \
                                        self._static[fid][i].ev_factor != 1.0:
                            diff = time - self._static[fid][
                                i].ev_stamp
                            if diff >= rospy.Duration(
                                    self._static[fid][i].ev_time):
                                n = diff.secs // self._static[fid][i].ev_time
                                self._static[fid][i].diffusion *= \
                                    self._static[fid][i].ev_factor ** n
                                self._static[fid][i].ev_stamp += rospy.\
                                    Duration(n * self._static[fid][i].ev_time)
                        else:  # delta t for evaporation = 0 and evaporation
                            # applies, set diffusion immediately to 0
                            if self._static[fid][i].ev_factor < 1.0:
                                self._static[fid][i].diffusion = 0.0

                        # in case that gradient concentration is lower than
                        # minimum and no goal_radius exists, delete data
                        if self._static[fid][i].goal_radius == 0.0 and \
                                        self._static[fid][i].diffusion < \
                                        self._min_diffusion:
                            del self._static[fid][i]  # remove element

            for fid in self._moving.keys():
                if self._moving[fid]:
                    for pid in self._moving[fid].keys():
                        for i in xrange(len(self._moving[fid][pid]) - 1,
                                        -1, -1):
                            if self._moving[fid][pid][i].ev_time > 0 and \
                                    self._moving[fid][pid][i].ev_factor != 1.0:
                                diff = time - self._moving[fid][
                                    pid][i].ev_stamp
                                if diff >= rospy.Duration(
                                        self._moving[fid][pid][i].ev_time):
                                    n = diff.secs // self._moving[fid][pid][
                                        i].ev_time
                                    self._moving[fid][pid][i].diffusion *= \
                                        self._moving[fid][pid][i].ev_factor \
                                        ** n
                                    self._moving[fid][pid][i].ev_stamp += \
                                        rospy.Duration(n * self._moving[fid][
                                            pid][i].ev_time)
                            else:  # delta t for evaporation = 0 and evaporation
                                # applies, set diffusion immediately to 0
                                if self._moving[fid][pid][i].ev_factor < 1.0:
                                    self._moving[fid][pid][i].diffusion = 0.0

                                # in case that gradient concentration is lower
                                    # than minimum and no goal_radius exists,
                                    # delete data
                            if self._moving[fid][pid][i].goal_radius == 0.0 \
                                    and self._moving[fid][pid][i].diffusion < \
                                        self._min_diffusion:
                                del self._moving[fid][pid][i]  # remove element



    def _evaporate_msg(self, msg, time):
        """
        evaporate a single message
        :param msg: gradient message
        :param time: time stamp that is used to determine the evaporation in respect to the msg ev_stamps
        :return: evaporated message
        """
        if msg.ev_time > 0:
            diff = time - msg.ev_stamp
            if diff >= rospy.Duration(msg.ev_time):
                n = diff.secs // msg.ev_time
                msg.diffusion *= msg.ev_factor ** n
                msg.ev_stamp += rospy.Duration(n * msg.ev_time)
        else:  # delta t for evaporation = 0 and evaporation applies,
            # set diffusion immediately to 0
            if msg.ev_factor < 1.0:
                msg.diffusion = 0

        return msg

    @property
    def id(self):
        return self._id

    @property
    def view_distance(self):
        return self._view_distance

    @property
    def own_pos(self):
        return self._own_pos

    def reset_buffer(self):
        self._static = {}
        self._own_pos = []
        self._moving = {}


def deserialize_msg_payload(msg):
    """
    This method deserializes the payload values of the payload KeyValue elements and returns the converted msg
    Attention the msg is altered in this method
    :param msg: so message with not yet deserialized values in the payload
    :type msg: SoMessage
    :return: altered msg with deserialized payload
    """
    # TODO for some reason the messages are sometimes not serialized here, couldn't find the problem!?
    # Following check avoids problems in such cases.
    if isinstance(msg.payload, str):
        msg.payload = yaml.load(msg.payload)
    else:
        rospy.logdebug("Trying to deserialize not serialized payload. Msg: %s", str(msg))
    return msg


def serialize_msg_payload(msg):
    """
    This method serializes the payload values of the payload KeyValue elements and returns the converted msg
    Attention the msg is altered in this method
    :param msg: so message with not yet serialized values in the payload
    :type msg: SoMessage
    :return: altered msg with serialized payload
    """
    msg.payload = yaml.dump(msg.payload)
    return msg
