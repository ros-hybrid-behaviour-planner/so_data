"""
Created on 16.02.2017

@author: kaiser

Module including sample implementations of decision patterns
"""

from patterns import DecisionPattern
import so_data.calc
import rospy
import numpy as np
from diagnostic_msgs.msg import KeyValue
from so_data.msg import SoMessage
from so_data.gradientnode import create_gradient


class Morphogenesis(DecisionPattern):
    """
    Find barycenter of robot group
    """
    def __init__(self, buffer, frame, key, moving=True, static=False,
                 goal_radius=0.5, ev_factor=1.0, ev_time=0.0, attraction=-1):

        super(Morphogenesis, self).__init__(buffer, frame, key, moving,
                                            static)

        self.last_decision = 'None'

        # TODO evtl blueprint fuer gradient einfuegen oder sowas
        self.goal_radius = goal_radius
        self.ev_factor = ev_factor
        self.ev_time = ev_time
        self.attraction = attraction

    def value(self):
        """
        sums up distance to all morphogenetic gradients
        :return: distance (float)
        """

        list = self._buffer.decision_list(self.frame, moving=self.moving,
                                                static=self.static)
        own_pos = self._buffer.get_own_pose()

        if not list:
            return -1

        # determine summed up distances to neighbors
        dist = 0
        for el in list:
            d = so_data.calc.get_gradient_distance(own_pos.p, el.p)
            dist += d

        # determine whether own agent is gradient
        # true if sum of distances is smallest compared to neighbors
        neighbors = 0
        count = 0
        for el in list:
            neighbors += 1

            # sum of distances of neighbor
            keys = [i.key for i in el.payload]
            index = keys.index(self.key)
            ndist = float(el.payload[index].value)
            # neighbor dist larger than own dist
            if ndist > dist:
                count += 1

        # set state
        if neighbors != 0 and count == neighbors:
            self.state = 'Center'
        else:
            self.state = 'None'

        return dist

    def spread(self):

        # get current summed up distance & set values for use in sensor
        self.last_value = self.value()
        self.last_decision = self.state

        # spread morphogenetic message
        msg = SoMessage()
        msg.header.frame_id = self.frame
        msg.parent_frame = self.get_id()

        now = rospy.Time.now()
        msg.header.stamp = now
        msg.ev_stamp = now

        current_pose = self._buffer.get_own_pose()
        msg.p = current_pose.p
        msg.q = current_pose.q
        msg.attraction = self.attraction

        # set diffusion to inf s.t. all gradients sense morphogenetic gradient
        msg.diffusion = np.inf

        msg.goal_radius = self.goal_radius
        msg.ev_factor = self.ev_factor
        msg.ev_time = self.ev_time

        msg.moving = True  # set to moving as morph gradient is tied to agent
        msg.payload.append(KeyValue(self.key, "%.9f" % self.last_value))

        # spread morphogenetic gradient
        self._broadcaster.send_data(msg)

        # if barycenter: spread gradient for chemotaxis
        if self.state == 'Center':
            rospy.loginfo("Agent state: Center")
            center_gradient = create_gradient(current_pose.p, goal_radius=2.0,
                                              attraction=1.0, diffusion=20,
                                              moving=False,
                                              frameid=self.frame)

            self._broadcaster.send_data(center_gradient)


class Gossip(DecisionPattern):
    """
    find maximum value
    """
    def __init__(self, buffer, frame, key, initial_value=1,
                 moving=True, static=False):

        super(Gossip, self).__init__(buffer, frame, key, moving, static)

        self._value = initial_value
        self.last_value = -1
        self._list = None

    def value(self):

        self._list = self._buffer.decision_list(self.frame, moving=self.moving,
                                                static=self.static)

        for el in self._list:
            k = [i.key for i in el.payload]
            index = k.index(self.key)

            tmp = float(el.payload[index].value)
            if self._value < tmp:
               self._value = tmp

        return self._value



    # QUORUM SENSING: DENSITY FUNCTION
    # def quorum(self):
    #     """
    #     calculates agent density within view; only considers agent data
    #     :return: True (threshold passed), False (threshold not passed)
    #     """
    #     count = 0
    #
    #     if self.pose_frame in self._moving.keys():
    #         for pid in self._moving[self.pose_frame].keys():
    #             if self._moving[self.pose_frame][pid]:
    #                 val = self._moving[self.pose_frame][pid][-1]
    #                 # check if neighbor is in sight
    #                 if calc.get_gradient_distance(val.p, self._own_pos[-1].p) \
    #                     <= val.diffusion + val.goal_radius + \
    #                     self._view_distance:
    #                     count += 1
    #
    #     if count >= self.threshold:
    #         return True
    #     else:
    #         return False
