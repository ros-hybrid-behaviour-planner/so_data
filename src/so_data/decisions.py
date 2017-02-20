"""
Created on 16.02.2017

@author: kaiser

Module including sample implementations of decision patterns: morphogenesis,
gossip
"""

from patterns import DecisionPattern
import so_data.calc
import rospy
import numpy as np
from so_data.gradientnode import create_gradient


class MorphogenesisBarycenter(DecisionPattern):
    """
    Find barycenter of robot group
    """
    def __init__(self, buffer, frame, key, moving=True, static=False,
                 goal_radius=0.5, ev_factor=1.0, ev_time=0.0, diffusion=np.inf,
                 attraction=-1, state='None', goal_center=2.0,
                 moving_center=False, attraction_center=1,
                 diffusion_center=20):

        super(MorphogenesisBarycenter, self).__init__(buffer, frame, key,
                                                      state, moving, static,
                                                      goal_radius, ev_factor,
                                                      ev_time, diffusion,
                                                      attraction)

        self.last_state = 'None'

        # Center gradient
        self.goal_center = goal_center
        self.moving_center = moving_center
        self.attraction_center = attraction_center
        self.diffusion_center = diffusion_center

    def value(self):
        """
        sums up distance to all morphogenetic gradients determines and
        sets state of robot
        :return: distance (float)
        """

        values = self._buffer.agent_list(self.frame, moving=self.moving,
                                         static=self.static)
        own_pos = self._buffer.get_own_pose()

        if not values or not own_pos:
            return -1

        # determine summed up distances to neighbors
        dist = 0
        for el in values:
            dist += so_data.calc.get_gradient_distance(own_pos.p, el.p)

        # determine whether own agent is gradient
        # true if sum of distances is smallest compared to neighbors
        neighbors = 0
        count = 0
        for el in values:
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
        """
        spreads morphogenetic gradient with sum of distances
        + spreads center gradient if robot is barycenter
        """
        super(MorphogenesisBarycenter, self).spread()
        self.last_state = self.state

        # if barycenter: spread gradient for chemotaxis
        if self.state == 'Center':
            rospy.loginfo("Agent state: Center")
            center_gradient = create_gradient(self.get_pos().p,
                                              goal_radius=self.goal_center,
                                              attraction=self.attraction_center,
                                              diffusion=self.diffusion_center,
                                              moving=self.moving_center,
                                              frameid=self.frame)

            self._broadcaster.send_data(center_gradient)


class GossipMax(DecisionPattern):
    """
    Gossip mechanism to find maximum value
    """
    def __init__(self, buffer, frame, key, state=1, moving=True,
                 static=False, diffusion=np.inf, goal_radius=0,
                 ev_factor=1.0, ev_time=0.0):

        super(GossipMax, self).__init__(buffer, frame, key, state, moving,
                                        static, goal_radius, ev_factor,
                                        ev_time, diffusion)

    def value(self):
        """
        determines maximum received value by all agent gradients
        :return: maximum number
        """
        values = self._buffer.agent_list(self.frame, moving=self.moving,
                                         static=self.static)

        for el in values:
            k = [i.key for i in el.payload]
            index = k.index(self.key)

            tmp = float(el.payload[index].value)
            if self.state < tmp:
                self.state = tmp

        return self.state

    def spread(self):
        """
        spreads message with maximum value
        :return:
        """
        super(GossipMax, self).spread()

        # Show info
        rospy.loginfo("Current max: " + str(self.last_value))



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
