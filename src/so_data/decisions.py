"""
Created on 16.02.2017

@author: kaiser

Module including sample implementations of decision patterns: morphogenesis,
gossip, quorum
"""

from patterns import DecisionPattern
import so_data.calc
import rospy
import numpy as np
from so_data.gradientnode import create_gradient


class MorphogenesisBarycenter(DecisionPattern):
    """
    Morphogenesis mechanism to determine barycenter of robot group
    """
    def __init__(self, buffer, frame, key, center_frame='Center', moving=True,
                 static=False, goal_radius=0.5, ev_factor=1.0, ev_time=0.0,
                 diffusion=np.inf, attraction=-1, value=0, state='None',
                 goal_center=2.0, moving_center=False, attraction_center=1):
        """
        initialize behaviour
        :param buffer: SoBuffer
        :param frame: morphogenesis frame id (header frame)
        :param key: payload key
        :param center_frame: frame ID of spread barycenter frame
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradient in list returned by buffer
        :param goal_radius: morphogenetic gradient goal radius
        :param ev_factor: morphogenetic gradient evaporation factor
        :param ev_time: morphogenetic gradient evaporation time
        :param diffusion: morphogenetic gradient diffusion
        :param attraction: morphogenetic gradient attraction
        :param state: robot state (Center, None)
        :param value: sum of distances to morphogentic gradients
        :param goal_center: goal radius barycenter gradient
        :param moving_center: moving attribute barycenter gradient
        :param attraction_center: attraction barycenter gradient
        :param diffusion_center: diffusion barycenter gradient
        """

        super(MorphogenesisBarycenter, self).__init__(buffer, frame, key,
                                                      value, state, moving,
                                                      static, goal_radius,
                                                      ev_factor, ev_time,
                                                      diffusion, attraction)

        # Center gradient
        self.goal_center = goal_center
        self.moving_center = moving_center
        self.attraction_center = attraction_center
        self.center_frame = center_frame

    def calc_value(self):
        """
        sums up distance to all morphogenetic gradients, determines and
        sets state of robot based on it
        :return: [distance (float), state]
        """
        values = self._buffer.agent_list([self.frame])
        own_pos = self._buffer.get_own_pose()

        if not own_pos:
            return None

        if not values:
            return [self.value, self.state]

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
            ndist = el.payload[index].value
            # neighbor dist larger than own dist
            if ndist > dist:
                count += 1

        # set state
        state = 'None'
        if neighbors != 0 and count == neighbors:
            state = 'Center'

        return [dist, state]

    def spread(self):
        """
        spreads morphogenetic gradient with sum of distances
        + spreads center gradient if robot is barycenter
        """
        super(MorphogenesisBarycenter, self).spread()

        # if barycenter: spread gradient for chemotaxis
        if self.state == 'Center':
            rospy.loginfo("Agent state: Center")
            # send Center gradient: diffusion = sum of distance of agent
            center_gradient = create_gradient(self.get_pos().p,
                                              goal_radius=self.goal_center,
                                              attraction=self.attraction_center,
                                              diffusion=self.value,
                                              moving=self.moving_center,
                                              frameid=self.center_frame)

            self._broadcaster.send_data(center_gradient)


class GossipMax(DecisionPattern):
    """
    Gossip mechanism to find maximum spread value
    """
    def __init__(self, buffer, frame, key, value=1, state=None, moving=True,
                 static=False, diffusion=np.inf, goal_radius=0,
                 ev_factor=1.0, ev_time=0.0):
        """
        initialize behaviour
        :param buffer: SoBuffer
        :param frame: gossip frame id (header frame)
        :param key: payload key
        :param value: initial value
        :param state: robot state - not changed during mechanism execution
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradient in list returned by buffer
        :param goal_radius: gossip gradient goal radius
        :param ev_factor: gossip gradient evaporation factor
        :param ev_time: gossip gradient evaporation time
        :param diffusion: gossip gradient diffusion
        """

        super(GossipMax, self).__init__(buffer, frame, key, value, state,
                                        moving, static, goal_radius, ev_factor,
                                        ev_time, diffusion)

    def calc_value(self):
        """
        determines maximum received value by all agent gradients
        :return: maximum number
        """
        values = self._buffer.agent_list([self.frame])

        tmp_max = self.value

        for el in values:
            k = [i.key for i in el.payload]
            index = k.index(self.key)

            tmp = el.payload[index].value
            if tmp_max < tmp:
                tmp_max = tmp

        return [tmp_max, self.state]

    def spread(self):
        """
        spreads message with maximum value
        :return:
        """
        super(GossipMax, self).spread()

        # Show info
        rospy.loginfo("Current max: " + str(self.value))


# QUORUM
class Quorum(DecisionPattern):
    """
    Quorum Sensing mechanism
    """
    def __init__(self, buffer, threshold, frame=None, value=0, state=False,
                 moving=True, static=False):
        """
        initialize behaviour
        :param buffer: SoBuffer
        :param threshold: number of agents which has to be reached
        :param frame: frame id (header frame) of agent data
        :param value: initial value of robot (count of neighbors)
        :param state: state to indicate whether threshold was passed
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradient in list returned by buffer
        """
        super(Quorum, self).__init__(buffer, frame, value=value, state=state,
                                     moving=moving, static=static)

        # set standard agent frame if no frame is specified
        if not frame:
            self.frame = self._buffer.pose_frame

        self.threshold = threshold

    def calc_value(self):
        """
        determines number of agents within view
        :return: state
        """

        values = self._buffer.agent_list([self.frame])

        count = len(values)

        state = False
        # set state
        if count >= self.threshold:
            state = True

        return [count, state]
