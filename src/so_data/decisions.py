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
        """
        initialize behaviour
        :param buffer: SoBuffer
        :param frame: morphogenesis frame id (header frame)
        :param key: payload key
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradient in list returned by buffer
        :param goal_radius: morphogenetic gradient goal radius
        :param ev_factor: morphogenetic gradient evaporation factor
        :param ev_time: morphogenetic gradient evaporation time
        :param diffusion: morphogenetic gradient diffusion
        :param attraction: morphogenetic gradient attraction
        :param state: robot state
        :param goal_center: goal radius barycenter gradient
        :param moving_center: moving attribute barycenter gradient
        :param attraction_center: attraction barycenter gradient
        :param diffusion_center: diffusion barycenter gradient
        """

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
        """
        initialize behaviour
        :param buffer: SoBuffer
        :param frame: gossip frame id (header frame)
        :param key: payload key
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradient in list returned by buffer
        :param goal_radius: gossip gradient goal radius
        :param ev_factor: gossip gradient evaporation factor
        :param ev_time: gossip gradient evaporation time
        :param diffusion: gossip gradient diffusion
        :param state: robot state
        """

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


# QUORUM
class Quorum(DecisionPattern):
    """
    Quorum Sensing
    """

    def __init__(self, buffer, threshold, frame=None, state=False,
                 moving=True,
                 static=False, diffusion=np.inf, goal_radius=0,
                 ev_factor=1.0, ev_time=0.0):
        """
        initialize behaviour
        :param buffer: SoBuffer
        :param threshold: number of agents which has to be reached
        :param frame: frame id (header frame) agent data
        :param moving: consider moving gradients in list returned by buffer
        :param static: consider static gradient in list returned by buffer
        :param goal_radius: gossip gradient goal radius
        :param ev_factor: gossip gradient evaporation factor
        :param ev_time: gossip gradient evaporation time
        :param diffusion: gossip gradient diffusion
        :param state: robot state
        """

        super(Quorum, self).__init__(buffer, frame, None, state, moving,
                                     static, goal_radius, ev_factor,
                                     ev_time,
                                     diffusion)

        # set standard agent frame if no frame is specified
        if not frame:
           self.frames = buffer.pose_frame

        self.threshold = threshold

    def value(self):
        """
        determines number of agents within view
        :return: maximum number
        """
        values = self._buffer.agent_list(self.frames, moving=self.moving,
                                         static=self.static)

        # set state
        if len(values) >= self.threshold:
            self.state = True
        else:
           self.state = False

        return self.state
