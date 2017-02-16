"""
Created on 16.02.2017

@author: kaiser

Module including sample implementations of decision patterns
"""

from patterns import DecisionPattern
import so_data.calc
import rospy


class Morphogenesis(DecisionPattern):
    """
    Find barycenter of robot group
    """
    def __init__(self, buffer, frames, moving=True, static=False):

        super(Morphogenesis, self).__init__(buffer, frames, moving, static)

        self._value = None
        self._list = None

        self.last_value = -1
        self.last_decision = False

    def value(self):

        self._list = self._buffer.decision_list(self.frames)
        own_pos = self._buffer.get_own_pose()

        if not self._list:
            return -1

        dist = 0
        # determine overall distance
        for el in self._list:
            d = so_data.calc.get_gradient_distance(own_pos.p, el.p)
            dist += d

        return dist

    def decision(self):

        # TODO: brauch ich das wirklich?
        self._value = self.value()

        neighbors = 0
        count = 0

        for el in self._list:
            neighbors += 1

            #keys = [i.key for i in el.payload]
            #index = keys.index(self.key)
            dist = float(el.payload[0].value)

            if dist > self._value:
                count += 1

        if neighbors != 0 and count == neighbors:
            return True

        return False


class Gossip(DecisionPattern):
    """
    find maximum value
    """
    def __init__(self, buffer, frames, initial_value=1,
                 moving=True, static=False):

        super(Gossip, self).__init__(buffer, frames, moving, static)

        self._value = initial_value
        self.last_value = -1
        self._list = None

    def value(self):

        self._list = self._buffer.decision_list(self.frames)

        for el in self._list:
            #keys = [i.key for i in el.payload]
            #index = keys.index(self.key)

            tmp = float(el.payload[0].value)

            if self._value < tmp:
                self._value = tmp

        return self._value
