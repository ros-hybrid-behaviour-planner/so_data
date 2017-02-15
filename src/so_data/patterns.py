"""
Created on 15.02.2017

@author: kaiser

Module including abstract class for patterns
"""
from geometry_msgs.msg import Vector3
from abc import ABCMeta, abstractmethod


class MovementPattern(object):
    """
    abstract class for movement patterns / mechanisms
    """

    __metaclass__ = ABCMeta

    def __init__(self, buffer, maxvel=1.0, frame=None, moving=True, static=True):
        """
        :param buffer: SoBuffer
        :param maxvel: maximum velocity / length of movement vector
        :param moving: include moving gradients in calculations
        :param static: include static gradients in calculations
        """

        self._buffer = buffer
        self.max_velocity = maxvel

        # frames to request list of SoMessages
        # TODO: make list instead of unique ID
        if not frame:
            self.frame = self._buffer.pose_frame
        else:
            self.frame = frame

        # store current position used for last calculation
        self._current_pos = None

    @abstractmethod
    def move(self):
        """
        method calculates movement vector based on Gradient List received
        by buffer
        :return: movement vector (Vector3)
        """

        self._current_pos = self._buffer.get_own_pose()
        view = self._buffer.decision_list(self.frame)
        mov = Vector3()

        # TODO implement movement vector calculation

        return mov

    def get_pos(self):
        """
        :return: current position of robot
        """
        return self._current_pos
