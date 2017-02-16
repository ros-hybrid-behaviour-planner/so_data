"""
Created on 15.02.2017

@author: kaiser

Module including abstract class for patterns
"""
from abc import ABCMeta, abstractmethod


class MovementPattern(object):
    """
    abstract class for movement patterns / mechanisms
    """

    __metaclass__ = ABCMeta

    def __init__(self,  buffer, frames=None, repulsion=False, moving=True,
                 static=True, maxvel=1.0, minvel=0.1):
        """
        :param buffer: SoBuffer
        :param maxvel: maximum velocity / length of movement vector
        :param moving: include moving gradients in calculations
        :param static: include static gradients in calculations
        """

        self._buffer = buffer
        self.max_velocity = maxvel

        self._buffer = buffer

        self.frames = frames
        # store current position used for last calculation
        self._current_pos = None

        # apply repulsion
        self.repulsion = repulsion
        # consider moving, static or both gradient types
        self.moving = moving
        self.static = static

        self.maxvel = maxvel
        self.minvel = minvel

    @abstractmethod
    def move(self):
        """
        method calculates movement vector based on Gradient List received
        by buffer
        :return: movement vector (Vector3)
        """
        pass

    def get_pos(self):
        """
        :return: current position of robot
        """
        return self._current_pos
