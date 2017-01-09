"""
Created on 11.11.2016

@author: kaiser
"""

import rospy
from so_data.soBroadcaster import SoBroadcaster
from so_data.msg import soMessage
from geometry_msgs.msg import Vector3


class SpreadGradient(object):
    """
    class for creating artificial gradients
    """
    def __init__(self):
        self._broadcaster = SoBroadcaster()

    @staticmethod
    def create_gradient(position, attraction=0, diffusion=3.0, angle=0.0, direction=Vector3(),
                        goal_radius=1.0, payload=[], ev_time=0, ev_factor=1.0):
        """
        creates a soMessage to specify a gradient
        :param position: agent position
        :param attraction: repulsion(-1), attraction(1)
        :param diffusion: diffusion radius
        :param angle: angle for directed gradient
        :param direction: direction for directed gradient
        :param goal_radius: goal radius of gradient
        :param payload: payload data
        :param ev_time: evaporation time
        :param ev_factor: evaporation factor
        :return: soMessage
        """
        msg = soMessage()
        msg.p = position
        msg.attraction = attraction
        msg.header.stamp = rospy.Time.now()
        msg.diffusion = diffusion
        msg.goal_radius = goal_radius
        msg.angle = angle
        msg.ev_factor = ev_factor
        msg.ev_time = ev_time
        msg.direction = direction
        msg.payload = payload
        return msg

    def create_send_message(self, position, attraction=0, diffusion=3.0, angle=0.0, ev_time=0, ev_factor=1.0,
                            direction=Vector3(), goal_radius=1.0, payload=[]):

        """
        create and broadcast/spread gradient data
        :param position:
        :param attraction:
        :param diffusion:
        :param angle:
        :param ev_time:
        :param ev_factor:
        :param direction:
        :param goal_radius:
        :param payload:
        :return:
        """
        msg = soMessage()
        msg.p = position
        msg.attraction = attraction  # 1 = attractive, -1 = repulsive
        msg.header.stamp = rospy.Time.now()
        msg.diffusion = diffusion
        msg.goal_radius = goal_radius
        msg.angle = angle
        msg.ev_factor = ev_factor
        msg.ev_time = ev_time
        msg.direction = direction
        msg.payload = payload
        self._broadcaster.send_data(msg)

    def send_message(self, msg):
        """
        broadcast/spread gradient data
        :param msg: soMessage
        :return:
        """
        self._broadcaster.send_data(msg)
