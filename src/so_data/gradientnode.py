#! /usr/bin/env python2
"""
Created on 09.01.2017

@author: kaiser

Module contains (sample) node to spread artificial gradients
(e.g. gradients which should be existent in the environment)
"""

import rospy
from so_data.sobroadcaster import SoBroadcaster
from so_data.msg import SoMessage
from geometry_msgs.msg import Vector3, Quaternion
from so_data.srv import EnvGradient, EnvGradientResponse


def create_gradient(position, attraction=0, diffusion=3.0, q=Quaternion(),
                    moving=False, direction=Vector3(1,0,0), goal_radius=1.0,
                    payload='', ev_time=0, ev_factor=1.0, frameid='',
                    parentframe='', time=None):
    """
    creates a soMessage to specify a gradient
    :param position: agent position
    :param attraction: repulsion(-1), attraction(1)
    :param diffusion: diffusion radius
    :param q: Quaternion defining rotation
    :param direction: initial direction to specify current gradient orientation
    :param goal_radius: goal radius of gradient
    :param payload: payload data
    :param ev_time: evaporation time
    :param ev_factor: evaporation factor
    :param frameid: header frame id of gradient
    :param: parentframe: parent frame id of gradient
    :param time: optional time stamp for evaporation calculations, if None time=rospy.Time.now()
    :return: soMessage
    """

    if time is None:
        time = rospy.Time.now()

    msg = SoMessage()
    msg.p = position
    msg.q = q
    msg.direction = direction
    msg.attraction = attraction
    msg.header.stamp = time
    msg.header.frame_id = frameid
    msg.parent_frame = parentframe
    msg.diffusion = diffusion
    msg.goal_radius = goal_radius
    msg.ev_factor = ev_factor
    msg.ev_time = ev_time
    msg.ev_stamp = time
    msg.moving = moving
    msg.payload = payload

    return msg


class EnvironmentGradients(object):
    """
    class offering service to store gradient set for usage in artificial
    gradient node
    """
    def __init__(self):
        self.gradients = []
        s = rospy.Service('env_gradients', EnvGradient, self.store_gradients)

    def store_gradients(self, req):
        """
        stores list of environment gradients
        :param req: EnvGradient.srv request - list of soMessages
        :return: string
        """
        self.gradients = req.gradients
        resp = EnvGradientResponse()
        resp.name = "Updated Gradient Set."
        return resp


if __name__ == '__main__':
    """
    Node to spread artificial gradients
    """
    rospy.init_node('artificialGradient')
    rate = rospy.Rate(rospy.get_param("~frequency", 1))

    # artificial gradient - initialize spreading
    env = EnvironmentGradients()
    so_broadcaster = SoBroadcaster()

    while not rospy.is_shutdown():
        # send gradients
        so_broadcaster.send_data(env.gradients)

        rate.sleep()
