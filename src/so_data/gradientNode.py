#! /usr/bin/env python2
"""
Created on 09.01.2017

@author: kaiser
"""

import rospy
from so_data.soBroadcaster import SoBroadcaster
from so_data.msg import soMessage
from geometry_msgs.msg import Vector3


def create_gradient(position, attraction=0, diffusion=3.0, angle_x=0.0,
                    angle_y=0.0, direction=Vector3(), moving = False,
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
    msg.angle_x = angle_x
    msg.angle_y = angle_y
    msg.ev_factor = ev_factor
    msg.ev_time = ev_time
    msg.direction = direction
    msg.moving = moving
    msg.payload = payload
    return msg


def get_gradient(index):
    """
    :param index: index of gradient list to be returned
    :return: gradient list
    """
    # possible gradients, can be changed / enhanced as required
    if index >= 0:
        gradients_set = []
        gradients_set.append([create_gradient(Vector3(2, 3, 0), attraction=-1,
                                              diffusion=1.0, goal_radius=1.0),
                              create_gradient(Vector3(5, 3, 0), attraction=1,
                                              diffusion=3.0),
                              create_gradient(Vector3(8, 3, 0), attraction=-1,
                                              diffusion=1.0, goal_radius=1.0)])
        gradients_set.append([create_gradient(Vector3(2, 1, 0), attraction=1,
                                              diffusion=3.0),
                              create_gradient(Vector3(4, 1, 0), attraction=-1,
                                              diffusion=1.0, goal_radius=2.0)
                               ])
        gradients_set.append([create_gradient(Vector3(2, 1, 0), attraction=1,
                                              diffusion=3.0)])
        gradients_set.append([create_gradient(Vector3(6, 6, 0), attraction=1,
                                              diffusion=3.0),
                              create_gradient(Vector3(4, 4, 0), attraction=-1,
                                              diffusion=5.0)])
        gradients_set.append([create_gradient(Vector3(4, 4, 0), attraction=-1,
                                              diffusion=2.0),
                              create_gradient(Vector3(6, 6, 0), attraction=1,
                                              diffusion=5.0)])

        if index < len(gradients_set):
            return gradients_set[index]
        else:
            rospy.logwarn("gradientSet index out of range")
            return []


if __name__ == '__main__':
    """
    Node to spread artificial gradients
    """
    rospy.init_node('artificialGradient')
    rate = rospy.Rate(rospy.get_param("~frequency", 1))
    gradient_poses_set_index = rospy.get_param("~gradientPosesSetIndex", -1)

    # artificial gradient - initialize spreading
    gradient = SoBroadcaster()
    gradients = get_gradient(gradient_poses_set_index)

    while not rospy.is_shutdown():
        # send gradients
        for val in gradients:
            gradient.send_data(val)

        rate.sleep()
