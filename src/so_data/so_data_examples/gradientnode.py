#! /usr/bin/env python2
"""
Created on 09.01.2017

@author: kaiser
"""

import rospy
from so_data.sobroadcaster import SoBroadcaster
from so_data.msg import SoMessage
from geometry_msgs.msg import Vector3, Quaternion

def create_gradient(position, attraction=0, diffusion=3.0, angle_x=0.0,
                    angle_y=0.0, q=Quaternion(), moving = False,
                    direction=Vector3(1,0,0), goal_radius=1.0, payload=[],
                    ev_time=0, ev_factor=1.0):
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

    now = rospy.Time.now()

    msg = SoMessage()
    msg.p = position
    msg.q = q
    msg.attraction = attraction
    msg.header.stamp = now
    msg.diffusion = diffusion
    msg.goal_radius = goal_radius
    msg.direction = direction
    msg.angle_x = angle_x
    msg.angle_y = angle_y
    msg.ev_factor = ev_factor
    msg.ev_time = ev_time
    msg.ev_stamp = now
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
        gradients_set.append([create_gradient(Vector3(4, 3, 0), attraction=1,
                                              diffusion=3.0),
                              create_gradient(Vector3(6, 3, 0), attraction=-1,
                                              diffusion=2.0, goal_radius=1.0)
                               ])
        gradients_set.append([create_gradient(Vector3(4, 3, 0), attraction=1,
                                              diffusion=3.0)
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
        gradients_set.append([])
        gradients_set.append([create_gradient(Vector3(9, 9, 0), attraction=-1,
                                              diffusion=2.5, goal_radius=1.5),
                             create_gradient(Vector3(6, 10, 0), attraction=1,
                                             diffusion=2.0, goal_radius=1.7),
                             create_gradient(Vector3(14, 3, 0), attraction=1,
                                              diffusion=2.0, goal_radius=0.75),
                             create_gradient(Vector3(3, 2.25,0), attraction=-1,
                                             diffusion=1.2, goal_radius=0.8)
                             ])

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
        gradient.send_data(gradients)

        rate.sleep()
