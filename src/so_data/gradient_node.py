#! /usr/bin/env python2
"""
Created on 09.01.2017

@author: kaiser
"""

import rospy
import selforga.gradient
from geometry_msgs.msg import Vector3


if __name__ == '__main__':
    """
    Node to spread artificial gradients
    """
    rospy.init_node('artificialGradient')
    rate = rospy.Rate(rospy.get_param("~frequency", 1))
    gradient_poses_set_index = rospy.get_param("~gradientPosesSetIndex", -1)

    # artificial gradient - initialize spreading
    gradient = selforga.gradient.SpreadGradient()

    gradients = []

    # possible gradients, can be changed / enhanced as required
    if gradient_poses_set_index >= 0:
        gradients_set = []
        gradients_set.append([gradient.create_gradient(Vector3(2, 3, 0), attraction=-1, diffusion=1.0, goal_radius=1.0),
                              gradient.create_gradient(Vector3(5, 3, 0), attraction=1, diffusion=3.0),
                              gradient.create_gradient(Vector3(8, 3, 0), attraction=-1, diffusion=1.0,
                                                       goal_radius=1.0)])
        gradients_set.append([gradient.create_gradient(Vector3(2, 1, 0), attraction=1, diffusion=3.0),
                              gradient.create_gradient(Vector3(4, 1, 0), attraction=-1, diffusion=1.0,
                                                       goal_radius=2.0)])
        gradients_set.append([gradient.create_gradient(Vector3(2, 1, 0), attraction=1, diffusion=3.0)])
        gradients_set.append([gradient.create_gradient(Vector3(6, 6, 0), attraction=1, diffusion=3.0),
                              gradient.create_gradient(Vector3(4, 4, 0), attraction=-1, diffusion=5.0)])
        gradients_set.append([gradient.create_gradient(Vector3(4, 4, 0), attraction=-1, diffusion=2.0),
                              gradient.create_gradient(Vector3(6, 6, 0), attraction=1, diffusion=5.0)])

        if gradient_poses_set_index < len(gradients_set):
            gradients = gradients_set[gradient_poses_set_index]
        else:
            rospy.logwarn("gradientSet index out of range")

    while not rospy.is_shutdown():

        # send gradients
        for val in gradients:
            gradient.send_message(val)

        rate.sleep()





