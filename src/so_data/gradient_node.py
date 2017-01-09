#! /usr/bin/env python2
"""
Created on 09.01.2017

@author: kaiser
"""

import rospy
from selforga.gradient import SpreadGradient
from geometry_msgs.msg import Vector3

index = rospy.get_param("~gradientPosesSetIndex", -1)

def get_gradient(index):
    # possible gradients, can be changed / enhanced as required
    if index >= 0:
        gradients_set = []
        gradients_set.append([SpreadGradient.create_gradient(Vector3(2, 3, 0), attraction=-1, diffusion=1.0,
                                                             goal_radius=1.0),
                              SpreadGradient.create_gradient(Vector3(5, 3, 0), attraction=1, diffusion=3.0),
                              SpreadGradient.create_gradient(Vector3(8, 3, 0), attraction=-1, diffusion=1.0,
                                                             goal_radius=1.0)])
        gradients_set.append([SpreadGradient.create_gradient(Vector3(2, 1, 0), attraction=1, diffusion=3.0),
                              SpreadGradient.create_gradient(Vector3(4, 1, 0), attraction=-1, diffusion=1.0,
                                                             goal_radius=2.0)])
        gradients_set.append([SpreadGradient.create_gradient(Vector3(2, 1, 0), attraction=1, diffusion=3.0)])
        gradients_set.append([SpreadGradient.create_gradient(Vector3(6, 6, 0), attraction=1, diffusion=3.0),
                              SpreadGradient.create_gradient(Vector3(4, 4, 0), attraction=-1, diffusion=5.0)])
        gradients_set.append([SpreadGradient.create_gradient(Vector3(4, 4, 0), attraction=-1, diffusion=2.0),
                              SpreadGradient.create_gradient(Vector3(6, 6, 0), attraction=1, diffusion=5.0)])

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
    gradient = SpreadGradient()
    gradients = get_gradient(gradient_poses_set_index)

    while not rospy.is_shutdown():

        # send gradients
        for val in gradients:
            gradient.send_message(val)

        rate.sleep()


