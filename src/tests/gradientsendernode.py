#!/usr/bin/env python2

import rospy
from so_data.sobroadcaster import SoBroadcaster
from so_data.gradientnode import create_gradient
from geometry_msgs.msg import Vector3
import random


if __name__ == '__main__':
    """
    spread gradients to so data topic
    """

    rospy.init_node('gradientTest')
    rate = rospy.Rate(rospy.get_param("~frequency", 5))

    # artificial gradient - initialize spreading
    gradient = SoBroadcaster()

    while not rospy.is_shutdown():
        # send gradients
        gradient.send_data(create_gradient(Vector3(random.uniform(0.0, 20.0), random.uniform(0,20), random.uniform(0.0,20.0)), ev_time=3, ev_factor=0.9))

        rate.sleep()
