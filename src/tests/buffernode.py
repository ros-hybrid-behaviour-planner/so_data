#!/usr/bin/env python2

import time
import cProfile
from geometry_msgs.msg import Point, Vector3, Quaternion
from so_data.sobuffer import SoBuffer, AGGREGATION
from so_data.msg import SoMessage
import rospy


def check(buffer, file):
    i = 0
    if buffer._static.keys():
        i = len(buffer._static.get('None'))
    t_start = time.time()
    cProfile.run('buffer.gradients()')
    #buffer.gradients()

    dt = time.time() - t_start

    file.write(str(i) + " , " + str(dt) + "\n")  # str() converts to string


if __name__ == '__main__':
    # buffer instance
    rospy.init_node('bufferNodeTest')
    rate = rospy.Rate(rospy.get_param("~frequency", 1))

    # Open new data file

    buffer = SoBuffer(view_distance=5.0, aggregation_distance=0.0, ev_thread=True, ev_time=5)

    buffer._static={'None': [

                    ]
                    }

    buffer._own_pos = [
        SoMessage(None, None, Vector3(1, 1, 1), Quaternion(), Vector3(), 1,
                  1.0, 0, 1.0, 0, None, False, [])]

    f = open("data2.txt", "w")


    while not rospy.is_shutdown():

        check(buffer, f)

        rate.sleep()

    f.close()
