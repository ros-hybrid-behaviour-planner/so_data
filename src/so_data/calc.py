'''
Created on 01.12.2016

@author: kaiser
'''

import numpy as np
from geometry_msgs.msg import Vector3





def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the directed angle in radians between vectors 'v1' and 'v2' - only working in 2D!::

            >>> angle_between((1, 0), (0, 1))
            1.5707963267948966
            >>> angle_between((0, 1), (1, 0))
            -1.5707963267948966
            >>> angle_between((1, 0), (1, 0))
            0.0
            >>> angle_between((1, 0), (-1, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    # det needs square matrix as input!
    angle = np.math.atan2(np.linalg.det([v1_u, v2_u]), np.dot(v1_u, v2_u))

    if np.isnan(angle):
        return 0.0

    return angle

def get_gradient_distance(gradpos, pose):
    '''
    :param gradpos: pose of the gradient to be investigated (Vector)
    :param pose: pose of the robot (Pose)
    :return: euclidian distance robot to last received gradient
    '''
    return np.linalg.norm([(gradpos.x - pose.x), (gradpos.y - pose.y)])