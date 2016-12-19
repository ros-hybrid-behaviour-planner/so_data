'''
Created on 15.12.2016

@author: kaiser
'''

import soBuffer
import unittest
import calc
from so_data.msg import soMessage
import rospy
from geometry_msgs.msg import Point, Vector3
from copy import deepcopy
from std_msgs.msg import Header
import numpy as np

class calcTest(unittest.TestCase):

    def test_get_gradient_distance(self):
         """
         test calculation of euclidian distance current position - gradient position
         """
         self.assertEqual(calc.get_gradient_distance(Vector3(3,4,0), Point(0,0,0)), 5.0)



# run tests - start roscore before running tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()