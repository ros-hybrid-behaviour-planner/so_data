"""
Created on 21.12.2016

@author: kaiser
"""

import unittest
import calc
from so_data.msg import soMessage
from geometry_msgs.msg import Point, Vector3
import numpy as np
import flocking


class flockingTest(unittest.TestCase):
    """
    unit test flocking.py
    """

    def test_bump_function(self):
        """
        unit test of bump function
        """

        h = 0.6

        z = 0.5
        self.assertEqual(flocking.bump_function(z, h), 1.0)

        z = 0.6
        self.assertEqual(flocking.bump_function(z, h), 1.0)

        z = 0.8
        self.assertEqual(flocking.bump_function(z, h), 0.5*(1+np.cos(np.pi * (z-h)/(1-h)))) # 0.5

        z = 1.0
        self.assertEqual(flocking.bump_function(z, h), 0.0)

        z = 2.0
        self.assertEqual(flocking.bump_function(z, h), 0.0)

        z = -1.0
        self.assertEqual(flocking.bump_function(z, h), 0.0)

    def test_sigma_norm(self):
        """
        unit test for sigma norm function for vectors
        """

        epsilon = 0.75
        z = Vector3()
        self.assertEqual(flocking.sigma_norm(epsilon, z), 0.0)

        epsilon = 1.0
        z = Vector3(0.6, 0.8, 0.0)
        self.assertEqual(round(flocking.sigma_norm(epsilon, z), 4), 0.4142)


    def test_sigma_norm_f(self):
        """
        unit test for sigma norm function for float values
        """

        epsilon = 0.75
        z = 2.0
        self.assertEqual(flocking.sigma_norm_f(epsilon, z), 4.0/3.0)

        epsilon = 1.0
        z = 3.0
        self.assertEqual(round(flocking.sigma_norm_f(epsilon, z), 4), 2.1623)



# run tests - start roscore before running tests
if __name__ == "__main__":
    unittest.main()