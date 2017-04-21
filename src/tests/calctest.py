"""
Created on 15.12.2016

@author: kaiser

Module containing unit test for calc.py
"""

import unittest
import numpy as np
import so_data.calc
from geometry_msgs.msg import Point, Vector3


class CalcTest(unittest.TestCase):
    def test_get_gradient_distance(self):
        """
        test calculation of euclidian distance current position - gradient position
        """
        self.assertEqual(
            so_data.calc.get_gradient_distance(Vector3(3, 4, 0),
                                               Point(0, 0, 0)), 5.0)

    def test_vector_length(self):
        """
        test calculation of vector length
        """
        self.assertEqual(round(
            so_data.calc.vector_length(Vector3(2, 3, 5)), 2), 6.16)

    def test_delta_vector(self):
        """
        test calculation of delta vector calculation
        """
        self.assertEqual(
            so_data.calc.delta_vector(Vector3(0, 2, 3), Vector3(1, 3, 2)),
            Vector3(-1, -1, 1))

    def test_add_vector(self):
        """
        test calculation of add vectors method
        """
        self.assertEqual(
            so_data.calc.add_vectors(Vector3(0, 2, 3), Vector3(1, 3, -2)),
            Vector3(1, 5, 1))

    def test_adjust_length(self):
        """
        test calculation adjust_length()
        """
        self.assertEqual(so_data.calc.adjust_length(Vector3(1, 0, 0), 5.0),
                         Vector3(5, 0, 0))

    def test_random(self):
        """
        test calculation of random vector with specified length
        """

        self.assertEqual(so_data.calc.vector_length(
            so_data.calc.random_vector(5.0)), 5.0)

    def test_unit_vector(self):
        """
        test unit vector calculation
        """
        new = so_data.calc.unit_vector([3, 4, 0])
        self.assertEqual(new[0], 0.6)
        self.assertEqual(new[1], 0.8)
        self.assertEqual(new[2], 0.0)

    def test_unit_vector3(self):
        """
        test unit vector 3 calculation
        """
        self.assertEqual(so_data.calc.unit_vector3(Vector3(3, 4, 0)),
                         Vector3(0.6, 0.8, 0))

    def test_angle_between(self):
        """
        test angle_between calculation
        """
        self.assertEqual(so_data.calc.angle_between([1,0], [0,1]), np.pi/2)
        self.assertEqual(so_data.calc.angle_between([0,1], [1,0]), -np.pi/2)
        self.assertEqual(so_data.calc.angle_between([1,0], [1,0]), 0.0)
        self.assertEqual(so_data.calc.angle_between([1,0], [-1,0]), np.pi)


# run tests - start roscore before running tests
if __name__ == "__main__":
    unittest.main()
