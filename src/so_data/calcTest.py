"""
Created on 15.12.2016

@author: kaiser
"""

import unittest
import calc
from geometry_msgs.msg import Point, Vector3


class CalcTest(unittest.TestCase):

    def test_get_gradient_distance(self):
         """
         test calculation of euclidian distance current position - gradient position
         """
         self.assertEqual(calc.get_gradient_distance(Vector3(3,4,0), Point(0,0,0)), 5.0)

    def test_vector_length(self):
        """
        test calculation of vector length
        """
        self.assertEqual(round(calc.vector_length(Vector3(2, 3, 5)), 2), 6.16)

    def test_delta_vector(self):
        """
        test calculation of delta vector calculation
        """
        self.assertEqual(calc.delta_vector(Vector3(0, 2, 3), Vector3(1, 3, 2)), Vector3(-1, -1, 1))

    def test_add_vector(self):
        """
        test calculation of add vectors
        """
        self.assertEqual(calc.add_vectors(Vector3(0,2,3), Vector3(1,3,-2)), Vector3(1,5,1))


# run tests - start roscore before running tests
if __name__ == "__main__":
    unittest.main()