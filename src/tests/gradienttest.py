"""
Created on 20.02.2017

@author: kaiser

Unit test for gradient.py
"""

import unittest
from so_data.msg import SoMessage
import rospy
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
import so_data.gradient as gradient


class GradientTest(unittest.TestCase):
    """
    testing methods of module gradient
    """

    def test_calc_attractive_gradient(self):
        """
        test _calc_attractive_gradient method for 2D and 3D
        """
        # 2D - D < r <= C
        grad = SoMessage(None, None, Vector3(3, 4, 0), Quaternion(), 1, 5.0,
                         1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        pose = SoMessage(None, None, Vector3(0, 0, 0), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_attractive_gradient(grad, pose),
                         Vector3(0.6 * 0.8, 0.8 * 0.8, 0))

        # 2D - r > C
        grad = SoMessage(None, None, Vector3(3, 4, 0), Quaternion(), 1, 2.0,
                         1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_attractive_gradient(grad, pose),
                         Vector3(0.6, 0.8, 0))

        # 2D - r <= D
        grad = SoMessage(None, None, Vector3(3, 4, 0), Quaternion(), 1, 2.0,
                         5.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_attractive_gradient(grad, pose),
                         Vector3(0, 0, 0))

        # 2D - r > C - non zero robot pose
        grad = SoMessage(None, None, Vector3(4, 5, 0), Quaternion(), 1,
                         2.0, 1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        pose = SoMessage(None, None, Vector3(1, 1, 0), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_attractive_gradient(grad, pose),
                         Vector3(0.6, 0.8, 0))

        # 3D - D < r <= C
        grad = SoMessage(None, None, Vector3(3, 5, 10), Quaternion(), 1, 6.0,
                         2.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        pose = SoMessage(None, None, Vector3(1, 2, 4), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        result = Vector3((2.0 / 7.0) * (5.0 / 6.0), (3.0 / 7.0) * (5.0 / 6.0),
                         (6.0 / 7.0) * (5.0 / 6.0))
        self.assertEqual(gradient.calc_attractive_gradient(grad, pose), result)

    # 3D - r > C
        grad = SoMessage(None, None, Vector3(3, 5, 10), Quaternion(), 1, 5.0,
                         2.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        result = Vector3((2.0 / 7.0), (3.0 / 7.0), (6.0 / 7.0))
        self.assertEqual(gradient.calc_attractive_gradient(grad, pose), result)

    # 3D - r <= D
        grad = SoMessage(None, None, Vector3(3, 5, 10), Quaternion(), 1, 5.0,
                         7.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_attractive_gradient(grad, pose),
                         Vector3(0, 0, 0))

    def test_calc_repulsive_gradient(self):
        """
        test _calc_repulsive_gradient method
        """
        # 2D - D < r <= C
        grad = SoMessage(None, None, Vector3(3, 4, 0), Quaternion(), -1, 5.0,
                         1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])

        pose = SoMessage(None, None, Vector3(0, 0, 0), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_repulsive_gradient(grad, pose),
                         Vector3(-0.6 * 0.2, -0.8 * 0.2, 0))

        # 2D - r > C
        grad = SoMessage(None, None, Vector3(3, 4, 0), Quaternion(), -1, 2.0,
                         1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_repulsive_gradient(grad, pose),
                         Vector3(0, 0, 0))

        # 2D - r <= D
        grad = SoMessage(None, None, Vector3(3, 4, 0), Quaternion(), -1, 2.0,
                         5.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_repulsive_gradient(grad, pose),
                         Vector3(-1.0 * np.inf, -1.0 * np.inf, np.inf))

        # 2D - r > C - non zero robot pose
        grad = SoMessage(None, None, Vector3(4, 5, 0), Quaternion(), -1, 2.0,
                         1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        pose = SoMessage(None, None, Vector3(1, 1, 0), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_repulsive_gradient(grad, pose),
                         Vector3(0, 0, 0))

        # 3D - D < r <= C
        grad = SoMessage(None, None, Vector3(3, 5, 10), Quaternion(), -1, 6.0,
                         2.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        pose = SoMessage(None, None, Vector3(1, 2, 4), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        result = Vector3((-2.0 / 7.0) * (1.0 / 6.0), (-3.0 / 7.0) *
                         (1.0 / 6.0), (-6.0 / 7.0) * (1.0 / 6.0))
        self.assertEqual(gradient.calc_repulsive_gradient(grad, pose), result)

        # 3D - r > C
        grad = SoMessage(None, None, Vector3(3, 5, 10), Quaternion(), -1, 5.0,
                         2.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_repulsive_gradient(grad, pose),
                         Vector3(0, 0, 0))

        # 3D - r <= D
        grad = SoMessage(None, None, Vector3(3, 5, 10), Quaternion(), -1, 5.0,
                         7.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_repulsive_gradient(grad, pose),
                         Vector3(-1 * np.inf, -1 * np.inf, -1 * np.inf))

    def test_calc_attractive_gradient_ge(self):
        """
        test calc attractive gradient method based on Ge & Cui paper
        :return:
        """
        # robot within diffusion radius + goal radius of gradient
        grad = SoMessage(None, None, Vector3(3, 5, 10), Quaternion(), -1,
                         10.0, 2.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        pose = SoMessage(None, None, Vector3(0, 0, 0), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        result = gradient.calc_attractive_gradient_ge(grad, pose)
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(2.48, 4.14, 8.27))

        # robot within goal radius of gradient
        pose = SoMessage(None, None, Vector3(3, 7, 10), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_attractive_gradient_ge(grad, pose),
                         Vector3(0, 0, 0))

        # robot without radius + goal radius of gradient, but gradient is
        # within view_distance
        grad = SoMessage(None, None, Vector3(2, 3, 6), Quaternion(), -1,
                         4.0, 2.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        pose = SoMessage(None, None, Vector3(0, 0, 0), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_attractive_gradient_ge(grad, pose),
                         Vector3((2.0 / 7.0) * 4.0, (3.0 / 7.0) * 4.0,
                                 (6.0 / 7.0) * 4.0))

    def test_repulsive_gradient_ge(self):
        """
        test repulsion vector calculation based on Ge
        :return:
        """
        grad = SoMessage(None, None, Vector3(4, 2, 0), Quaternion(), -1, 4.0,
                         2.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        goal = SoMessage(None, None, Vector3(1, 0, 0), Quaternion(), 1, 3.0,
                         1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])

        # diffusion and goal_radius of gradient shorter than distance
        pose = SoMessage(None, None, Vector3(8, 8, 0), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_repulsive_gradient_ge(grad, goal, pose),
                         Vector3())

        # agent within goal area of repulsive gradient
        pose = SoMessage(None, None, Vector3(3, 2, 0), Quaternion(), -1, 3.0,
                         0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        self.assertEqual(gradient.calc_repulsive_gradient_ge(grad, goal, pose),
                         Vector3(-np.inf, np.inf, np.inf))

        # robot within reach of gradient
        pose = SoMessage(None, None, Vector3(1, -2, 0), Quaternion(), -1, 3.0,
                          0.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        v = gradient.calc_repulsive_gradient_ge(grad, goal, pose)
        v.x = round(v.x, 2)
        v.y = round(v.y, 2)
        v.z = round(v.z, 2)
        self.assertEqual(v, Vector3(-0.02, -0.02, 0.0))


# run tests - start roscore before running tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()

