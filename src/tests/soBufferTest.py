"""
Created on 14.11.2016

@author: kaiser
"""

from so_data.soBuffer import SoBuffer
import unittest
import so_data.calc
from so_data.msg import soMessage
import rospy
from geometry_msgs.msg import Vector3
from copy import deepcopy
from std_msgs.msg import Header
import numpy as np


class SoBufferTest(unittest.TestCase):
    """
    testing of soBuffer methods
    """
    maxDiff = None

    def test_quorum(self):
        """
        test quorum / density function
        :return:
        """
        bffr = SoBuffer()

        # 2 neighbors within view, one outside view
        bffr._moving = {
            'robot1': [soMessage(), soMessage(None, Vector3(2, 2, 0), 1, 1.0,
                                              1.0, 1.0, 0, None, 0, 0,
                                              Vector3(),
                                              True, [])],
            'robot2': [soMessage(), soMessage(None, Vector3(5, 6, 0), 1, 2.0,
                                              1.0, 1.0, 0, None, 0, 0,
                                              Vector3(),
                                              True, [])],
            'robot3': [soMessage(), soMessage(), soMessage(None,
                                                           Vector3(1, 2, 0),
                                                           1, 4.0, 1.0, 1.0, 0,
                                                           None,
                                                           0, 0, Vector3(),
                                                           True, [])],
            'robot4': []}

        bffr._static = {
            'None': [soMessage(None, Vector3(5, 6, 5), 1, 1.0,
                               1.0, 1.0, 0, None, 0, 0, Vector3(),
                               False, [])]
        }

        bffr._own_pos = [soMessage(None, Vector3(1, 1, 1), 1, 1.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]

        # no. of robots in view distance < threshold
        bffr.threshold = 5
        self.assertEqual(bffr.quorum(), False)
        bffr.threshold = 3
        self.assertEqual(bffr.quorum(), False)

        # no. of robots in view distance > threshold
        bffr.threshold = 2
        self.assertEqual(bffr.quorum(), True)
        bffr.threshold = 1
        self.assertEqual(bffr.quorum(), True)

    def test_quorum_list(self):
        """
        test quorum / density function
        :return:
        """
        bffr = SoBuffer(view_distance=2.0, result_static=False,
                        result_moving=True)

        self.assertEqual(bffr.quorum_list(), [])

        # 2 neighbors within view, one outside view
        bffr._moving = {
            'robot1': [soMessage(), soMessage(None, Vector3(2, 2, 0), 1, 1.0,
                                              1.0, 1.0, 0, None, 0, 0,
                                              Vector3(),
                                              True, [])],
            'robot2': [soMessage(), soMessage(None, Vector3(5, 6, 0), 1, 2.0,
                                              1.0, 1.0, 0, None, 0, 0,
                                              Vector3(),
                                              True, [])],
            'robot3': [soMessage(), soMessage(), soMessage(None,
                                                           Vector3(1, 2, 0),
                                                           1, 4.0, 1.0, 1.0, 0,
                                                           None,
                                                           0, 0, Vector3(),
                                                           True, [])]}

        bffr._static = {
            'None': [soMessage(None, Vector3(5, 6, 5), 1, 1.0,
                               1.0, 1.0, 0, None, 0, 0, Vector3(),
                               False, []), soMessage()],
        }

        bffr._own_pos = [soMessage(None, Vector3(1, 1, 1), 1, 1.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]

        result = [
            soMessage(None, Vector3(1, 2, 0), 1, 4.0, 1.0, 1.0, 0, None, 0, 0,
                      Vector3(), True, []),
            soMessage(None, Vector3(2, 2, 0), 1, 1.0, 1.0, 1.0, 0, None, 0, 0,
                      Vector3(), True, [])]

        self.assertEqual(bffr.quorum_list(), result)
        bffr.result_static = True
        result.append(soMessage())
        self.assertEqual(bffr.quorum_list(), result)

    # GOAL REACHED
    def test_get_goal_reached(self):
        """
        test get_goal_reached method
        :return:
        """
        bffr = SoBuffer()

        # no gradients available --> True
        bffr._own_pos = [soMessage(None, Vector3(2, 2, 2), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_goal_reached(), False)

        bffr._static = {
            'gradient': [
                soMessage(None, Vector3(2, 3, 1), 1, 3.0, 1.0, 1.0, 0, None,
                          0, 0, Vector3(), False, []),
                soMessage(None, Vector3(2, 2, 2), 1, 1.0, 1.0, 1.0, 0, None,
                          0, 0, Vector3(), False, [])],
            'None': [
                soMessage(None, Vector3(7, 3, 2), -1, 3.0, 1.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(5, 6, 3), 1, 2.0, 1.0, 1.0, 0, None, 0,
                          0, Vector3(), False, [])],
            'test': [
                soMessage(None, Vector3(5, 3, 2), -1, 3.0, 1.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(7, 2, 3), -1, 3.0, 1.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(1, 2, 6), 1, 4.0, 1.0, 1.0, 0, None, 0,
                          0, Vector3(), False, [])]}

        # goal reached
        bffr._own_pos = [soMessage(None, Vector3(2, 2, 2), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_goal_reached(), True)
        bffr._own_pos = [soMessage(None, Vector3(2, 2, 1), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_goal_reached(), True)
        # goal not reached
        bffr._own_pos = [soMessage(None, Vector3(6, 7, 2), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_goal_reached(), False)
        # only consider some frameIDs
        bffr._own_pos = [soMessage(None, Vector3(2, 2, 2), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_goal_reached(frameids=['gradient']), True)
        self.assertEqual(bffr.get_goal_reached(frameids=['None', 'test']),
                         False)

    # no potential
    def test_get_no_potential(self):
        """
        test get_no_potential method
        :return:
        """
        bffr = SoBuffer()

        # no gradients available --> True
        bffr._own_pos = [soMessage(None, Vector3(2, 2, 2), -1, 3.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_no_potential(), True)

        bffr._static = {
            'gradient': [
                soMessage(None, Vector3(2, 3, 1), 1, 3.0, 1.0, 1.0, 0, None,
                          0, 0, Vector3(), False, []),
                soMessage(None, Vector3(2, 2, 2), 1, 1.0, 1.0, 1.0, 0, None,
                          0, 0, Vector3(), False, [])],
            'None': [
                soMessage(None, Vector3(7, 3, 2), -1, 3.0, 1.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(5, 6, 3), 1, 2.0, 1.0, 1.0, 0, None, 0,
                          0, Vector3(), False, [])],
            'test': [
                soMessage(None, Vector3(5, 3, 2), -1, 3.0, 1.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(7, 2, 3), -1, 3.0, 1.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(1, 2, 6), 1, 4.0, 1.0, 1.0, 0, None, 0,
                          0, Vector3(), False, [])]}

        # potential
        bffr._own_pos = [soMessage(None, Vector3(2, 2, 2), -1, 3.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_no_potential(), False)
        bffr._own_pos = [soMessage(None, Vector3(2, 2, 1), -1, 3.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_no_potential(), False)
        # no potential
        bffr._own_pos = [soMessage(None, Vector3(0, 9, 9), -1, 3.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_no_potential(), True)

        # only consider some frameIDs
        # no potential
        bffr._own_pos = [soMessage(None, Vector3(2, 2, 2), -1, 3.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_no_potential(frameids=['gradient']), True)
        # potential
        self.assertEqual(bffr.get_no_potential(frameids=['None', 'test']),
                         False)

    def test_get_neighbors_bool(self):
        """
        test get_neighbors_bool method
        :return:
        """

        bffr = SoBuffer(view_distance=2.0, collision_avoidance='repulsion')

        # no gradients available --> True
        bffr._own_pos = [soMessage(None, Vector3(2, 2, 2), -1, 3.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr.get_neighbors_bool(), True)

        bffr._moving = {
            'gradient': [soMessage(None, Vector3(2, 3, 1), -1, 3.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, []),
                         soMessage(None, Vector3(2, 1, 2), -1, 1.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        }

        #self.assertEqual(bffr.get_neighbors_bool(), False)

        bffr._own_pos = [soMessage(None, Vector3(9, 9, 9), -1, 3.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        #self.assertEqual(bffr.get_neighbors_bool(), True)

    # GRADIENT CALCULATIONS
    def test_calc_attractive_gradient(self):
        """
        test _calc_attractive_gradient method for 2D and 3D
        """
        bffr = SoBuffer()

        # 2D - D < r <= C
        gradient = soMessage(None, Vector3(3, 4, 0), 1, 5.0, 1.0, 1.0, 0, None,
                             0, 0,
                             Vector3(), False, [])
        bffr._own_pos = [soMessage(None, Vector3(0, 0, 0), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr._calc_attractive_gradient(gradient),
                         Vector3(0.6 * 0.8, 0.8 * 0.8, 0))

        # 2D - r > C
        gradient = soMessage(None, Vector3(3, 4, 0), 1, 2.0, 1.0, 1.0, 0, None,
                             0, 0,
                             Vector3(), False, [])
        self.assertEqual(bffr._calc_attractive_gradient(gradient),
                         Vector3(0.6, 0.8, 0))

        # 2D - r <= D
        gradient = soMessage(None, Vector3(3, 4, 0), 1, 2.0, 5.0, 1.0, 0, None,
                             0, 0,
                             Vector3(), False, [])
        self.assertEqual(bffr._calc_attractive_gradient(gradient),
                         Vector3(0, 0, 0))

        # 2D - r > C - non zero robot pose
        gradient = soMessage(None, Vector3(4, 5, 0), 1, 2.0, 1.0, 1.0, 0, None,
                             0, 0,
                             Vector3(), False, [])
        bffr._own_pos = [soMessage(None, Vector3(1, 1, 0), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr._calc_attractive_gradient(gradient),
                         Vector3(0.6, 0.8, 0))

        # 3D - D < r <= C
        gradient = soMessage(None, Vector3(3, 5, 10), 1, 6.0, 2.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        bffr._own_pos = [soMessage(None, Vector3(1, 2, 4), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        result = Vector3((2.0 / 7.0) * (5.0 / 6.0), (3.0 / 7.0) * (5.0 / 6.0),
                         (6.0 / 7.0) * (5.0 / 6.0))
        self.assertEqual(bffr._calc_attractive_gradient(gradient), result)

        # 3D - r > C
        gradient = soMessage(None, Vector3(3, 5, 10), 1, 5.0, 2.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        result = Vector3((2.0 / 7.0), (3.0 / 7.0), (6.0 / 7.0))
        self.assertEqual(bffr._calc_attractive_gradient(gradient), result)

        # 3D - r <= D
        gradient = soMessage(None, Vector3(3, 5, 10), 1, 5.0, 7.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        self.assertEqual(bffr._calc_attractive_gradient(gradient),
                         Vector3(0, 0, 0))

    def test_calc_repulsive_gradient(self):
        """
        test _calc_repulsive_gradient method
        """
        bffr = SoBuffer()

        # 2D - D < r <= C
        gradient = soMessage(None, Vector3(3, 4, 0), -1, 5.0, 1.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        bffr._own_pos = [soMessage(None, Vector3(0, 0, 0), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr._calc_repulsive_gradient(gradient),
                         Vector3(-0.6 * 0.2, -0.8 * 0.2, 0))

        # 2D - r > C
        gradient = soMessage(None, Vector3(3, 4, 0), -1, 2.0, 1.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        self.assertEqual(bffr._calc_repulsive_gradient(gradient),
                         Vector3(0, 0, 0))

        # 2D - r <= D
        gradient = soMessage(None, Vector3(3, 4, 0), -1, 2.0, 5.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        self.assertEqual(bffr._calc_repulsive_gradient(gradient),
                         Vector3(-1.0 * np.inf, -1.0 * np.inf, np.inf))

        # 2D - r > C - non zero robot pose
        gradient = soMessage(None, Vector3(4, 5, 0), -1, 2.0, 1.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        bffr._own_pos = [soMessage(None, Vector3(1, 1, 0), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr._calc_repulsive_gradient(gradient),
                         Vector3(0, 0, 0))

        # 3D - D < r <= C
        gradient = soMessage(None, Vector3(3, 5, 10), -1, 6.0, 2.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        bffr._own_pos = [soMessage(None, Vector3(1, 2, 4), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        result = Vector3((-2.0 / 7.0) * (1.0 / 6.0), (-3.0 / 7.0) *
                         (1.0 / 6.0), (-6.0 / 7.0) * (1.0 / 6.0))
        self.assertEqual(bffr._calc_repulsive_gradient(gradient), result)

        # 3D - r > C
        gradient = soMessage(None, Vector3(3, 5, 10), -1, 5.0, 2.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        self.assertEqual(bffr._calc_repulsive_gradient(gradient),
                         Vector3(0, 0, 0))

        # 3D - r <= D
        gradient = soMessage(None, Vector3(3, 5, 10), -1, 5.0, 7.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        self.assertEqual(bffr._calc_repulsive_gradient(gradient),
                         Vector3(-1 * np.inf, -1 * np.inf, -1 * np.inf))

    def test_calc_attractive_gradient_ge(self):
        """
        test calc attractive gradient method based on Ge & Cui paper
        :return:
        """
        bffr = SoBuffer()
        # robot within diffusion radius + goal radius of gradient
        gradient = soMessage(None, Vector3(3, 5, 10), -1, 10.0, 2.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        bffr._own_pos = [soMessage(None, Vector3(0, 0, 0), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr._calc_attractive_gradient_ge(gradient),
                         Vector3(3, 5, 10))

        # robot within goal radius of gradient
        bffr._own_pos = [soMessage(None, Vector3(3, 7, 10), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr._calc_attractive_gradient_ge(gradient),
                         Vector3(0, -2, 0))

        # robot without radius + goal radius of gradient, but gradient is
        # within view_distance
        gradient = soMessage(None, Vector3(2, 3, 6), -1, 4.0, 2.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        bffr._own_pos = [soMessage(None, Vector3(0, 0, 0), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr._calc_attractive_gradient_ge(gradient),
                         Vector3((2.0 / 7.0) * 6.0, (3.0 / 7.0) * 6.0,
                                 (6.0 / 7.0) * 6.0))

    def test_repulsive_gradient_ge(self):
        """
        test repulsion vector calculation based on Ge
        :return:
        """
        bffr = SoBuffer()
        gradient = soMessage(None, Vector3(4, 2, 0), -1, 4.0, 2.0, 1.0, 0,
                             None, 0,
                             0, Vector3(), False, [])
        goal = soMessage(None, Vector3(1, 0, 0), 1, 3.0, 1.0, 1.0, 0, None, 0,
                         0,
                         Vector3(), False, [])

        # diffusion and goal_radius of gradient shorter than distance
        bffr._own_pos = [soMessage(None, Vector3(8, 8, 0), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr._calc_repulsive_gradient_ge(gradient, goal),
                         Vector3())

        # agent within goal area of repulsive gradient
        bffr._own_pos = [soMessage(None, Vector3(3, 2, 0), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        self.assertEqual(bffr._calc_repulsive_gradient_ge(gradient, goal),
                         Vector3(-np.inf, np.inf, np.inf))

        # robot within reach of gradient
        bffr._own_pos = [soMessage(None, Vector3(1, -2, 0), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]
        v = bffr._calc_repulsive_gradient_ge(gradient, goal)
        v.x = round(v.x, 4)
        v.y = round(v.y, 4)
        v.z = round(v.z, 4)
        self.assertEqual(v,
                         Vector3(round((1.0 - 4.0) / 5.0 * (1.0 / 3.0 -
                                                            1.0 / 4.0) *
                                       (2.0 / 3.0), 4),
                                 round(0.5 * ((1.0 / 3.0 - 1.0 / 4.0) ** 2) +
                                       -0.8 * (1.0 / 3.0 - 1.0 / 4.0) * (
                                           2.0 / 3.0), 4),
                                 0.0))

    # EVAPORATION
    def test_evaporation_buffer(self):
        """
        test evaporation of buffer data using evaporate_buffer method
        """

        bffr = SoBuffer(aggregation='max', min_diffusion=1.0)
        now = rospy.Time.now()

        data = {'None': [  # message has goal radius - should be kept
            soMessage(Header(None, now - rospy.Duration(45), 'None'),
                      Vector3(1, 1, 0), 1, 4.0, 1.0, 0.8, 5,
                      now - rospy.Duration(45), 0, 0,
                      Vector3(), False, []),
            # evaporation time is zero cases
            soMessage(Header(None, now - rospy.Duration(45), 'None'),
                      Vector3(1, 1, 0), 1, 4.0, 1.0, 0.8, 0,
                      now - rospy.Duration(45), 0, 0,
                      Vector3(), False, []),
            soMessage(Header(None, now - rospy.Duration(45), 'None'),
                      Vector3(1, 1, 0), 1, 4.0, 0.0, 1.0, 0,
                      now - rospy.Duration(45), 0, 0,
                      Vector3(), False, []),
            soMessage(Header(None, now - rospy.Duration(45), 'None'),
                      Vector3(1, 1, 0), 1, 4.0, 1.0, 1.0, 0,
                      now - rospy.Duration(45), 0, 0,
                      Vector3(), False, []),
            # messages without goal radius - will be sorted out based on
            # min diffusion
            soMessage(Header(None, now - rospy.Duration(20), 'None'),
                      Vector3(2, 2, 0), 1, 4.0, 0.0, 0.75, 5,
                      now - rospy.Duration(20), 0, 0,
                      Vector3(), False, []),
            soMessage(Header(None, now - rospy.Duration(5), 'None'),
                      Vector3(5, 5, 0), 1, 4.0, 0.0, 0.8, 3,
                      now - rospy.Duration(5), 0, 0,
                      Vector3(), False, []),
            soMessage(Header(None, now, 'None'), Vector3(6, 6, 0),
                      1, 4.0, 0.0, 0.8, 5, now, 0, 0, Vector3(), False, [])
        ], 'gradient': [
            soMessage(Header(None, now - rospy.Duration(45), 'gradient'),
                      Vector3(1, 1, 0), 1, 4.0, 0.0, 0.8, 5,
                      now - rospy.Duration(45), 0, 0,
                      Vector3(), False, []),
            soMessage(Header(None, now - rospy.Duration(15), 'gradient'),
                      Vector3(3, 3, 0), 1, 4.0, 0.0, 0.6, 5,
                      now - rospy.Duration(15), 0,
                      0, Vector3(), False, [])
        ], 'robo': [
            soMessage(Header(None, now - rospy.Duration(10), 'robo'),
                      Vector3(4, 4, 0), 1, 4.0, 0.0, 0.8, 4,
                      now - rospy.Duration(10), 0, 0,
                      Vector3(), False, [])]}

        bffr._static = deepcopy(data)
        bffr._evaporate_buffer()

        data = {'None': [
            soMessage(Header(None, now - rospy.Duration(45), 'None'),
                      Vector3(1, 1, 0), 1, 4.0 *
                      (0.8 ** 9), 1.0, 0.8, 5, now, 0, 0, Vector3(), False,
                      []),
            soMessage(Header(None, now - rospy.Duration(45), 'None'),
                      Vector3(1, 1, 0), 1, 0.0, 1.0, 0.8, 0,
                      now - rospy.Duration(45), 0, 0,
                      Vector3(), False, []),
            soMessage(Header(None, now - rospy.Duration(45), 'None'),
                      Vector3(1, 1, 0), 1, 4.0, 0.0, 1.0, 0,
                      now - rospy.Duration(45), 0,
                      0, Vector3(), False, []),
            soMessage(Header(None, now - rospy.Duration(45), 'None'),
                      Vector3(1, 1, 0), 1, 4.0, 1.0, 1.0, 0,
                      now - rospy.Duration(45), 0,
                      0, Vector3(), False, []),
            soMessage(Header(None, now - rospy.Duration(20), 'None'),
                      Vector3(2, 2, 0), 1,
                      4.0 * (0.75 ** 4), 0.0, 0.75, 5, now, 0, 0,
                      Vector3(), False, []),
            soMessage(Header(None, now - rospy.Duration(5), 'None'),
                      Vector3(5, 5, 0), 1, 4.0 * 0.8, 0.0, 0.8, 3,
                      now - rospy.Duration(2), 0,
                      0, Vector3(), False, []),
            soMessage(Header(None, now, 'None'), Vector3(6, 6, 0),
                      1, 4.0, 0.0, 0.8, 5, now, 0, 0, Vector3(), False, [])],
            'gradient': [],
            'robo': [
                soMessage(Header(None, now - rospy.Duration(10), 'robo'),
                          Vector3(4, 4, 0), 1, 4.0 * (0.8 ** 2), 0.0,
                          0.8, 4, now - rospy.Duration(2), 0, 0, Vector3(),
                          False, []),

            ]}

        self.assertEqual(bffr._static, data)

    def test_evaporation_msg(self):
        """
        test the evaporation of one specified message
        :return:
        """
        bffr = SoBuffer(min_diffusion=1.0)
        now = rospy.Time.now()

        # with goal radius --> should be kept
        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'),
                        Vector3(1, 1, 0), 1, 4.0, 1.0, 0.8, 5,
                        now - rospy.Duration(45), 0, 0, Vector3(),
                        False, [])
        result = soMessage(Header(None, now - rospy.Duration(45), 'None'),
                           Vector3(1, 1, 0), 1,
                           4.0 * (0.8 ** 9), 1.0, 0.8, 5, now, 0, 0,
                           Vector3(), False, [])
        self.assertEqual(bffr._evaporate_msg(msg), result)

        # without goal radius --> should be deleted
        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'),
                        Vector3(1, 1, 0), 1, 4.0, 0.0, 0.8, 5,
                        now - rospy.Duration(45), 0, 0,
                        Vector3(), False, [])
        self.assertEqual(bffr._evaporate_msg(msg), None)

        # without goal radius & ev time is 0, ev factor < 1
        # --> should be deleted
        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'),
                        Vector3(1, 1, 0), 1, 4.0, 0.0, 0.8, 0,
                        now - rospy.Duration(45), 0, 0,
                        Vector3(), False, [])
        self.assertEqual(bffr._evaporate_msg(msg), None)

        # without goal radius & ev time is 0, ev factor == 1.0
        # --> kept as it is
        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'),
                        Vector3(1, 1, 0), 1, 4.0, 0.0, 1.0, 0,
                        now - rospy.Duration(45), 0, 0,
                        Vector3(), False, [])
        result = soMessage(Header(None, now - rospy.Duration(45), 'None'),
                           Vector3(1, 1, 0), 1, 4.0, 0.0, 1.0, 0,
                           now - rospy.Duration(45), 0, 0,
                           Vector3(), False, [])
        self.assertEqual(bffr._evaporate_msg(msg), result)

        # with goal radius & ev time is 0, ev factor < 1.0
        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'),
                        Vector3(1, 1, 0), 1, 4.0, 1.0, 0.0, 0,
                        now - rospy.Duration(45), 0, 0,
                        Vector3(), False, [])
        result = soMessage(Header(None, now - rospy.Duration(45), 'None'),
                           Vector3(1, 1, 0), 1, 0.0, 1.0, 0.0, 0,
                           now - rospy.Duration(45), 0, 0,
                           Vector3(), False, [])
        self.assertEqual(bffr._evaporate_msg(msg), result)

    # STORE DATA IN SoBUFFER
    def test_store_data_max(self):
        """
        test store_data method aggregation option = max
        """
        bffr = SoBuffer(aggregation={'DEFAULT': 'max'},
                        aggregation_distance=1.0)
        testlist = []

        msg = soMessage(None, Vector3(2, 0.8, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 2, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 2, 0), 1, 5.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        # store max value within aggregation distance!
        msg = soMessage(None, Vector3(2, 1.5, 0), 1, 6.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        # keep max of both
        msg = soMessage(None, Vector3(3, 3, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 3.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        # only value at this postion / area
        msg = soMessage(None, Vector3(5, 5, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_max_aggregation_distance(self):
        """
        test store_data method aggregation option = max, aggregation distance = 0.0
        """
        bffr = SoBuffer(aggregation={'DEFAULT': 'max'},
                        aggregation_distance=0.0)
        testlist = []

        msg = soMessage(None, Vector3(2, 2, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 2, 0), 1, 5.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        # store max value within aggregation distance!
        msg = soMessage(None, Vector3(2, 1, 0), 1, 6.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        # keep max of both
        msg = soMessage(None, Vector3(3, 3, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 3.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        # only value at this postion / area
        msg = soMessage(None, Vector3(5, 5, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_fids(self):
        """
        test store_data method with different frame IDs (all)
        """
        bffr = SoBuffer(aggregation={'DEFAULT': 'max'})
        testlist = {'None': [], 'grad': []}
        now = rospy.Time.now()

        msg = soMessage(Header(None, now, 'None'), Vector3(2, 2, 0), 1, 4.0,
                        1.0,
                        1.0, 0, None, 0, 0, Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'grad'), Vector3(3, 3, 0), 1, 4.0,
                        1.0,
                        1.0, 0, None, 0, 0, Vector3(), False, [])
        testlist['grad'].append(msg)
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'None'), Vector3(3, 3, 0), 1, 3.0,
                        1.0,
                        1.0, 0, None, 0, 0, Vector3(), False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'grad'), Vector3(5, 5, 0), 1, 4.0,
                        1.0,
                        1.0, 0, None, 0, 0, Vector3(), False, [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'None'), Vector3(2, 2, 0), 1, 5.0,
                        1.0,
                        1.0, 0, None, 0, 0, Vector3(), False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg)

        self.assertEqual(bffr._static, testlist)

    def test_store_data_fids_selection(self):
        """
        test store_data method, only store gradients with specified frameids
        """
        bffr = SoBuffer(aggregation={'DEFAULT': 'max'}, store_all=False,
                        framestorage=['grad', 'silk'])
        testlist = {'grad': [], 'silk': []}
        now = rospy.Time.now()

        msg = soMessage(Header(None, now, 'pheromone'), Vector3(2, 2, 0), 1,
                        4.0, 1.0, 1.0, 0, None, 0, 0, Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'grad'), Vector3(3, 3, 0), 1, 4.0,
                        1.0,
                        1.0, 0, None, 0, 0, Vector3(), False, [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 3.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'grad'), Vector3(5, 5, 0), 1, 4.0,
                        1.0,
                        1.0, 0, None, 0, 0, Vector3(), False, [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'silk'), Vector3(5, 5, 0), 1, 4.0,
                        1.0,
                        1.0, 0, None, 0, 0, Vector3(), False, [])
        testlist['silk'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 2, 0), 1, 5.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        self.assertEqual(bffr._static, testlist)

    def test_store_data_min(self):
        """
        test store_data method aggregation option = min
        """
        bffr = SoBuffer(aggregation={'DEFAULT': 'min'})
        testlist = []

        msg = soMessage(None, Vector3(2, 2, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 3.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 2, 0), 1, 5.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_avg(self):
        """
        test store_data method aggregation option = avg
        """
        bffr = SoBuffer(aggregation={'DEFAULT': 'avg'}, min_diffusion=1.0)
        testlist = []

        msg = soMessage(None, Vector3(2, 2, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 3.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(soMessage(None, Vector3(3, 3, 0), 1, 3.5, 1.0, 1.0, 0,
                                  None, 0, 0, Vector3(), False, []))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 1, 0), -1, 5.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(soMessage(None, Vector3(2, 1.5, 0), -1, 1.0, 0.0, 1.0,
                                  0, None, 0, 0, Vector3(), False, []))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), -1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_newest(self):
        """
        test store_data method aggregation option = newest
        """
        bffr = SoBuffer(aggregation={'DEFAULT': 'newest'})
        testlist = []

        msg = soMessage(None, Vector3(2, 2, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 3.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), 1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 2, 0), -1, 5.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), -1, 4.0, 1.0, 1.0, 0, None, 0,
                        0,
                        Vector3(), False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_ev(self):
        """
        test store_data method, evaporation of received data
        """
        bffr = SoBuffer(aggregation={'DEFAULT': 'max'})
        testlist = {'None': []}
        now = rospy.Time.now()

        # not to be stored - ev time is zero, no goal radius
        msg = soMessage(None, Vector3(2, 2, 0), 1, 4.0, 0.0, 0.3, 0, None, 0,
                        0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        # received data is older than stored one, will be evaporated and than compared
        msg = soMessage(Header(None, now - rospy.Duration(10), None),
                        Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5,
                        now - rospy.Duration(10), 0,
                        0, Vector3(), False, [])
        result = soMessage(Header(None, now - rospy.Duration(10), 'None'),
                           Vector3(2, 2, 0), 1,
                           4.0 * (0.8 ** 2), 1.0, 0.8, 5, now, 0, 0,
                           Vector3(), False, [])
        testlist['None'].append(result)
        bffr.store_data(msg)

        msg = soMessage(Header(None, now - rospy.Duration(15), None),
                        Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5,
                        now - rospy.Duration(15), 0, 0,
                        Vector3(), False, [])
        bffr.store_data(msg)

        self.assertEqual(bffr._static, testlist)

    def test_store_data_neighbors(self):
        """
        test store_data method, store neighbor data and own pos
        """
        bffr = SoBuffer(aggregation='max', id='robot3')
        testlist = {'robot1': [], 'robot2': []}
        ownpos = []
        now = rospy.Time.now()

        # replaced by fourth robot1 message
        msg = soMessage(Header(None, now - rospy.Duration(10), 'robot1'),
                        Vector3(2.1, 2.2, 0), 1, 4.0, 1.0, 0.8, 5, None, 0, 0,
                        Vector3(), True, [])
        bffr.store_data(msg)

        # position older than newest stored position --> ignore
        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot1'),
                        Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, None, 0, 0,
                        Vector3(), True, [])
        bffr.store_data(msg)

        # add to position list
        msg = soMessage(Header(None, now - rospy.Duration(5), 'robot1'),
                        Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, None, 0, 0,
                        Vector3(), True, [])
        testlist['robot1'].append(msg)
        bffr.store_data(msg)

        # add to position list
        msg = soMessage(Header(None, now, 'robot1'), Vector3(2, 2, 0), 1, 4.0,
                        1.0, 0.8, 5, None, 0, 0, Vector3(), True, [])
        testlist['robot1'].append(msg)
        bffr.store_data(msg)

        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot2'),
                        Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, None, 0, 0,
                        Vector3(), True, [])
        testlist['robot2'].append(msg)
        bffr.store_data(msg)

        # own position
        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot3'),
                        Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, None, 0, 0,
                        Vector3(), True, [])
        ownpos.append(msg)
        bffr.store_data(msg)

        self.assertEqual(bffr._moving, testlist)
        self.assertEqual(bffr._own_pos, ownpos)

    def test_store_data_neighbors_False(self):
        """
        test store_data method, neighbor gradients will not be stored
        :return:
        """
        bffr = SoBuffer(aggregation='max', moving_storage_size=0, id='')
        now = rospy.Time.now()

        # replaced by fourth robot1 message
        msg = soMessage(Header(None, now - rospy.Duration(10), 'robot1'),
                        Vector3(2.1, 2.2, 0), 1, 4.0, 1.0, 0.8, 5, None, 0, 0,
                        Vector3(), True, [])
        bffr.store_data(msg)

        # position older than newest stored position --> ignore
        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot1'),
                        Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, None, 0, 0,
                        Vector3(), True, [])
        bffr.store_data(msg)

        # add to position list
        msg = soMessage(Header(None, now - rospy.Duration(5), 'robot1'),
                        Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, None, 0, 0,
                        Vector3(), True, [])
        bffr.store_data(msg)

        # add to position list
        msg = soMessage(Header(None, now, 'robot1'), Vector3(2, 2, 0), 1, 4.0,
                        1.0, 0.8, 5, None, 0, 0, Vector3(), True, [])
        bffr.store_data(msg)

        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot2'),
                        Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, None, 0, 0,
                        Vector3(), True, [])
        bffr.store_data(msg)

        # own position
        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot3'),
                        Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, None, 0, 0,
                        Vector3(), True, [])
        bffr.store_data(msg)

        self.assertEqual(bffr._moving, {})
        self.assertEqual(bffr._own_pos, [])

    def test_store_data_various_aggregations(self):
        """
        test store data method with different aggregation options for different frame IDs
        :return:
        """
        bffr = SoBuffer(aggregation={'DEFAULT': 'max', 'grad': 'avg'})
        testlist = {'None': [], 'grad': []}
        now = rospy.Time.now()

        msg = soMessage(Header(None, now, 'None'), Vector3(2, 2, 0), 1, 4.0,
                        1.0, 1.0, 0, None, 0, 0, Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'grad'), Vector3(4, 5, 0), 1, 4.0,
                        1.0, 1.0, 0, None, 0, 0, Vector3(), False, [])
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'None'), Vector3(3, 3, 0), 1, 3.0,
                        1.0, 1.0, 0, None, 0, 0, Vector3(), False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'grad'), Vector3(5, 5, 0), 1, 4.0,
                        1.0, 1.0, 0, None, 0, 0, Vector3(), False, [])
        testlist['grad'].append(soMessage(Header(None, now, 'grad'),
                                          Vector3(4.5, 5, 0), 1, 4.0, 1.0, 1.0,
                                          0, None, 0, 0, Vector3(), False, []))
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'None'), Vector3(2, 2, 0), 1, 5.0,
                        1.0, 1.0, 0, None, 0, 0, Vector3(), False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg)

        self.assertEqual(bffr._static, testlist)

    # Collision Avoidance / Repulsion
    def test_gradient_repulsion(self):
        """
        test gradient repulsion method based on gradients
        :return:
        """

        bffr = SoBuffer(id='robot1')

        # no own position specified
        self.assertEqual(bffr._gradient_repulsion(), Vector3())

        bffr._own_pos = [
            soMessage(Header(None, rospy.Time.now(), 'None'), Vector3(2, 4, 0),
                      1, 4.0, 0.0, 1.0, 0, None, 0, 0, Vector3(), False, []),
            soMessage(Header(None, rospy.Time.now(), 'None'), Vector3(2, 2, 0),
                      1, 2.0, 0.0, 1.0, 0, None, 0, 0, Vector3(), False, [])
        ]

        # no neighbors specified
        self.assertEqual(bffr._gradient_repulsion(), Vector3())

        bffr._moving = {
            'robot2': [
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(1, 3, 0), -1, 1.0, 1.0, 1.0, 0, None, 0, 0,
                          Vector3(), False, [])

            ],
            'robot3': [
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(2, 2, 0), -1, 4.0, 1.0, 1.0, 0, None, 0, 0,
                          Vector3(), False, []),
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(3, 2, 0), -1, 1.0, 0.8, 1.0, 0, None, 0, 0,
                          Vector3(), False, [])
            ]
        }
        # calculate resulting vector
        result = bffr._gradient_repulsion()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        # calculate vector
        self.assertEqual(result, Vector3(-0.39, -0.41, 0.0))

        # neighbor within goal_radius - returns vector with
        # ||vector|| = repulsion_radius
        bffr._moving = {
            'robot2': [
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(2, 1.5, 0), -1, 2.0, 1.0, 1.0, 0, None, 0, 0,
                          Vector3(), False, [])
            ]
        }

        d = round(so_data.calc.vector_length(bffr._gradient_repulsion()), 0)

        # calculate vector
        self.assertEqual(d, 2.0)

    def test_repulsion_vector(self):
        """
        test gradient repulsion method based on Fernandez-Marquez et al.
        :return:
        """
        bffr = SoBuffer(id='robot1', view_distance=2.0)

        # no own position specified
        self.assertEqual(bffr._gradient_repulsion(), Vector3())

        bffr._own_pos = [
            soMessage(Header(None, rospy.Time.now(), 'None'), Vector3(2, 4, 0),
                      -1, 4.0, 1.0, 1.0, 0, rospy.Time.now(), 0, 0, Vector3(),
                      False, []),
            soMessage(Header(None, rospy.Time.now(), 'None'), Vector3(2, 2, 0),
                      -1, 1.0, 0.0, 1.0, 0, rospy.Time.now(), 0, 0, Vector3(),
                      False, [])
        ]

        # no neighbors specified
        self.assertEqual(bffr._repulsion_vector(), Vector3())

        bffr._moving = {
            'robot2': [
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(1, 3, 0), -1, 1.0, 1.0, 1.0, 0,
                          rospy.Time.now(), 0, 0,
                          Vector3(), False, [])

            ],
            'robot3': [
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(2, 2, 0), -1, 4.0, 1.0, 1.0, 0,
                          rospy.Time.now(), 0, 0,
                          Vector3(), False, []),
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(3, 2, 0), -1, 1.0, 0.8, 1.0, 0,
                          rospy.Time.now(), 0, 0,
                          Vector3(), False, [])
            ],
            'robot4': []
        }
        # calculate resulting vector
        result = bffr._repulsion_vector()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        # calculate vector
        self.assertEqual(result, Vector3(-0.88, -0.48, 0))

        # neighbor within goal_radius - returns vector with
        # ||vector|| = repulsion_radius
        bffr._moving = {
            'robot2': [
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(2, 2, 0), -1, 2.0, 1.0, 1.0, 0,
                          rospy.Time.now(), 0, 0,
                          Vector3(), False, [])
            ]
        }

        d = round(so_data.calc.vector_length(bffr._repulsion_vector()), 0)

        # calculate vector
        self.assertEqual(d, 1.0)

    def test_get_collision_avoidance(self):
        """
        test get collision avoidance method
        :return:
        """

        bffr = SoBuffer(id='robot1', collision_avoidance='gradient', result='',
                        max_velocity=2.0, result_static=False, view_distance=1.5)

        bffr._own_pos = [
            soMessage(Header(None, rospy.Time.now(), 'None'), Vector3(2, 4, 0),
                      -1, 5.0, 0.0, 1.0, 0, None, 0, 0,
                      Vector3(), False, []),
            soMessage(Header(None, rospy.Time.now(), 'None'), Vector3(2, 2, 0),
                      -1, 1.0, 0.0, 1.0, 0, None, 0, 0,
                      Vector3(), False, [])
        ]

        bffr._moving = {
            'robot2': [
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(1, 3, 0), -1, 1.0, 1.0, 1.0, 0, None, 0, 0,
                          Vector3(), False, [])

            ],
            'robot3': [
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(2, 2, 0), -1, 4.0, 1.0, 1.0, 0, None, 0, 0,
                          Vector3(), False, []),
                soMessage(Header(None, rospy.Time.now(), 'None'),
                          Vector3(3, 2, 0), -1, 1.0, 0.8, 1.0, 0, None, 0, 0,
                          Vector3(), False, [])
            ]
        }

        # calculate resulting vector
        result = bffr.get_current_gradient()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        # calculate vector
        self.assertEqual(result, Vector3(-0.39, -0.41, 0.0))

        bffr.collision_avoidance = 'repulsion'

        # calculate resulting vector
        result = bffr.get_current_gradient()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        # calculate vector
        self.assertEqual(result, Vector3(-0.88, -0.48, 0.0))

    # Aggregation return vectors
    def test_aggregate_max(self):
        """
        test aggregate max method
        :return:
        """
        bffr = SoBuffer(result='max', result_moving=False)

        bffr._own_pos = [soMessage(None, Vector3(1, 2, 3), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]

        bffr._moving = {
            'robot1': [soMessage(None, Vector3(1, 2, 3), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])],
            'robot2': []
        }

        bffr._static = {
            'gradient': [soMessage(None, Vector3(2, 3, 1), 1, 3.0, 1.0, 1.0, 0,
                                   None, 0, 0, Vector3(), False, []),
                         soMessage(None, Vector3(2, 2, 2), 1, 1.0, 1.0, 1.0, 0,
                                   None, 0, 0, Vector3(), False, [])],
            'None': [
                soMessage(None, Vector3(7, 3, 2), -1, 3.0, 1.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(5, 6, 3), 1, 2.0, 1.0, 1.0, 0, None, 0,
                          0, Vector3(), False, [])],
            'test': [
                soMessage(None, Vector3(5, 3, 2), -1, 3.0, 1.0, 1.0, 0, None,
                          0, 0, Vector3(), False, []),
                soMessage(None, Vector3(7, 2, 3), -1, 3.0, 1.0, 1.0, 0, None,
                          0, 0, Vector3(), False, []),
                soMessage(None, Vector3(1, 2, 6), 1, 4.0, 1.0, 1.0, 0, None,
                          0, 0, Vector3(), False, [])]
        }

        # with all frameIDs
        result = bffr._aggregate_max()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))

        # only one frameID considered - no gradient within view
        result = bffr._aggregate_max(frameids=['None'])
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3())

        # two frameIDs
        result = bffr._aggregate_max(frameids=['None', 'gradient'])
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))

    def test_aggregate_nearest_repulsion(self):
        """
        test aggregate nearest repulsion method
        :return:
        """
        bffr = SoBuffer(result='near')
        bffr._own_pos = [soMessage(None, Vector3(1, 2, 3), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]

        bffr._static = {
            'gradient': [soMessage(None, Vector3(2, 3, 1), -1, 1.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, []),
                         soMessage(None, Vector3(2, 2, 2), 1, 1.0, 1.0, 1.0, 0,
                                   None, 0, 0, Vector3(), False, [])],
            'None': [soMessage(None, Vector3(0, 3, 2), -1, 3.0, 1.0, 1.0, 0,
                               None, 0, 0, Vector3(), False, []),
                     soMessage(None, Vector3(5, 6, 3), 1, 2.0, 1.0, 1.0, 0,
                               None, 0,
                               0, Vector3(), False, [])]
        }

        # only one frameID + repulsive gradient is not considered
        result = bffr._aggregate_nearest_repulsion(frameids=['gradient'])
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))

        # all frameIDs
        result = bffr._aggregate_nearest_repulsion()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(0.73, -0.44, 0.14))

    def test_aggregate_nearest_ge(self):
        """
        test aggregate nearest Ge method, do not consider moving gradients
        :return:
        """

        bffr = SoBuffer(result='reach', collision_avoidance='',
                        result_moving=False)
        bffr._own_pos = [soMessage(None, Vector3(1, -2, 0), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]

        bffr._static = {
            'gradient': [soMessage(None, Vector3(2, 3, 1), -1, 4.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, []),
                         soMessage(None, Vector3(1, 0, 0), 1, 3.0, 1.0, 1.0, 0,
                                   None, 0, 0,
                                   Vector3(), False, [])],
            'None': [
                soMessage(None, Vector3(4, 2, 0), -1, 4.0, 2.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(5, 6, 3), 1, 2.0, 1.0, 1.0, 0, None, 0,
                          0, Vector3(), False, [])]
        }

        bffr._moving = {
            'robot1': [soMessage(None, Vector3(1, -2, 1), -1, 4.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])],
            'robot2': []
        }

        # only one frameID + repulsive gradient is not considered
        pose = Vector3(1, -2, 0)
        result = bffr._aggregate_nearest_ge(frameids=['gradient'])
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.0, 2.0, 0.0))

        # all frameIDs
        result = bffr._aggregate_nearest_ge()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(-0.03, 1.96, 0.0))

    def test_aggregate_all(self):
        """
        test aggregate all method
        :return:
        """
        bffr = SoBuffer(result='all')
        bffr._static = {
            'gradient': [soMessage(None, Vector3(2, 3, 1), -1, 1.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, []),
                         soMessage(None, Vector3(2, 2, 2), 1, 1.0, 1.0, 1.0, 0,
                                   None, 0, 0,
                                   Vector3(), False, [])],
            'None': [
                soMessage(None, Vector3(0, 3, 2), -1, 3.0, 1.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(5, 6, 3), 1, 2.0, 1.0, 1.0, 0, None, 0,
                          0, Vector3(), False, [])]
        }

        bffr._own_pos = [soMessage(None, Vector3(1, 2, 3), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]

        # only one frameID + repulsive gradient is not considered as outside
        # view distance
        result = bffr._aggregate_all(frameids=['gradient'])
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))

        # all frameIDs - everything within view is aggregated
        result = bffr._aggregate_nearest_repulsion()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(0.73, -0.44, 0.14))

    def test_aggregate_avoid_all(self):
        """
        test aggregate avoid all method
        :return:
        """

        bffr = SoBuffer(result='all')
        bffr._static = {
            'gradient': [soMessage(None, Vector3(2, 3, 1), -1, 1.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, []),
                         soMessage(None, Vector3(2, 2, 2), 1, 1.0, 1.0, 1.0, 0,
                                   None, 0,
                                   0, Vector3(), False, [])],
            'None': [
                soMessage(None, Vector3(0, 3, 2), -1, 3.0, 1.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(5, 6, 3), 1, 2.0, 1.0, 1.0, 0, None, 0,
                          0,
                          Vector3(), False, [])]
        }

        bffr._own_pos = [soMessage(None, Vector3(1, 2, 3), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]

        # only one frameID + repulsive gradient is not considered as outside
        # view distance
        result = bffr._aggregate_avoid_all(frameids=['gradient'])
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(-0.41, 0.0, 0.41))

        # all frameIDs - everything within view is aggregated
        result = bffr._aggregate_avoid_all()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(0.02, -0.44, 0.85))


    def test_get_attractive_distance(self):
        """
        test get_attractive_distance method
        :return:
        """
        bffr = SoBuffer(result='near')

        bffr._own_pos = [soMessage(None, Vector3(1, 2, 3), -1, 3.0, 0.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, [])]

        bffr._static = {
            'gradient': [soMessage(None, Vector3(2, 3, 1), -1, 1.0, 1.0, 1.0,
                                   0, None, 0, 0, Vector3(), False, []),
                         soMessage(None, Vector3(2, 2, 2), 1, 1.0, 1.0, 1.0, 0,
                                   None, 0,
                                   0, Vector3(), False, [])],
            'None': [
                soMessage(None, Vector3(0, 3, 2), -1, 3.0, 1.0, 1.0, 0, None,
                          0,
                          0, Vector3(), False, []),
                soMessage(None, Vector3(5, 6, 3), 1, 2.0, 1.0, 1.0, 0, None, 0,
                          0,
                          Vector3(), False, [])]
        }

        self.assertEqual(bffr.get_attractive_distance(), np.sqrt(2)-1)







# run tests - start roscore before running tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()
