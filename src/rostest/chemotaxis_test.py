#! /usr/bin/env python2
"""
Created on 20.02.2017

@author: kaiser, hrabia

Unit test for chemotaxis.py
"""
import unittest
import rostest
import rospy
import numpy as np
from so_data.msg import SoMessage
from geometry_msgs.msg import Vector3, Quaternion
import so_data.calc as calc
from so_data.sobuffer import SoBuffer, AGGREGATION
from so_data.chemotaxis import *


class ChemotaxisTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test')

    # Aggregation return vectors
    def test_follow_strongest(self):
        """
        test follow strongest mechanism
        :return:
        """
        bffr = SoBuffer()
        chem = FollowStrongest(bffr, moving=False, static=True, maxvel=np.inf)

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 2, 3), Quaternion(), Vector3(),
                      -1, 3.0, 0.0, 1.0, 0, None, False, [])]

        bffr._moving = {
            'robot1': {'None': [
                SoMessage(None, None, Vector3(1, 2, 3), Quaternion(),
                          Vector3(), -1, 3.0, 0.0, 1.0, 0, None, False, [])]},
            'robot2': {}
        }

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), 1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(),
                          Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False, [])],
            'None': [
                SoMessage(None, None, Vector3(7, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(),
                          Vector3(), 1, 2.0, 1.0, 1.0, 0, None, False, [])],
            'test': [
                SoMessage(None, None, Vector3(5, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(7, 2, 3), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(1, 2, 6), Quaternion(),
                          Vector3(), 1, 4.0, 1.0, 1.0, 0, None, False, [])]
        }

        # with all frameIDs
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))

        # only one frameID considered - no gradient within view
        chem.frames = ['None']
        result = chem.move()
        self.assertEqual(result, None)

        # two frameIDs
        chem.frames = ['None', 'gradient']
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))

        bffr._static = {
            'test': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, [])]
        }

        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, Vector3())

    def test_chemotaxis_balch(self):
        """
        test chemotaxis balch mechanism
        :return:
        """
        bffr = SoBuffer()
        chem = ChemotaxisBalch(bffr, moving=False, static=True, maxvel=np.inf)
        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 2, 3), Quaternion(), Vector3(),
                      -1, 3.0, 0.0, 1.0, 0, None, False, [])]

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(),
                          Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False, [])],
            'None': [
                SoMessage(None, None, Vector3(0, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(),
                          Vector3(), 1, 2.0, 1.0, 1.0, 0, None, False, [])]
        }

        # only one frameID + repulsive gradient is not considered
        chem.frames = ['gradient']
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))

        # all frameIDs
        chem.frames = []
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.73, -0.44, 0.14))

        # not existing frame ID - no gradient will be followed
        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, None)

        bffr._static = {
            'test': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, [])]
        }

        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, Vector3())

        bffr._static = {
            'test': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, [])]
        }

        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, Vector3())

    def test_chemotaxis_ge(self):
        """
        test chemotaxis ge mechanism
        :return:
        """

        bffr = SoBuffer()
        chem = ChemotaxisGe(bffr, moving=False, static=True, maxvel=np.inf)

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, -2, 0), Quaternion(), Vector3(),
                      -1, 3.0, 0.0, 1.0, 0, None, False, [])]

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 4.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(1, 0, 0), Quaternion(),
                          Vector3(), 1, 3.0, 1.0, 1.0, 0, None, False, [])],
            'None': [
                SoMessage(None, None, Vector3(4, 2, 0), Quaternion(),
                          Vector3(), -1, 4.0, 2.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(),
                          Vector3(), 1, 2.0, 1.0, 1.0, 0, None, False, [])]
        }

        bffr._moving = {
            'robot1': {'None': [
                SoMessage(None, None, Vector3(1, -2, 1), Quaternion(),
                          Vector3(), -1, 4.0, 0.0, 1.0, 0, None, False, [])]},
            'robot2': {}
        }

        # only one frameID + repulsive gradient is not considered
        chem.frames = ['gradient']
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.0, 1.0, 0.0))

        # all frameIDs
        chem.frames = []
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(-0.02, 0.98, 0.0))

        # no gradient within view
        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, None)

        bffr._static = {
            'test': [
                SoMessage(None, None, Vector3(0, -1, 2), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, [])]
        }

        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, Vector3())

    def test_follow_all(self):
        """
        test aggregate all method
        :return:
        """
        bffr = SoBuffer()
        chem = FollowAll(bffr, moving=False, static=True, maxvel=np.inf)

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(),
                          Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False, [])],
            'None': [
                SoMessage(None, None, Vector3(0, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(),
                          Vector3(), 1, 2.0, 1.0, 1.0, 0, None, False, [])]
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 2, 3), Quaternion(), Vector3(),
                      -1, 3.0, 0.0, 1.0, 0, None, False, [])]

        # only one frameID + repulsive gradient is not considered as outside
        # view distance
        chem.frames = ['gradient']
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))

        # all frameIDs - everything within view is aggregated
        chem.frames = []
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.73, -0.44, 0.14))

        # noe gradient within view
        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, None)

        bffr._static = {
            'test': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, [])]
        }

        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, Vector3())

    def test_avoid_all(self):
        """
        test aggregate avoid all method
        :return:
        """

        bffr = SoBuffer()
        chem = AvoidAll(bffr, moving=False, static=True, maxvel=np.inf)

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(),
                          Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False, [])],
            'None': [
                SoMessage(None, None, Vector3(0, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(),
                          Vector3(), 1, 2.0, 1.0, 1.0, 0, None, False, [])]
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 2, 3), Quaternion(), Vector3(),
                      -1, 3.0, 0.0, 1.0, 0, None, False, [])]

        # only one frameID + repulsive gradient is not considered as outside
        # view distance
        chem.frames = ['gradient']
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(-0.41, 0.0, 0.41))

        # all frameIDs - everything within view is aggregated
        chem.frames = []
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.02, -0.44, 0.85))

        # no gradient within view
        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, None)

        bffr._static = {
            'test': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, [])]
        }

        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, Vector3())

    def test_collision_avoidance(self):
        """
        test aggregate avoid all method
        :return:
        """

        bffr = SoBuffer()
        chem = CollisionAvoidance(bffr, moving=True, static=True,
                                  maxvel=np.inf)

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, [])],
            'None': [
                SoMessage(None, None, Vector3(0, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(),
                          Vector3(), -1, 2.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(10, 20, 1), Quaternion(),
                          Vector3(), 1, 2.0, 1.0, 1.0, 0, None, False, [])]
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 2, 3), Quaternion(), Vector3(),
                      -1, 3.0, 0.0, 1.0, 0, None, False, [])]

        # only one frameID + repulsive gradient is not considered as outside
        # view distance
        chem.frames = ['gradient']
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(-0.41, 0.0, 0.41))

        # all frameIDs - everything within view is aggregated
        chem.frames = []
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.02, -0.44, 0.85))

        # no gradient within view
        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, None)

    def test_goal_gradient(self):
        """
        test goal gradient method of chemotaxisBalch Behaviour
        :return:
        """
        bffr = SoBuffer()
        chem = ChemotaxisBalch(bffr, moving=False, static=True,
                               maxvel=np.inf)

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 2, 3), Quaternion(), Vector3(),
                      -1, 3.0, 0.0, 1.0, 0, None, False, [])]

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(),
                          Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False, [])],
            'None': [
                SoMessage(None, None, Vector3(0, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 10, 0, None, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(),
                          Vector3(), 1, 2.0, 1.0, 1.0, 0, None, False, [])]
        }

        self.assertEqual(calc.vector_length(chem.goal_gradient()),
                         np.sqrt(2) - 1)

        # no gradient within view
        chem.frames = ['test']
        result = chem.move()
        self.assertEqual(result, None)

    def test_follow_min(self):
        """
        test FollowMin Mechanism
        :return:
        """
        bffr = SoBuffer()
        chem = FollowMin(bffr, moving=False, static=True, maxvel=np.inf)

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 2, 3), Quaternion(), Vector3(),
                      -1, 3.0, 0.0, 1.0, 0, None, False, [])]

        bffr._moving = {
            'robot1': {'None': [
                SoMessage(None, None, Vector3(1, 2, 3), Quaternion(),
                          Vector3(), -1, 3.0, 0.0, 1.0, 0, None, False, [])]},
            'robot2': {}
        }

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(),
                          Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False, [])],
            'None': [
                SoMessage(None, None, Vector3(7, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(),
                          Vector3(), 1, 2.0, 1.0, 1.0, 0, None, False, [])],
            'test': [
                SoMessage(None, None, Vector3(5, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(7, 2, 3), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(1, 2, 6), Quaternion(),
                          Vector3(), 1, 4.0, 1.0, 1.0, 0, None, False, [])]
        }

        # with all frameIDs
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(0.0, 0.0, 0.5))

        # only one frameID considered - no gradient within view
        chem.frames = ['None']
        result = chem.move()
        self.assertEqual(result, None)

        # two frameIDs
        chem.frames = ['None', 'gradient']
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))

    def test_follow_min_reach(self):
        """
        test FollowMin Mechanism
        :return:
        """
        bffr = SoBuffer()
        chem = FollowMinReach(bffr, moving=False, static=True, maxvel=np.inf)

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 2, 3), Quaternion(), Vector3(),
                      -1, 3.0, 0.0, 1.0, 0, None, False, [])]

        bffr._moving = {
            'robot1': {'None': [
                SoMessage(None, None, Vector3(1, 2, 3), Quaternion(),
                          Vector3(), -1, 3.0, 0.0, 1.0, 0, None, False, [])]},
            'robot2': {}
        }

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), 1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(),
                          Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False, [])],
            'None': [
                SoMessage(None, None, Vector3(7, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(),
                          Vector3(), 1, 2.0, 1.0, 1.0, 0, None, False, [])],
            'test': [
                SoMessage(None, None, Vector3(5, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(7, 2, 3), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(1, 2, 6), Quaternion(),
                          Vector3(), 1, 4.0, 1.0, 1.0, 0, None, False, [])]
        }

        # with all frameIDs
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))

        # only one frameID considered - no gradient within view
        chem.frames = ['None']
        result = chem.move()
        self.assertEqual(result, None)

        # two frameIDs
        chem.frames = ['None', 'test']
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.0, 0.0, 0.5))

    def test_follow_max_reach(self):
        """
        test FollowMin Mechanism
        :return:
        """
        bffr = SoBuffer()
        chem = FollowMaxReach(bffr, moving=False, static=True, maxvel=np.inf)

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 2, 3), Quaternion(), Vector3(),
                      -1, 3.0, 0.0, 1.0, 0, None, False, [])]

        bffr._moving = {
            'robot1': {'None': [
                SoMessage(None, None, Vector3(1, 2, 3), Quaternion(),
                          Vector3(), -1, 3.0, 0.0, 1.0, 0, None, False, [])]},
            'robot2': {}
        }

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(),
                          Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False, [])],
            'None': [
                SoMessage(None, None, Vector3(7, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(),
                          Vector3(), 1, 2.0, 1.0, 1.0, 0, None, False, [])],
            'test': [
                SoMessage(None, None, Vector3(5, 3, 2), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(7, 2, 3), Quaternion(),
                          Vector3(), -1, 3.0, 1.0, 1.0, 0, None, False, []),
                SoMessage(None, None, Vector3(1, 2, 6), Quaternion(),
                          Vector3(), 1, 4.0, 1.0, 1.0, 0, None, False, [])]
        }

        # with all frameIDs
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(0.0, 0.0, 0.5))

        # only one frameID considered - no gradient within view
        chem.frames = ['None']
        result = chem.move()
        self.assertEqual(result, None)

        # two frameIDs
        chem.frames = ['gradient']
        result = chem.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        self.assertEqual(result, Vector3(0.29, 0.0, -0.29))


if __name__ == "__main__":
    rostest.rosrun("so_data", 'chemotaxis_test_node', ChemotaxisTest)
