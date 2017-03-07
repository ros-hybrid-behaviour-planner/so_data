"""
Created on 21.12.2016

@author: kaiser

Module with unit test for flockingrey.py
"""
import rospy
import unittest
from geometry_msgs.msg import Vector3, Quaternion
from so_data.flockingrey import *
from so_data.gradientnode import create_gradient


class FlockingAITest(unittest.TestCase):
    def testCohesion(self):
        agent = create_gradient(Vector3(1, 2, 3), q=Quaternion(),
                                direction=Vector3(1, 0, 0), goal_radius=1.0)
        neighbors = []
        neighbors.append(create_gradient(Vector3(1, 2, 3),
                                         q=Quaternion(0.707, 0.707, 0, 0),
                                         direction=Vector3(1, 0, 0),
                                         goal_radius=1.0))
        neighbors.append(create_gradient(Vector3(2, 2, 5),
                                         q=Quaternion(0.707, 0.707, 0, 0),
                                         direction=Vector3(1, 0, 0),
                                         goal_radius=1.0))
        neighbors.append(create_gradient(Vector3(1, 6, 3),
                                         q=Quaternion(0.707, 0.707, 0, 0),
                                         direction=Vector3(1, 0, 0),
                                         goal_radius=1.0))

        result = cohesion(agent, neighbors)
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(0.33, 1.33, 0.67))

    def testAlignment(self):
        agent = create_gradient(Vector3(1, 2, 3), q=Quaternion(),
                                direction=Vector3(1, 0, 0), goal_radius=1.0)
        neighbors = []
        neighbors.append(create_gradient(Vector3(1, 2, 3),
                                         q=Quaternion(0.707, 0.707, 0, 0),
                                         direction=Vector3(1, 0, 0),
                                         goal_radius=1.0))

        result = alignment(agent, neighbors)

        self.assertEqual(result, Vector3(-1, 1, 0))

    def testSeparation(self):
        agent = create_gradient(Vector3(1, 2, 3), q=Quaternion(),
                                direction=Vector3(1, 0, 0), goal_radius=1.0)
        neighbors = []
        neighbors.append(create_gradient(Vector3(2, 2, 5),
                                         q=Quaternion(0.707, 0.707, 0, 0),
                                         direction=Vector3(1, 0, 0),
                                         goal_radius=1.0))
        neighbors.append(create_gradient(Vector3(1, 6, 3),
                                         q=Quaternion(0.707, 0.707, 0, 0),
                                         direction=Vector3(1, 0, 0),
                                         goal_radius=1.0))

        result = separation(agent, neighbors)
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)

        self.assertEqual(result, Vector3(-1.89, -0.5, -3.79))

# run tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()
