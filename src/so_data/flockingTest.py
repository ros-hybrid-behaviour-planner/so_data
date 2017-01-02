"""
Created on 21.12.2016

@author: kaiser
"""

import unittest
from so_data.msg import soMessage
from geometry_msgs.msg import Vector3
import numpy as np
from std_msgs.msg import Header
import flocking
import rospy
import collections


class FlockingTest(unittest.TestCase):
    """
    unit test flocking.py
    """

    def test_bump_function(self):
        """
        unit test of bump function
        """
        h = 0.6

        # z within [0, h)
        z = 0.5
        self.assertEqual(flocking.bump_function(z, h), 1.0)

        # z within [h, 1]
        z = 0.6
        self.assertEqual(flocking.bump_function(z, h), 1.0)

        z = 0.8
        self.assertEqual(flocking.bump_function(z, h), 0.5*(1+np.cos(np.pi * (z-h)/(1-h))))  # 0.5

        z = 1.0
        self.assertEqual(flocking.bump_function(z, h), 0.0)

        # z otherwise
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

    def test_adjacency_matrix(self):
        """
        unit test for adjacency matrix function
        """
        # result larger than 1 --> bump function returns 0
        qj = Vector3(2.0,3.0,1.0)
        qi = Vector3(3.0,1.0,0.0)
        epsilon_z = 1.0
        epsilon_r = 2.0
        r = 1.0
        h = 0.5
        self.assertEqual(flocking.adjacency_matrix(qj, qi, epsilon_z, epsilon_r, r, h), 0)

        # result in [h, 1]
        r = 4.0
        self.assertEqual(round(flocking.adjacency_matrix(qj, qi, epsilon_z, epsilon_r, r, h), 3), 0.673)

        # result in [0,h)
        r = 7.0
        self.assertEqual(flocking.adjacency_matrix(qj, qi, epsilon_z, epsilon_r, r, h), 1.0)

    def test_agent_velocity(self):
        """
        unit test for agent velocity function
        """
        # no movement at all
        p1 = soMessage()
        p2 = soMessage()
        self.assertEqual(flocking.agent_velocity(p1, p2), Vector3())

        p1 = soMessage(Header(None, rospy.Duration(0), ''), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0, Vector3(), [])
        p2 = soMessage(Header(None, rospy.Duration(5), ''), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0, Vector3(), [])
        self.assertEqual(flocking.agent_velocity(p1, p2), Vector3(0, 0, 0))

        # 5 seconds passed
        p1 = soMessage(Header(None, rospy.Duration(0), ''), Vector3(0,0,0), 1, 4.0, 1.0, 0.8, 5, 0, Vector3(), [])
        p2 = soMessage(Header(None, rospy.Duration(5), ''), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0, Vector3(), [])
        self.assertEqual(flocking.agent_velocity(p1, p2), Vector3(0.4, 0.4, 0))

        # nanoseconds passed
        p1 = soMessage(Header(None, rospy.Duration(0, 0), ''), Vector3(0,0,0), 1, 4.0, 1.0, 0.8, 5, 0, Vector3(), [])
        p2 = soMessage(Header(None, rospy.Duration(0, 100000), ''), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0, Vector3(),
                       [])
        self.assertEqual(flocking.agent_velocity(p1, p2), Vector3(20000.0, 20000.0, 0))

        # negative velocity
        p1 = soMessage(Header(None, rospy.Duration(0, 0), ''), Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, 0, Vector3(), [])
        p2 = soMessage(Header(None, rospy.Duration(0, 100000), ''), Vector3(0, 0, 0), 1, 4.0, 1.0, 0.8, 5, 0, Vector3(),
                       [])
        self.assertEqual(flocking.agent_velocity(p1, p2), Vector3(-20000.0, -20000.0, 0))

    def test_vector_btw_agents(self):
        """
        unit test of vector between agents function
        """
        qj = Vector3(2, 1, 0)
        qi = Vector3(3, 4, 2)
        epsilon = 0.5
        self.assertEqual(flocking.vector_btw_agents(qi, qj, epsilon),
                         Vector3(-1.0/np.sqrt(8), -3.0/np.sqrt(8), -2.0/np.sqrt(8)))

    def test_action_function(self):
        """
        unit test for action function
        """
        qj = Vector3(1,2,3)
        qi = Vector3(0,4,2)

        epsilon = 0.5
        r = 1.0
        a = 1.0
        b = 1.0
        avoidance_distance = 2.0
        h = 0.5

        # bump function returns 0
        self.assertEqual(flocking.action_function(qj, qi, r, epsilon, a, b, avoidance_distance, h), 0.0)

        # bump function returns 1
        r = 4.0
        self.assertEqual(round(flocking.action_function(qj, qi, r, epsilon, a, b, avoidance_distance, h),3), 0.472)

        # bump function returns value not being 0 or 1
        r = 3.0
        self.assertEqual(round(flocking.action_function(qj, qi, r, epsilon, a, b, avoidance_distance, h),3), 0.246)

    def test_velocity_consensus(self):
        """
        unit test for velocity consensus function
        """
        Boid = collections.namedtuple('Boid', ['p', 'v'])

        neighbors = [Boid(Vector3(0,0,0), Vector3(2.0, 1.0, 0.0))]
        agent = Boid(Vector3(1,1,0), Vector3(0.0,0.0,0.0))
        epsilon_z = 0.5
        epsilon_r = 0.5
        r = 2.0
        h = 0.5
        result = flocking.velocity_consensus(neighbors, agent, epsilon_z, epsilon_r, r, h)
        result.x = round(result.x, 3)
        result.y = round(result.y, 3)
        result.z = round(result.z, 3)
        self.assertEqual(result, Vector3(1.916, 0.958, 0.0))

    def test_gradient_based(self):
        """
        unit test for gradient based term
        """
        Boid = collections.namedtuple('Boid', ['p', 'v'])

        neighbors = [Boid(Vector3(0, 4, 2), Vector3(2.0, 1.0, 0.0))]
        agent = Boid(Vector3(1, 2, 3), Vector3(0.0, 0.0, 0.0))

        epsilon = 0.5
        view_distance = 4.0
        a = 1.0
        b = 1.0
        avoidance_distance = 2.0
        h = 0.5

        result = flocking.gradient_based(neighbors, agent, epsilon, a, b, avoidance_distance, view_distance, h)

        result.x = round(result.x, 3)
        result.y = round(result.y, 3)
        result.z = round(result.z, 3)

        self.assertEqual(result, Vector3(-0.236, 0.472, -0.236))



# run tests
if __name__ == "__main__":
    unittest.main()