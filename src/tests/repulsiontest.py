"""
Created on 20.02.2017

@author: kaiser

Module with unit test for repulsion.py
"""

import unittest
import rospy
from so_data.msg import SoMessage
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import so_data.calc
from so_data.repulsion import RepulsionFernandez, RepulsionGradient
from so_data.sobuffer import SoBuffer


class RepulsionTest(unittest.TestCase):
    """
    class to test repulsion py module / calculations
    """
    def test_gradient_repulsion(self):
        """
        test gradient repulsion method based on gradients
        :return:
        """

        bffr = SoBuffer(id='robot1')
        repulsion = RepulsionGradient(bffr)

        # no own position specified
        self.assertEqual(repulsion.move(), Vector3())

        bffr._own_pos = [
            SoMessage(Header(None, rospy.Time.now(), 'None'), None,
                      Vector3(2, 4, 0), Quaternion(), Vector3(), 1, 4.0, 0.0,
                      1.0, 0, None, False, []),
            SoMessage(Header(None, rospy.Time.now(), 'None'), None,
                      Vector3(2, 2, 0), Quaternion(), Vector3(), 1, 2.0, 0.0,
                      1.0, 0, None, False, [])
        ]

        # no neighbors specified
        self.assertEqual(repulsion.move(), Vector3())

        bffr._moving = {
            'robot': {'robot2': [
                SoMessage(Header(None, rospy.Time.now(), 'robot'), 'robot2',
                          Vector3(1, 3, 0), Quaternion(), Vector3(), -1, 1.0,
                          1.0, 1.0, 0, None, False, [])
            ],
                'robot3': [
                    SoMessage(Header(None, rospy.Time.now(), 'robot'),
                              'robot3', Vector3(2, 2, 0), Quaternion(),
                              Vector3(), -1, 4.0, 1.0, 1.0, 0, None, False,
                              []),
                    SoMessage(Header(None, rospy.Time.now(), 'robot'),
                              'robot3', Vector3(3, 2, 0), Quaternion(),
                              Vector3(), -1, 1.0, 0.8, 1.0, 0, None, False, [])
                ]}
        }
        # calculate resulting vector
        result = repulsion.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        # calculate vector
        self.assertEqual(result, Vector3(-0.39, -0.41, 0.0))

        # neighbor within goal_radius - returns vector with
        # ||vector|| = repulsion_radius
        bffr._moving = {
            'robot': {'robot2': [
                SoMessage(Header(None, rospy.Time.now(), 'robot'), 'robot2',
                          Vector3(2, 1.5, 0), Quaternion(), Vector3(), -1, 2.0,
                          1.0, 1.0, 0, None, False, [])]}}

        d = round(so_data.calc.vector_length(repulsion.move()), 0)

        # calculate vector
        self.assertEqual(d, 2.0)

    def test_repulsion_fernandez(self):
        """
        test gradient repulsion method based on Fernandez-Marquez et al.
        :return:
        """

        # Mechanism
        bffr = SoBuffer(id='robot1', view_distance=2.0)
        repulsion = RepulsionFernandez(bffr)

        # no own position specified
        self.assertEqual(repulsion.move(), Vector3())

        bffr._own_pos = [
            SoMessage(Header(None, rospy.Time.now(), 'None'), None,
                      Vector3(2, 4, 0), Quaternion(), Vector3(), -1, 4.0, 1.0,
                      1.0, 0, rospy.Time.now(), False, []),
            SoMessage(Header(None, rospy.Time.now(), 'None'), None,
                      Vector3(2, 2, 0), Quaternion(), Vector3(), -1, 1.0, 0.0,
                      1.0, 0, rospy.Time.now(), False, [])
        ]

        # no neighbors specified
        self.assertEqual(repulsion.move(), Vector3())

        bffr._moving = {
            'robot':
                {'robot2': [
                    SoMessage(Header(None, rospy.Time.now(), 'None'), 'None',
                              Vector3(1, 3, 0), Quaternion(), Vector3(), -1,
                              1.0, 1.0, 1.0, 0, rospy.Time.now(), False, [])],
                    'robot3':
                        [SoMessage(Header(None, rospy.Time.now(), 'None'),
                                   'None', Vector3(2, 2, 0), Quaternion(),
                                   Vector3(), -1, 4.0, 1.0, 1.0, 0,
                                   rospy.Time.now(), False, []),
                         SoMessage(Header(None, rospy.Time.now(), 'None'),
                                   'None', Vector3(3, 2, 0), Quaternion(),
                                   Vector3(), -1, 1.0, 0.8, 1.0, 0,
                                   rospy.Time.now(), False, [])],
                    'robot4': []
                }
        }

        # calculate resulting vector
        result = repulsion.move()
        result.x = round(result.x, 2)
        result.y = round(result.y, 2)
        result.z = round(result.z, 2)
        # calculate vector
        self.assertEqual(result, Vector3(-0.88, -0.48, 0))

        # neighbor within goal_radius - returns vector with
        # ||vector|| = repulsion_radius
        bffr._moving = {
            'robot': {'robot2': [
                SoMessage(Header(None, rospy.Time.now(), 'None'), None,
                          Vector3(2, 2, 0), Quaternion(), Vector3(), -1, 2.0,
                          1.0, 1.0, 0, rospy.Time.now(), False, [])]}}

        d = round(so_data.calc.vector_length(repulsion.move()), 0)

        # calculate vector
        self.assertEqual(d, 1.0)


# run tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()
