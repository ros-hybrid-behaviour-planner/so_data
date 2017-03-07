"""
Created on 20.02.2017

@author: kaiser

Unit test for decisions.py
"""
import unittest
import rospy
import numpy as np
from so_data.msg import SoMessage
from geometry_msgs.msg import Vector3, Quaternion
from diagnostic_msgs.msg import KeyValue
from so_data.decisions import MorphogenesisBarycenter, GossipMax, Quorum
from so_data.sobuffer import SoBuffer, AGGREGATION


class DecisionsTest(unittest.TestCase):
    """
    class to test decisions.py
    """

    def test_morphogenesis(self):
        """
        test class morphogenesis barycenter
        """

        bffr = SoBuffer()
        morph = MorphogenesisBarycenter(bffr, 'morphogenesis', 'dist')

        bffr._moving = {
            'morphogenesis': {
                'robot1': [SoMessage(None, None, Vector3(2, 2, 0),
                                     Quaternion(), 1, 1.0, 1.0, 1.0, 0, None,
                                     Vector3(), 0, 0, True,
                                     [KeyValue('dist', '7.0')])],
                'robto2': [SoMessage(None, None, Vector3(1, 2, 0),
                                     Quaternion(), 1, 2.0, 1.0, 1.0, 0, None,
                                     Vector3(), 0, 0, True,
                                     [KeyValue('dist', '4.0')])]
            }
        }

        bffr._own_pos = [SoMessage(None, None, Vector3(1, 1, 0), Quaternion(),
                                   1, 1.0, 1.0, 1.0, 0, None, Vector3(), 0, 0,
                                   False, [KeyValue('dist', '5.0')])]

        self.assertEqual(morph.state, 'None')
        self.assertEqual(morph.calc_value()[0], np.sqrt(2) + 1)
        self.assertEqual(morph.calc_value()[1], 'Center')

    def test_gossip(self):
        """
        test class gossip max
        """

        bffr = SoBuffer()
        gossip = GossipMax(bffr, 'gossip', 'max')

        bffr._moving = {
            'gossip': {
                'robot1': [SoMessage(None, None, Vector3(2, 2, 0),
                                     Quaternion(), 1, 1.0, 1.0, 1.0, 0, None,
                                     Vector3(), 0, 0, True,
                                     [KeyValue('max', '1.0')])],
                'robot2': [SoMessage(None, None, Vector3(1, 2, 0),
                                     Quaternion(), 1, 2.0, 1.0, 1.0, 0, None,
                                     Vector3(), 0, 0, True,
                                     [KeyValue('max', '4.0')])],
                'robot4': [SoMessage(None, None, Vector3(5, 6, 0),
                                     Quaternion(), 1, 2.0, 1.0, 1.0, 0, None,
                                     Vector3(), 0, 0, True,
                                     [KeyValue('max', '8.9')])]
            }
        }

        bffr._own_pos = [SoMessage(None, None, Vector3(1, 1, 0), Quaternion(),
                                   1, 1.0, 1.0, 1.0, 0, None, Vector3(), 0, 0,
                                   False, [KeyValue('max', '3.0')])]

        self.assertEqual(gossip.calc_value()[0], 4.0)

    def test_quorum(self):
        """
        test quorum sensing class
        :return:
        """

        bffr = SoBuffer()

        quorum = Quorum(bffr, 2)

        self.assertEqual(quorum.calc_value()[1], False)

        # 2 neighbors within view, one outside view
        bffr._moving = {
            'robot': {'robot1': [SoMessage(),
                                 SoMessage(None, None, Vector3(2, 2, 0),
                                           Quaternion(), 1, 1.0, 1.0, 1.0, 0,
                                           None, Vector3(), 0, 0, True, [])],
                      'robot2': [SoMessage(),
                                 SoMessage(None, None, Vector3(5, 6, 0),
                                           Quaternion(), 1, 2.0, 1.0, 1.0, 0,
                                           None, Vector3(), 0, 0, True, [])],
                      'robot3': [SoMessage(), SoMessage(),
                                 SoMessage(None, None, Vector3(1, 2, 0),
                                           Quaternion(), 1, 4.0, 1.0, 1.0, 0,
                                           None, Vector3(), 0, 0, True, [])],
                      'robot4': [] } }

        bffr._static = {
            'None': [SoMessage(None, None, Vector3(5, 6, 5), Quaternion(), 1,
                               1.0, 1.0, 1.0, 0, None, Vector3(), 0, 0, False,
                               [])]}

        bffr._own_pos = [SoMessage(None, None, Vector3(1, 1, 1), Quaternion(),
                                   1, 1.0, 1.0, 1.0, 0, None, Vector3(), 0, 0,
                                   False, [])]

        self.assertEqual(quorum.calc_value()[1], True)

        quorum.threshold = 3
        self.assertEqual(quorum.calc_value()[1], False)


# run tests - start roscore before running tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()
