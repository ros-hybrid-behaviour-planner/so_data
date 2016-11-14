'''
Created on 14.11.2016

@author: kaiser
'''

import soBuffer
from collections import deque
import unittest
from so_data.msg import soMessage
from turtlesim.msg import Pose
import rospy


class soBufferTest(unittest.TestCase):
    def test_prune_buffer(self):
        buffer = soBuffer.SoBuffer(2.0)
        self._msg = soMessage()
        self._msg.p = Pose(2, 2, 0, 0, 0)
        self._msg.info = 1.0
        self._msg.stamp = rospy.Time.now() - rospy.Duration(2)
        buffer.store_data(self._msg)
        self._msg = soMessage()
        self._msg.stamp = rospy.Time.now()
        buffer.store_data(self._msg)
        testlist = deque([])
        testlist.append(self._msg)
        self.assertEqual(buffer.get_data(), testlist)



    def test_store_data(self):
        buffer = soBuffer.SoBuffer(2.0)
        self._msg = soMessage()
        self._msg.p = Pose(2,2,0,0,0)
        self._msg.info = 1.0
        self._msg.stamp = rospy.Time.now() + rospy.Duration(1)
        testlist = deque([])
        testlist.append(self._msg)
        buffer.store_data(self._msg)
        self._msg = soMessage()
        self._msg.stamp = rospy.Time.now()
        buffer.store_data(self._msg)
        self.assertEqual(buffer.get_data(), testlist)

# run tests - start roscore before running tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()


