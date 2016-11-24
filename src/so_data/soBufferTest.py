'''
Created on 14.11.2016

@author: kaiser
'''

import soBuffer
import unittest
from so_data.msg import Vector, soMessage, Pose
import rospy
from copy import deepcopy


class soBufferTest(unittest.TestCase):

    def test_store_data(self):
        '''
        test store_data method
        '''
        bffr = soBuffer.SoBuffer(aggregation=False)
        testlist = []

        msg = soMessage(Vector(2,2,0), rospy.Time.now(), 1, 4.0, 0, Vector())
        bffr.store_data(msg)

        msg = soMessage(Vector(3,3,0), rospy.Time.now(), 1, 4.0, 0, Vector())
        testlist.append(msg)
        bffr.store_data(msg)

        msg = soMessage(Vector(3,3,0), rospy.Time.now(), 1, 3.0, 0, Vector())
        bffr.store_data(msg)

        msg = soMessage(Vector(5,5,0), rospy.Time.now(), 1, 4.0, 0, Vector())
        testlist.append(msg)
        bffr.store_data(msg)

        msg = soMessage(Vector(2,2,0), rospy.Time.now(), 1, 5.0, 0, Vector())
        testlist.append(msg)
        bffr.store_data(msg)

        self.assertEqual(bffr.get_data(), testlist)

    def test_get_current_gradient(self):
        '''
        test gradient update of current gradient
        :return:
        '''
        bffr = soBuffer.SoBuffer(aggregation = False, evaporation_factor=1.0)

        # distance > diffusion radius
        gradient = soMessage(Vector(3,3,0), rospy.Time.now(), 1, 3.0, 0, Vector())
        bffr._current_gradient = gradient
        self.assertEqual(bffr.get_current_gradient(Pose(0,0,0,0,0)), soMessage())

        # distance < diffusion radius
        gradient = soMessage(Vector(2,2,0), rospy.Time.now(), 1, 3.0, 0, Vector())
        bffr._current_gradient = gradient
        self.assertEqual(bffr.get_current_gradient(Pose(0,0,0,0,0)), gradient)

    def test_get_gradient_distance(self):
        '''
        test calculation of euclidian distance current position - gradient position
        :return:
        '''
        bffr = soBuffer.SoBuffer(aggregation = False)
        self.assertEqual(bffr.get_gradient_distance(Vector(3,4,0), Pose(0,0,0,0,0)), 5.0)

    def test_aggregate_min(self):
        '''
        test of aggregation of data (closest gradient = current gradient)
        :return:
        '''
        bffr = soBuffer.SoBuffer(aggregation = True)

        bffr.store_data(soMessage(Vector(2,2,0), rospy.Time.now(), 1, 4.0, 0, Vector()))
        bffr.store_data(soMessage(Vector(3,3,0), rospy.Time.now(), 1, 3.0, 0, Vector()))
        gradient = soMessage(Vector(1,1,0), rospy.Time.now(), 1, 5.0, 0, Vector())
        bffr.store_data(deepcopy(gradient))

        self.assertEqual(bffr.get_current_gradient(Pose(0,0,0,0,0)) , gradient)

    def test_evaporation_gradient(self):
        '''
        test evaporation of current gradient
        :return:
        '''
        bffr = soBuffer.SoBuffer(aggregation = False, evaporation_factor=0.8, evaporation_time=5, min_diffusion=1.0)

        gradient = soMessage(Vector(1,1,0), rospy.Time.now() - rospy.Duration(10), 1, 5.0, 0, Vector())
        bffr._current_gradient = deepcopy(gradient)
        bffr.evaporate_gradient(Pose(0,0,0,0,0))

        # evaporation effects
        gradient.diffusion *= 0.8 * 0.8
        gradient.stamp += rospy.Duration(10)

        self.assertEqual(bffr.get_current_gradient(Pose(0,0,0,0,0)), gradient)

        gradient.stamp -= rospy.Duration(30)
        bffr._current_gradient = deepcopy(gradient)
        bffr.evaporate_gradient(Pose(0,0,0,0,0))

        self.assertEqual(bffr.get_current_gradient(Pose(0,0,0,0,0)), soMessage())

    def test_evaporation_buffer(self):
        '''
        test evaporation of buffer data
        :return:
        '''

        bffr = soBuffer.SoBuffer(aggregation = False, evaporation_factor=0.8, evaporation_time=5, min_diffusion=1.0)
        now = rospy.Time.now()

        data = [soMessage(Vector(1,1,0), now - rospy.Duration(50), 1, 4.0, 0, Vector()),
                soMessage(Vector(2,2,0), now - rospy.Duration(20), 1, 4.0, 0, Vector()),
                soMessage(Vector(3,3,0), now - rospy.Duration(15), 1, 4.0, 0, Vector()),
                soMessage(Vector(4,4,0), now - rospy.Duration(10), 1, 4.0, 0, Vector()),
                soMessage(Vector(5,5,0), now - rospy.Duration(5), 1, 4.0, 0, Vector()),
                soMessage(Vector(6,6,0), now, 1, 4.0, 0, Vector())
                ]

        for d in data:
            bffr.store_data(deepcopy(d))

        bffr.evaporate_buffer()

        data = [soMessage(Vector(2,2,0), now, 1, 4.0 * (0.8 ** 4), 0, Vector()),
                soMessage(Vector(3,3,0), now, 1, 4.0 * (0.8 ** 3), 0, Vector()),
                soMessage(Vector(4,4,0), now, 1, 4.0 * (0.8 ** 2), 0, Vector()),
                soMessage(Vector(5,5,0), now, 1, 4.0 * 0.8, 0, Vector()),
                soMessage(Vector(6,6,0), now, 1, 4.0, 0, Vector())]

        self.assertEqual(bffr.get_data(), data)

    def test_get_current_gradient_full(self):
        '''
        test determination of current gradient to follow
        :return:
        '''
        bffr = soBuffer.SoBuffer(aggregation = True, evaporation_factor=0.8, evaporation_time=5, min_diffusion=1.0)

        now = rospy.Time.now()
        data = [soMessage(Vector(2,2,0), now - rospy.Duration(20), 1, 2.0, 0, Vector()),
            soMessage(Vector(3,3,0), now - rospy.Duration(15), 1, 3.0, 0, Vector()),
            soMessage(Vector(4,4,0), now - rospy.Duration(10), 1, 5.0, 0, Vector()),
            soMessage(Vector(5,5,0), now - rospy.Duration(5), 1, 4.0, 0, Vector()),
            soMessage(Vector(6,6,0), now, 1, 4.0, 0, Vector())
            ]

        for d in data:
            bffr.store_data(deepcopy(d))

        self.assertEqual(bffr.get_current_gradient(Pose(0,0,0,0,0)), soMessage(Vector(3,3,0), now, 1, 3.0 * (0.8 ** 3), 0, Vector()))

        for element in bffr.data:
            element.stamp -= rospy.Duration(10)

        self.assertEqual(bffr.get_current_gradient(Pose(0,0,0,0,0)), soMessage(Vector(4,4,0), now, 1, 5.0 * (0.8 ** 2) * (0.8 **2), 0, Vector()))


# run tests - start roscore before running tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()

