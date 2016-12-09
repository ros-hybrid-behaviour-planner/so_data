'''
Created on 14.11.2016

@author: kaiser
'''

import soBuffer
import unittest
import calc
from so_data.msg import soMessage
import rospy
from geometry_msgs.msg import Point, Vector3
from copy import deepcopy
from std_msgs.msg import Header
import numpy as np


class soBufferTest(unittest.TestCase):

    def test_store_data(self):
        '''
        test store_data method
        '''

        # Test aggregation = max
        bffr = soBuffer.SoBuffer(aggregation='max')
        testlist = []

        msg = soMessage(None, Vector3(2,2,0), 1, 4.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3,3,0), 1, 4.0, 1.0, 0, Vector3(), [])
        testlist.append(msg)
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3,3,0), 1, 3.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5,5,0), 1, 4.0, 1.0, 0, Vector3(), [])
        testlist.append(msg)
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2,2,0), 1, 5.0, 1.0, 0, Vector3(), [])
        testlist.append(msg)
        bffr.store_data(msg)

        self.assertEqual(bffr.get_data(), testlist)

        # Test aggregation = min
        bffr = soBuffer.SoBuffer(aggregation='min')
        testlist = []

        msg = soMessage(None, Vector3(2,2,0), 1, 4.0, 1.0, 0, Vector3(), [])
        testlist.append(msg)
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3,3,0), 1, 4.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3,3,0), 1, 3.0, 1.0, 0, Vector3(), [])
        testlist.append(msg)
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5,5,0), 1, 4.0, 1.0, 0, Vector3(), [])
        testlist.append(msg)
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2,2,0), 1, 5.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        self.assertEqual(bffr.get_data(), testlist)

        # Test aggregation = avg
        bffr = soBuffer.SoBuffer(aggregation='avg')
        testlist = []

        msg = soMessage(None, Vector3(2, 2, 0), 1, 4.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 4.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 3.0, 1.0, 0, Vector3(), [])
        testlist.append(soMessage(None, Vector3(3, 3, 0), 1, 3.5, 1.0, 0, Vector3(), []))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), 1, 4.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 2, 0), -1, 5.0, 1.0, 0, Vector3(), [])
        testlist.append(soMessage(None, Vector3(2, 2, 0), -1, 1.0, 0.0, 0, Vector3(), []))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), -1, 4.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        self.assertEqual(bffr.get_data(), testlist)

        # Test aggregation = avg
        bffr = soBuffer.SoBuffer(aggregation='newest')
        testlist = []

        msg = soMessage(None, Vector3(2, 2, 0), 1, 4.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 4.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 3.0, 1.0, 0, Vector3(), [])
        testlist.append(msg)
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), 1, 4.0, 1.0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 2, 0), -1, 5.0, 1.0, 0, Vector3(), [])
        testlist.append(msg)
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), -1, 4.0, 1.0, 0, Vector3(), [])
        testlist.append(msg)
        bffr.store_data(msg)

        self.assertEqual(bffr.get_data(), testlist)

    def test_calc_attractive_gradient(self):
        """
        test _calc_attractive_gradient method for 2D and 3D
        """
        bffr = soBuffer.SoBuffer()

        # 2D - D < r <= C
        gradient = soMessage(None, Vector3(3, 4, 0), 1, 5.0, 1.0, 0, Vector3(), [])
        pose = Point(0,0,0)
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), Vector3(0.6*0.8, 0.8*0.8, 0))

        # 2D - r > C
        gradient = soMessage(None, Vector3(3, 4, 0), 1, 2.0, 1.0, 0, Vector3(), [])
        pose = Point(0,0,0)
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), Vector3(0.6, 0.8, 0))

        # 2D - r <= D
        gradient = soMessage(None, Vector3(3, 4, 0), 1, 2.0, 5.0, 0, Vector3(), [])
        pose = Point(0,0,0)
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), Vector3(0, 0, 0), [])

        # 2D - r > C - non zero robot pose
        gradient = soMessage(None, Vector3(4, 5, 0), 1, 2.0, 1.0, 0, Vector3(), [])
        pose = Point(1,1,0)
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), Vector3(0.6, 0.8, 0))

        # 3D - D < r <= C
        gradient = soMessage(None, Vector3(3, 5, 10), 1, 6.0, 2.0, 0, Vector3(), [])
        pose = Point(1,2,4)
        result = Vector3((2.0/7.0)*(5.0/6.0), (3.0/7.0)*(5.0/6.0), (6.0/7.0)*(5.0/6.0))
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), result)

        # 3D - r > C
        gradient = soMessage(None, Vector3(3, 5, 10), 1, 5.0, 2.0, 0, Vector3(), [])
        pose = Point(1,2,4)
        result = Vector3((2.0/7.0), (3.0/7.0), (6.0/7.0))
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), result)

        # 3D - r <= D
        gradient = soMessage(None, Vector3(3, 5, 10), 1, 5.0, 7.0, 0, Vector3(), [])
        pose = Point(1, 2, 4)
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), Vector3(0, 0, 0))

    def test_calc_repulsive_gradient(self):
        """
        test _calc_repulsive_gradient method
        """
        bffr = soBuffer.SoBuffer()

        # 2D - D < r <= C
        gradient = soMessage(None, Vector3(3, 4, 0), -1, 5.0, 1.0, 0, Vector3(), [])
        pose = Point(0,0,0)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(-0.6*0.2, -0.8*0.2, 0))

        # 2D - r > C
        gradient = soMessage(None, Vector3(3, 4, 0), -1, 2.0, 1.0, 0, Vector3(), [])
        pose = Point(0,0,0)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(0, 0, 0))

        # 2D - r <= D
        gradient = soMessage(None, Vector3(3, 4, 0), -1, 2.0, 5.0, 0, Vector3(), [])
        pose = Point(0,0,0)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(-1 * np.inf, -1 * np.inf, -1*np.inf))

        # 2D - r > C - non zero robot pose
        gradient = soMessage(None, Vector3(4, 5, 0), -1, 2.0, 1.0, 0, Vector3(), [])
        pose = Point(1,1,0)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(0, 0, 0))

        # 3D - D < r <= C
        gradient = soMessage(None, Vector3(3, 5, 10), -1, 6.0, 2.0, 0, Vector3(), [])
        pose = Point(1,2,4)
        result = Vector3((-2.0/7.0)*(1.0/6.0), (-3.0/7.0)*(1.0/6.0), (-6.0/7.0)*(1.0/6.0))
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), result)

        # 3D - r > C
        gradient = soMessage(None, Vector3(3, 5, 10), -1, 5.0, 2.0, 0, Vector3(), [])
        pose = Point(1,2,4)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(0, 0, 0))

        # 3D - r <= D
        gradient = soMessage(None, Vector3(3, 5, 10), -1, 5.0, 7.0, 0, Vector3(), [])
        pose = Point(1, 2, 4)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(-1 * np.inf, -1 * np.inf, -1*np.inf))


    # def test_get_current_gradient(self):
    #     '''
    #     test gradient update of current gradient
    #     :return:
    #     '''
    #     bffr = soBuffer.SoBuffer(result='max', aggregation = 'max', evaporation_factor=1.0, collision_avoidance='')
    #
    #     # distance > diffusion radius
    #     gradient = soMessage(None, Vector3(5,5,0), 1, 3.0, 1.0, 0, Vector3())
    #     bffr.store_data(gradient)
    #     self.assertEqual(bffr.get_current_gradient(Point(0,0,0)), Vector3(0,0,0))
    #
    #     # distance < diffusion radius
    #     gradient = soMessage(None, Vector3(2,2,0), 1, 3.0, 1.0, 0, Vector3())
    #     bffr.store_data(gradient)
    #     self.assertEqual(bffr.get_current_gradient(Point(0,0,0)), Vector3(2/3.0,2/3.0,0))

    def test_get_gradient_distance(self):
         '''
         test calculation of euclidian distance current position - gradient position
         :return:
         '''
         self.assertEqual(calc.get_gradient_distance(Vector3(3,4,0), Point(0,0,0)), 5.0)

    # def test_aggregate_min(self):
    #     '''
    #     test of aggregation of data (closest gradient = current gradient)
    #     :return:
    #     '''
    #     bffr = soBuffer.SoBuffer(aggregation = True)
    #
    #     bffr.store_data(soMessage(Vector(2,2,0), rospy.Time.now(), 1, 4.0, 0, Vector()))
    #     bffr.store_data(soMessage(Vector(3,3,0), rospy.Time.now(), 1, 3.0, 0, Vector()))
    #     gradient = soMessage(Vector(1,1,0), rospy.Time.now(), 1, 5.0, 0, Vector())
    #     bffr.store_data(deepcopy(gradient))
    #
    #     self.assertEqual(bffr.get_current_gradient(Pose(0,0,0,0,0)) , Vector(1/5.0, 1/5.0, 0))
    #
    def test_evaporation_buffer(self):
        '''
        test evaporation of buffer data
        :return:
        '''

        bffr = soBuffer.SoBuffer(aggregation = 'max', evaporation_factor=0.8, evaporation_time=5, min_diffusion=1.0)
        now = rospy.Time.now()

        data = [soMessage(Header(None, now - rospy.Duration(45), None), Vector3(1,1,0), 1, 4.0, 1.0, 0, Vector3(), []),
                 soMessage(Header(None, now - rospy.Duration(20), None), Vector3(2,2,0), 1, 4.0, 1.0, 0, Vector3(), []),
                 soMessage(Header(None, now - rospy.Duration(15), None), Vector3(3,3,0), 1, 4.0, 1.0, 0, Vector3(), []),
                 soMessage(Header(None, now - rospy.Duration(10), None), Vector3(4,4,0), 1, 4.0, 1.0, 0, Vector3(), []),
                 soMessage(Header(None, now - rospy.Duration(5), None), Vector3(5,5,0), 1, 4.0, 1.0, 0, Vector3(), []),
                 soMessage(Header(None, now, None), Vector3(6,6,0), 1, 4.0, 1.0, 0, Vector3(), [])
                 ]

        for d in data:
            bffr.store_data(deepcopy(d))

        bffr._evaporate_buffer()

        data = [ soMessage(Header(None, now, None), Vector3(2,2,0), 1, 4.0 * (0.8 ** 4), 1.0 * (0.8 ** 4), 0, Vector3(), []),
                 soMessage(Header(None, now, None), Vector3(3,3,0), 1, 4.0 * (0.8 ** 3), 1.0 * (0.8 ** 3), 0, Vector3(), []),
                 soMessage(Header(None, now, None), Vector3(4,4,0), 1, 4.0 * (0.8 ** 2), 1.0 * (0.8 ** 2), 0, Vector3(), []),
                 soMessage(Header(None, now, None), Vector3(5,5,0), 1, 4.0 * 0.8, 1.0 * 0.8, 0, Vector3(), []),
                 soMessage(Header(None, now, None), Vector3(6,6,0), 1, 4.0, 1.0, 0, Vector3(), [])]

        self.assertEqual(bffr.get_data(), data)

    def test_repulsion_ge(self):
        """
        test repulsion vector calculation based on Ge
        :return:
        """
        bffr = soBuffer.SoBuffer()
        gradient = soMessage(None, Vector3(4, 2, 0), 1, 3.0, 1.0, 0, Vector3(), [])
        goal = soMessage(None, Vector3(2, 2, 0), 1, 3.0, 1.0, 0, Vector3(), [])
        pose = Point(3, 3, 0)
        vector = bffr._calc_repulsive_gradient_ge(gradient, goal, pose)

    #
    # def test_get_current_gradient_full(self):
    #     '''
    #     test determination of current gradient to follow
    #     :return:
    #     '''
    #     bffr = soBuffer.SoBuffer(aggregation = True, evaporation_factor=0.8, evaporation_time=5, min_diffusion=1.0)
    #
    #     now = rospy.Time.now()
    #     data = [soMessage(Vector(2,2,0), now - rospy.Duration(20), 1, 2.0, 0, Vector()),
    #         soMessage(Vector(3,3,0), now - rospy.Duration(15), 1, 3.0, 0, Vector()),
    #         soMessage(Vector(4,4,0), now - rospy.Duration(10), 1, 5.0, 0, Vector()),
    #         soMessage(Vector(5,5,0), now - rospy.Duration(5), 1, 4.0, 0, Vector()),
    #         soMessage(Vector(6,6,0), now, 1, 4.0, 0, Vector())
    #         ]
    #
    #     for d in data:
    #         bffr.store_data(deepcopy(d))
    #
    #     self.assertEqual(bffr.get_current_gradient(Pose(0,0,0,0,0)), Vector(3,3,0))
    #
    #     for element in bffr.data:
    #         element.stamp -= rospy.Duration(10)
    #
    #     # TODO: change
    #     self.assertEqual(bffr.get_current_gradient(Pose(0,0,0,0,0)), Vector(0,0,0))
    #
    # #def test_aggregate_several(self):
    # #    bffr = soBuffer.SoBuffer(aggregation=False, evaporation_factor=1.0, evaporation_time=5, min_diffusion=1.0)
    #
    # #    now = rospy.Time.now()
    #
    #     #data = [soMessage(Vector(1, 1, 0), now, -1, 2.0, 0, Vector()),
    #     #            soMessage(Vector(0, 3, 0), now, 1, 4.0, 0, Vector()),
    #     #        ]
    #
    # #    data = [soMessage(Vector(1, 1, 0), now, -1, 2.0, 0, Vector()),
    # #             soMessage(Vector(0, 3, 0), now, 1, 4.0, 0, Vector()),
    # #             soMessage(Vector(-1, 2, 0), now, -1, 3.0, 0, Vector()),
    # #            ]
    #
    #
    # #    for d in data:
    # #            bffr.store_data(deepcopy(d))
    #
    # #    bffr.aggregate_several(Pose(0,0,0,0,0))



# run tests - start roscore before running tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()

