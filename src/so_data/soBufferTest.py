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
    maxDiff = None

    # STORE DATA IN SOBUFFER
    def test_store_data_max(self):
        """
        test store_data method aggregation option = max
        """
        bffr = soBuffer.SoBuffer(aggregation='max')
        testlist = []

        msg = soMessage(None, Vector3(2,2,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3,3,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3,3,0), 1, 3.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5,5,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2,2,0), 1, 5.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'

        self.assertEqual(bffr.get_data(), {'None': testlist})

    def test_store_data_min(self):
        """
        test store_data method aggregation option = min
        """
        bffr = soBuffer.SoBuffer(aggregation='min')
        testlist = []

        msg = soMessage(None, Vector3(2,2,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3,3,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3,3,0), 1, 3.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5,5,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2,2,0), 1, 5.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'

        self.assertEqual(bffr.get_data(), {'None' : testlist})

    def test_store_data_avg(self):
        """
        test store_data method aggregation option = avg
        """
        bffr = soBuffer.SoBuffer(aggregation='avg', min_diffusion=1.0)
        testlist = []

        msg = soMessage(None, Vector3(2, 2, 0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 3.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(soMessage(None, Vector3(3, 3, 0), 1, 3.5, 1.0, 1.0, 0, 0, Vector3(), []))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 2, 0), -1, 5.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(soMessage(None, Vector3(2, 2, 0), -1, 1.0, 0.0, 1.0, 0, 0, Vector3(), []))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), -1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'

        self.assertEqual(bffr.get_data(), {'None' : testlist})

    def test_store_data_newest(self):
        """
        test store_data method aggregation option = newest
        """
        bffr = soBuffer.SoBuffer(aggregation='newest')
        testlist = []

        msg = soMessage(None, Vector3(2, 2, 0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3, 3, 0), 1, 3.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2, 2, 0), -1, 5.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(5, 5, 0), -1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'

        self.assertEqual(bffr.get_data(), {'None' : testlist})

    def test_store_data_fids(self):
        """
        test store_data method with different frame IDs (all)
        """
        bffr = soBuffer.SoBuffer(aggregation='max')
        testlist = {'None': [], 'grad': []}
        now = rospy.Time.now()

        msg = soMessage(Header(None, now, 'None'), Vector3(2,2,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'grad'), Vector3(3,3,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist['grad'].append(msg)
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'None'), Vector3(3,3,0), 1, 3.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'grad'), Vector3(5,5,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'None'), Vector3(2,2,0), 1, 5.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg)

        self.assertEqual(bffr.get_data(), testlist)


    def test_store_data_fids_selection(self):
        """
        test store_data method, only store gradients with specified frameids
        """
        bffr = soBuffer.SoBuffer(aggregation='max', framestorage=['grad', 'silk'])
        testlist = {'grad': [], 'silk': []}
        now = rospy.Time.now()

        msg = soMessage(None, Vector3(2,2,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'grad'), Vector3(3,3,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(3,3,0), 1, 3.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'grad'), Vector3(5,5,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(Header(None, now, 'silk'), Vector3(5,5,0), 1, 4.0, 1.0, 1.0, 0, 0, Vector3(), [])
        testlist['silk'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = soMessage(None, Vector3(2,2,0), 1, 5.0, 1.0, 1.0, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        self.assertEqual(bffr.get_data(), testlist)

    def test_store_data_ev(self):
        """
        test store_data method, evaporation of received data
        """
        bffr = soBuffer.SoBuffer(aggregation='max')
        testlist = {'None': []}
        now = rospy.Time.now()

        # not to be stored - ev time is zero, no goal radius
        msg = soMessage(None, Vector3(2,2,0), 1, 4.0, 0.0, 0.3, 0, 0, Vector3(), [])
        bffr.store_data(msg)

        # received data is older than stored one, will be evaporated and than compared
        msg = soMessage(Header(None, now - rospy.Duration(10), None), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        result = soMessage(Header(None, now, 'None'), Vector3(2,2,0), 1, 4.0 * (0.8 ** 2), 1.0, 0.8, 5, 0,
                        Vector3(), [])
        testlist['None'].append(result)
        bffr.store_data(msg)

        msg = soMessage(Header(None, now - rospy.Duration(15), None), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        bffr.store_data(msg)

        self.assertEqual(bffr.get_data(), testlist)


    def test_store_data_neighbors(self):
        """
        test store_data method, store neighbor data and own pos
        """
        bffr = soBuffer.SoBuffer(aggregation='max', store_neighbors=True, id='robot3')
        testlist = {'robot1': [], 'robot2': []}
        ownpos = []
        now = rospy.Time.now()

        # replaced by fourth robot1 message
        msg = soMessage(Header(None, now - rospy.Duration(10), 'robot1'), Vector3(2.1,2.2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        bffr.store_data(msg)

        # position older than newest stored position --> ignore
        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot1'), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        bffr.store_data(msg)

        # add to position list
        msg = soMessage(Header(None, now - rospy.Duration(5), 'robot1'), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        testlist['robot1'].append(msg)
        bffr.store_data(msg)

        # add to position list
        msg = soMessage(Header(None, now, 'robot1'), Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        testlist['robot1'].append(msg)
        bffr.store_data(msg)

        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot2'), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        testlist['robot2'].append(msg)
        bffr.store_data(msg)

        # own position
        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot3'), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        ownpos.append(msg)
        bffr.store_data(msg)

        self.assertEqual(bffr._neighbors, testlist)
        self.assertEqual(bffr._own_pos, ownpos)

    def test_store_data_neighbors_False(self):
        """
        test store_data method, neighbor gradients will not be stored
        :return:
        """
        bffr = soBuffer.SoBuffer(aggregation='max', store_neighbors=False, id='')
        now = rospy.Time.now()

        # replaced by fourth robot1 message
        msg = soMessage(Header(None, now - rospy.Duration(10), 'robot1'), Vector3(2.1,2.2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        bffr.store_data(msg)

        # position older than newest stored position --> ignore
        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot1'), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        bffr.store_data(msg)

        # add to position list
        msg = soMessage(Header(None, now - rospy.Duration(5), 'robot1'), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        bffr.store_data(msg)

        # add to position list
        msg = soMessage(Header(None, now, 'robot1'), Vector3(2, 2, 0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        bffr.store_data(msg)

        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot2'), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        bffr.store_data(msg)

        # own position
        msg = soMessage(Header(None, now - rospy.Duration(15), 'robot3'), Vector3(2,2,0), 1, 4.0, 1.0, 0.8, 5, 0,
                        Vector3(), [])
        bffr.store_data(msg)

        self.assertEqual(bffr._neighbors, {})
        self.assertEqual(bffr._own_pos, [])


    # EVAPORATION
    def test_evaporation_buffer(self):
        """
        test evaporation of buffer data using evaporate_buffer method
        """

        bffr = soBuffer.SoBuffer(aggregation = 'max', min_diffusion=1.0)
        now = rospy.Time.now()

        data = {'None': [  # message has goal radius - should be kept
                soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1,1,0), 1, 4.0, 1.0, 0.8, 5, 0,
                          Vector3(), []),
                # evaporation time is zero cases
                soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1,1,0), 1, 4.0, 1.0, 0.8, 0, 0,
                      Vector3(), []),
                soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 4.0, 0.0, 1.0, 0, 0,
                      Vector3(), []),
                soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 4.0, 1.0, 1.0, 0, 0,
                      Vector3(), []),
                # messages without goal radius - will be sorted out based on min diffusion
                soMessage(Header(None, now - rospy.Duration(20), 'None'), Vector3(2,2,0), 1, 4.0, 0.0, 0.75, 5, 0,
                          Vector3(), []),
                soMessage(Header(None, now - rospy.Duration(5), 'None'), Vector3(5,5,0), 1, 4.0, 0.0, 0.8, 3, 0,
                          Vector3(), []),
                soMessage(Header(None, now, 'None'), Vector3(6,6,0), 1, 4.0, 0.0, 0.8, 5, 0,
                          Vector3(), [])
                 ], 'gradient': [
                soMessage(Header(None, now - rospy.Duration(45), 'gradient'), Vector3(1,1,0), 1, 4.0, 0.0, 0.8, 5, 0,
                          Vector3(), []),
                soMessage(Header(None, now - rospy.Duration(15), 'gradient'), Vector3(3, 3, 0), 1, 4.0, 0.0, 0.6, 5, 0,
                      Vector3(), [])
                ], 'robo': [
                soMessage(Header(None, now - rospy.Duration(10), 'robo'), Vector3(4, 4, 0), 1, 4.0, 0.0, 0.8, 4, 0,
                      Vector3(), [])]}

        bffr._data = deepcopy(data)
        bffr._evaporate_buffer()

        data = {'None': [
                soMessage(Header(None, now, 'None'), Vector3(1,1,0), 1, 4.0 * (0.8**9), 1.0, 0.8, 5, 0, Vector3(), []),
                soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1,1,0), 1, 0.0, 1.0, 0.8, 0, 0,
                          Vector3(), []),
                soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 4.0, 0.0, 1.0, 0, 0,
                      Vector3(), []),
                soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1,1,0), 1, 4.0, 1.0, 1.0, 0, 0,
                      Vector3(), []),
                soMessage(Header(None, now, 'None'), Vector3(2,2,0), 1, 4.0 * (0.75**4), 0.0, 0.75, 5, 0,
                          Vector3(), []),
                soMessage(Header(None, now - rospy.Duration(2), 'None'), Vector3(5,5,0), 1, 4.0 * 0.8, 0.0, 0.8, 3, 0,
                          Vector3(), []),
                soMessage(Header(None, now, 'None'), Vector3(6,6,0), 1, 4.0, 0.0, 0.8, 5, 0, Vector3(), [])],
                'gradient': [],
                'robo': [
                soMessage(Header(None, now - rospy.Duration(2), 'robo'), Vector3(4, 4, 0), 1, 4.0 * (0.8 ** 2), 0.0,
                              0.8, 4, 0, Vector3(), []),

                ]}

        self.assertEqual(bffr.get_data(), data)

    def test_evaporation_msg(self):
        """
        test the evaporation of one specified message
        :return:
        """
        bffr = soBuffer.SoBuffer(min_diffusion=1.0)
        now = rospy.Time.now()

        # with goal radius --> should be kept
        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 4.0, 1.0, 0.8, 5, 0,
                  Vector3(), [])
        result = soMessage(Header(None, now, 'None'), Vector3(1, 1, 0), 1, 4.0 * (0.8 ** 9), 1.0, 0.8, 5, 0,
                  Vector3(), [])
        self.assertEqual(bffr._evaporate_msg(msg), result)

        # without goal radius --> should be deleted
        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 4.0, 0.0, 0.8, 5, 0,
                  Vector3(), [])
        self.assertEqual(bffr._evaporate_msg(msg), None)

        # without goal radius & ev time is 0, ev factor < 1 --> should be deleted
        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 4.0, 0.0, 0.8, 0, 0,
                        Vector3(), [])
        self.assertEqual(bffr._evaporate_msg(msg), None)

        # without goal radius & ev time is 0, ev factor == 1.0 --> kept as it is
        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 4.0, 0.0, 1.0, 0, 0,
                        Vector3(), [])
        result = soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 4.0, 0.0, 1.0, 0, 0,
                  Vector3(), [])
        self.assertEqual(bffr._evaporate_msg(msg), result)

        # with goal radius & ev time is 0, ev factor < 1.0
        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 4.0, 1.0, 0.0, 0, 0,
                        Vector3(), [])
        result = soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 0.0, 1.0, 0.0, 0, 0,
                  Vector3(), [])
        self.assertEqual(bffr._evaporate_msg(msg), result)


    # GOAL REACHED
    def test_get_goal_reached(self):
        bffr = soBuffer.SoBuffer()
        now = rospy.Time.now()

        # no gradients available
        self.assertEqual(bffr.get_goal_reached(Point(0,0,0)), False)

        msg = soMessage(Header(None, now - rospy.Duration(45), 'None'), Vector3(1, 1, 0), 1, 4.0, 0.0, 1.0, 0, 0,
                    Vector3(), [])
        bffr.store_data(msg)
        # no gradients available
        self.assertEqual(bffr.get_goal_reached(Point(0,0,0)), False)





    # POTENTIAL FIELD VECTOR CALCULATIONS
    def test_calc_attractive_gradient(self):
        """
        test _calc_attractive_gradient method for 2D and 3D
        """
        bffr = soBuffer.SoBuffer()

        # 2D - D < r <= C
        gradient = soMessage(None, Vector3(3, 4, 0), 1, 5.0, 1.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(0, 0, 0)
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), Vector3(0.6 * 0.8, 0.8 * 0.8, 0))

        # 2D - r > C
        gradient = soMessage(None, Vector3(3, 4, 0), 1, 2.0, 1.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(0, 0, 0)
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), Vector3(0.6, 0.8, 0))

        # 2D - r <= D
        gradient = soMessage(None, Vector3(3, 4, 0), 1, 2.0, 5.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(0, 0, 0)
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), Vector3(0, 0, 0), [])

        # 2D - r > C - non zero robot pose
        gradient = soMessage(None, Vector3(4, 5, 0), 1, 2.0, 1.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(1, 1, 0)
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), Vector3(0.6, 0.8, 0))

        # 3D - D < r <= C
        gradient = soMessage(None, Vector3(3, 5, 10), 1, 6.0, 2.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(1, 2, 4)
        result = Vector3((2.0 / 7.0) * (5.0 / 6.0), (3.0 / 7.0) * (5.0 / 6.0), (6.0 / 7.0) * (5.0 / 6.0))
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), result)

        # 3D - r > C
        gradient = soMessage(None, Vector3(3, 5, 10), 1, 5.0, 2.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(1, 2, 4)
        result = Vector3((2.0 / 7.0), (3.0 / 7.0), (6.0 / 7.0))
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), result)

        # 3D - r <= D
        gradient = soMessage(None, Vector3(3, 5, 10), 1, 5.0, 7.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(1, 2, 4)
        self.assertEqual(bffr._calc_attractive_gradient(gradient, pose), Vector3(0, 0, 0))

    def test_calc_repulsive_gradient(self):
        """
        test _calc_repulsive_gradient method
        """
        bffr = soBuffer.SoBuffer()

        # 2D - D < r <= C
        gradient = soMessage(None, Vector3(3, 4, 0), -1, 5.0, 1.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(0, 0, 0)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(-0.6 * 0.2, -0.8 * 0.2, 0))

        # 2D - r > C
        gradient = soMessage(None, Vector3(3, 4, 0), -1, 2.0, 1.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(0, 0, 0)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(0, 0, 0))

        # 2D - r <= D
        gradient = soMessage(None, Vector3(3, 4, 0), -1, 2.0, 5.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(0, 0, 0)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(-1 * np.inf, -1 * np.inf, -1 * np.inf))

        # 2D - r > C - non zero robot pose
        gradient = soMessage(None, Vector3(4, 5, 0), -1, 2.0, 1.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(1, 1, 0)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(0, 0, 0))

        # 3D - D < r <= C
        gradient = soMessage(None, Vector3(3, 5, 10), -1, 6.0, 2.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(1, 2, 4)
        result = Vector3((-2.0 / 7.0) * (1.0 / 6.0), (-3.0 / 7.0) * (1.0 / 6.0), (-6.0 / 7.0) * (1.0 / 6.0))
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), result)

        # 3D - r > C
        gradient = soMessage(None, Vector3(3, 5, 10), -1, 5.0, 2.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(1, 2, 4)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(0, 0, 0))

        # 3D - r <= D
        gradient = soMessage(None, Vector3(3, 5, 10), -1, 5.0, 7.0, 1.0, 0, 0, Vector3(), [])
        pose = Point(1, 2, 4)
        self.assertEqual(bffr._calc_repulsive_gradient(gradient, pose), Vector3(-1 * np.inf, -1 * np.inf, -1 * np.inf))

    def test_repulsion_ge(self): #TODO
        """
        test repulsion vector calculation based on Ge
        :return:
        """
        bffr = soBuffer.SoBuffer()
        gradient = soMessage(None, Vector3(4, 2, 0), 1, 3.0, 1.0, 1.0, 0, 0, Vector3(), [])
        goal = soMessage(None, Vector3(2, 2, 0), 1, 3.0, 1.0, 1.0, 0, 0, Vector3(), [])
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


# run tests - start roscore before running tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()

