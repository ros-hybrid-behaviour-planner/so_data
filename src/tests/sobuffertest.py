"""
Created on 14.11.2016

@author: kaiser

Unit test for sobuffer.py
"""

from so_data.sobuffer import SoBuffer, AGGREGATION
import unittest
from so_data.msg import SoMessage
import rospy
from geometry_msgs.msg import Vector3, Quaternion
from copy import deepcopy
from std_msgs.msg import Header
import numpy as np


class SoBufferTest(unittest.TestCase):
    """
    testing of soBuffer methods
    """
    maxDiff = None

    # TODO: agent list dings draus machen
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
            'robot': {
                'robot1': [SoMessage(),
                       SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1,
                                 1.0, 1.0, 1.0, 0, None, Vector3(), 0, 0, True,
                                 [])],
                'robot2': [SoMessage(),
                       SoMessage(None, None, Vector3(5, 6, 0), Quaternion(), 1,
                                 2.0, 1.0, 1.0, 0, None, Vector3(), 0, 0,
                                 True, [])],
                'robot3': [SoMessage(), SoMessage(), SoMessage(None, None,
                                                           Vector3(1, 2, 0),
                                                           Quaternion(),
                                                           1, 4.0, 1.0, 1.0, 0,
                                                           None, Vector3(),
                                                           0, 0,
                                                           True, [])],
                'robot4': []
            }
        }

        bffr._static = {
            'None': [SoMessage(None, None, Vector3(5, 6, 5), Quaternion(), 1,
                               1.0, 1.0, 1.0, 0, None, Vector3(), 0, 0,
                               False, []), SoMessage()],
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 1, 1), Quaternion(), 1, 1.0, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]

        result = [
            SoMessage(None, None, Vector3(1, 2, 0), Quaternion(), 1, 4.0, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, True, []),
            SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 1.0, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, True, [])]

        self.assertEqual(bffr.quorum_list(), result)
        bffr.result_static = True
        self.assertEqual(bffr.quorum_list(), result)

    # GOAL REACHED
    def test_get_goal_reached(self):
        """
        test get_goal_reached method
        :return:
        """
        bffr = SoBuffer()

        # no gradients available --> True
        bffr._own_pos = [
            SoMessage(None, None, Vector3(2, 2, 2), Quaternion(), -1, 3.0, 0.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        self.assertEqual(bffr.get_goal_reached(), False)

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(), 1, 3.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(), 1, 1.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])],
            'None': [
                SoMessage(None, None, Vector3(7, 3, 2), Quaternion(), -1, 3.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(), 1, 2.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])],
            'test': [
                SoMessage(None, None, Vector3(5, 3, 2), Quaternion(), -1, 3.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, []),
                SoMessage(None, None, Vector3(7, 2, 3), Quaternion(), -1, 3.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, []),
                SoMessage(None, None, Vector3(1, 2, 6), Quaternion(), 1, 4.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])]}

        # goal reached
        bffr._own_pos = [
            SoMessage(None, None, Vector3(2, 2, 2), Quaternion(), -1, 3.0, 0.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        self.assertEqual(bffr.get_goal_reached(), True)
        bffr._own_pos = [
            SoMessage(None, None, Vector3(2, 2, 1), Quaternion(), -1, 3.0, 0.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        self.assertEqual(bffr.get_goal_reached(), True)
        # goal not reached
        bffr._own_pos = [
            SoMessage(None, None, Vector3(6, 7, 2), Quaternion(), -1, 3.0, 0.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        self.assertEqual(bffr.get_goal_reached(), False)
        # only consider some frameIDs
        bffr._own_pos = [
            SoMessage(None, None, Vector3(2, 2, 2), Quaternion(), -1, 3.0, 0.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        bffr.chem_frames = ['gradient']
        self.assertEqual(bffr.get_goal_reached(), True)
        bffr.chem_frames = ['None', 'test']
        self.assertEqual(bffr.get_goal_reached(), False)

    # no potential
    def test_get_attraction_bool(self):
        """
        test get_no_potential method
        :return:
        """
        bffr = SoBuffer()

        # no gradients available --> True
        bffr._own_pos = [
            SoMessage(None, None, Vector3(2, 2, 2), Quaternion(), -1, 3.0, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        self.assertEqual(bffr.get_attraction_bool(), False)

        bffr._static = {
            'gradient': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(), 1, 3.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, []),
                SoMessage(None, None, Vector3(2, 2, 2), Quaternion(), 1, 1.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])],
            'None': [
                SoMessage(None, None, Vector3(7, 3, 2), Quaternion(), -1, 3.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, []),
                SoMessage(None, None, Vector3(5, 6, 3), Quaternion(), 1, 2.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])],
            'test': [
                SoMessage(None, None, Vector3(5, 3, 2), Quaternion(), -1, 3.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, []),
                SoMessage(None, None, Vector3(7, 2, 3), Quaternion(), -1, 3.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, []),
                SoMessage(None, None, Vector3(1, 2, 6), Quaternion(), 1, 4.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])]}

        # potential
        bffr._own_pos = [
            SoMessage(None, None, Vector3(2, 2, 2), Quaternion(), -1, 3.0, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        self.assertEqual(bffr.get_attraction_bool(), True)
        bffr._own_pos = [
            SoMessage(None, None, Vector3(2, 2, 1), Quaternion(), -1, 3.0, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        self.assertEqual(bffr.get_attraction_bool(), True)
        # no potential
        bffr._own_pos = [
            SoMessage(None, None, Vector3(0, 9, 9), Quaternion(), -1, 3.0, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        self.assertEqual(bffr.get_attraction_bool(), False)

        # only consider some frameIDs
        # no potential
        bffr._own_pos = [
            SoMessage(None, None, Vector3(2, 2, 2), Quaternion(), -1, 3.0, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        bffr.chem_frames = ['gradient']
        self.assertEqual(bffr.get_attraction_bool(), False)
        # potential
        bffr.chem_frames = ['None', 'test']
        self.assertEqual(bffr.get_attraction_bool(), True)

    def test_get_neighbors_bool(self):
        """
        test get_neighbors_bool method
        :return:
        """

        bffr = SoBuffer(view_distance=2.0,
                        repulsion=REPULSION.REPULSION)

        # no gradients available --> True
        bffr._own_pos = [
            SoMessage(None, None, Vector3(2, 2, 2), Quaternion(), -1, 3.0, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        self.assertEqual(bffr.get_neighbors_bool(), True)

        bffr._moving = {
            'robot': {'robot1': [
                SoMessage(None, None, Vector3(2, 3, 1), Quaternion(), -1, 3.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, []),
                SoMessage(None, None, Vector3(2, 1, 2), Quaternion(), -1, 1.0,
                          1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])]
        }}

        self.assertEqual(bffr.get_neighbors_bool(), False)

        bffr._own_pos = [
            SoMessage(None, None, Vector3(9, 9, 9), Quaternion(), -1, 3.0, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, False, [])]
        self.assertEqual(bffr.get_neighbors_bool(), True)


    # EVAPORATION
    def test_evaporation_buffer(self):
        """
        test evaporation of buffer data using evaporate_buffer method
        """

        bffr = SoBuffer(aggregation=AGGREGATION.MAX, min_diffusion=1.0)
        now = rospy.Time.now()

        data = {'None': [  # message has goal radius - should be kept
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                      now - rospy.Duration(45), Vector3(), 0, 0,
                      False, []),
            # evaporation time is zero cases
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 0,
                      now - rospy.Duration(45), Vector3(), 0, 0,
                      False, []),
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), 1, 4.0, 0.0, 1.0, 0,
                      now - rospy.Duration(45), Vector3(), 0, 0,
                      False, []),
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), 1, 4.0, 1.0, 1.0, 0,
                      now - rospy.Duration(45), Vector3(), 0, 0,
                      False, []),
            # messages without goal radius - will be sorted out based on
            # min diffusion
            SoMessage(Header(None, now - rospy.Duration(20), 'None'), 'None',
                      Vector3(2, 2, 0), Quaternion(), 1, 4.0, 0.0, 0.75, 5,
                      now - rospy.Duration(20), Vector3(), 0, 0,
                      False, []),
            SoMessage(Header(None, now - rospy.Duration(5), 'None'), 'None',
                      Vector3(5, 5, 0), Quaternion(), 1, 4.0, 0.0, 0.8, 3,
                      now - rospy.Duration(5), Vector3(), 0, 0,
                      False, []),
            SoMessage(Header(None, now, 'None'), 'None', Vector3(6, 6, 0),
                      Quaternion(),
                      1, 4.0, 0.0, 0.8, 5, now, Vector3(), 0, 0, False, [])
        ], 'gradient': [
            SoMessage(Header(None, now - rospy.Duration(45), 'gradient'), 'None',
                      Vector3(1, 1, 0), Quaternion(), 1, 4.0, 0.0, 0.8, 5,
                      now - rospy.Duration(45), Vector3(), 0, 0,
                      False, []),
            SoMessage(Header(None, now - rospy.Duration(15), 'gradient'), 'None',
                      Vector3(3, 3, 0), Quaternion(), 1, 4.0, 0.0, 0.6, 5,
                      now - rospy.Duration(15), Vector3(), 0,
                      0, False, [])
        ], 'robo': [
            SoMessage(Header(None, now - rospy.Duration(10), 'robo'), 'None',
                      Vector3(4, 4, 0), Quaternion(), 1, 4.0, 0.0, 0.8, 4,
                      now - rospy.Duration(10), Vector3(), 0, 0,
                      False, [])]}

        bffr._static = deepcopy(data)
        bffr._evaporate_buffer()

        data = {'None': [
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), 1, 4.0 *
                      (0.8 ** 9), 1.0, 0.8, 5, now, Vector3(), 0, 0, False,
                      []),
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), 1, 0.0, 1.0, 0.8, 0,
                      now - rospy.Duration(45), Vector3(), 0, 0,
                      False, []),
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), 1, 4.0, 0.0, 1.0, 0,
                      now - rospy.Duration(45), Vector3(), 0,
                      0, False, []),
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), 1, 4.0, 1.0, 1.0, 0,
                      now - rospy.Duration(45), Vector3(), 0,
                      0, False, []),
            SoMessage(Header(None, now - rospy.Duration(20), 'None'), 'None',
                      Vector3(2, 2, 0), Quaternion(), 1,
                      4.0 * (0.75 ** 4), 0.0, 0.75, 5, now, Vector3(), 0, 0,
                      False, []),
            SoMessage(Header(None, now - rospy.Duration(5), 'None'), 'None',
                      Vector3(5, 5, 0), Quaternion(), 1, 4.0 * 0.8, 0.0, 0.8,
                      3,
                      now - rospy.Duration(2), Vector3(), 0, 0, False, []),
            SoMessage(Header(None, now, 'None'), 'None', Vector3(6, 6, 0),
                      Quaternion(), 1, 4.0, 0.0, 0.8, 5, now, Vector3(), 0, 0,
                      False, [])],
            'gradient': [],
            'robo': [
                SoMessage(Header(None, now - rospy.Duration(10), 'robo'), 'None',
                          Vector3(4, 4, 0), Quaternion(), 1, 4.0 * (0.8 ** 2),
                          0.0,
                          0.8, 4, now - rospy.Duration(2), Vector3(), 0, 0,
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
        msg = SoMessage(Header(None, now - rospy.Duration(45), 'None'), None,
                        Vector3(1, 1, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        now - rospy.Duration(45), Vector3(), 0, 0,
                        False, [])
        result = SoMessage(Header(None, now - rospy.Duration(45), 'None'),
                           None, Vector3(1, 1, 0), Quaternion(), 1,
                           4.0 * (0.8 ** 9), 1.0, 0.8, 5, now, Vector3(), 0, 0,
                           False, [])
        self.assertEqual(bffr._evaporate_msg(msg), result)

        # without goal radius --> should be deleted
        msg = SoMessage(Header(None, now - rospy.Duration(45), 'None'), None,
                        Vector3(1, 1, 0), Quaternion(), 1, 4.0, 0.0, 0.8, 5,
                        now - rospy.Duration(45), Vector3(), 0, 0,
                        False, [])
        self.assertEqual(bffr._evaporate_msg(msg), None)

        # without goal radius & ev time is 0, ev factor < 1
        # --> should be deleted
        msg = SoMessage(Header(None, now - rospy.Duration(45), 'None'), None,
                        Vector3(1, 1, 0), Quaternion(), 1, 4.0, 0.0, 0.8, 0,
                        now - rospy.Duration(45), Vector3(), 0, 0,
                        False, [])
        self.assertEqual(bffr._evaporate_msg(msg), None)

        # without goal radius & ev time is 0, ev factor == 1.0
        # --> kept as it is
        msg = SoMessage(Header(None, now - rospy.Duration(45), 'None'), None,
                        Vector3(1, 1, 0), Quaternion(), 1, 4.0, 0.0, 1.0, 0,
                        now - rospy.Duration(45), Vector3(), 0, 0,
                        False, [])
        result = SoMessage(Header(None, now - rospy.Duration(45), 'None'),
                           None, Vector3(1, 1, 0), Quaternion(), 1, 4.0, 0.0,
                           1.0, 0, now - rospy.Duration(45), Vector3(), 0, 0,
                           False, [])
        self.assertEqual(bffr._evaporate_msg(msg), result)

        # with goal radius & ev time is 0, ev factor < 1.0
        msg = SoMessage(Header(None, now - rospy.Duration(45), 'None'), None,
                        Vector3(1, 1, 0), Quaternion(), 1, 4.0, 1.0, 0.0, 0,
                        now - rospy.Duration(45), Vector3(), 0, 0, False, [])
        result = SoMessage(Header(None, now - rospy.Duration(45), 'None'),
                           None, Vector3(1, 1, 0), Quaternion(), 1, 0.0, 1.0,
                           0.0, 0, now - rospy.Duration(45), Vector3(), 0, 0,
                           False, [])
        self.assertEqual(bffr._evaporate_msg(msg), result)

    # STORE DATA IN SoBUFFER
    def test_store_data_max(self):
        """
        test store_data method aggregation option = max
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX},
                        aggregation_distance=1.0)
        testlist = []

        msg = SoMessage(None, None, Vector3(2, 0.8, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 5.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        # store max value within aggregation distance!
        msg = SoMessage(None, None, Vector3(2, 1.5, 0), Quaternion(), 1, 6.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        # keep max of both
        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 3.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        # only value at this postion / area
        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'
            el.parent_frame = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_max_aggregation_distance(self):
        """
        test store_data method aggregation option = max, aggregation distance = 0.0
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX},
                        aggregation_distance=0.0)
        testlist = []

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 5.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        # store max value within aggregation distance!
        msg = SoMessage(None, None, Vector3(2, 1, 0), Quaternion(), 1, 6.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        # keep max of both
        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 3.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        # only value at this postion / area
        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'
            el.parent_frame = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_fids(self):
        """
        test store_data method with different frame IDs (all)
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX})
        testlist = {'None': [], 'grad': []}
        now = rospy.Time.now()

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(2, 2, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(3, 3, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist['grad'].append(msg)
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(3, 3, 0),
                        Quaternion(), 1, 3.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(5, 5, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(2, 2, 0),
                        Quaternion(), 1, 5.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg)

        self.assertEqual(bffr._static, testlist)

    def test_store_data_fids_selection(self):
        """
        test store_data method, only store gradients with specified frameids
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX},
                        store_all=False, framestorage=['grad', 'silk'])
        testlist = {'grad': [], 'silk': []}
        now = rospy.Time.now()

        msg = SoMessage(Header(None, now, 'pheromone'), 'None', Vector3(2, 2, 0),
                        Quaternion(), 1,
                        4.0, 1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(3, 3, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 3.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(5, 5, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'silk'), 'None', Vector3(5, 5, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist['silk'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 5.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        self.assertEqual(bffr._static, testlist)

    def test_store_data_min(self):
        """
        test store_data method aggregation option = min
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MIN},
                        store_all=True)
        testlist = []

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 3.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 5.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'
            el.parent_frame = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_avg(self):
        """
        test store_data method aggregation option = avg
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.AVG},
                        min_diffusion=1.0)
        testlist = []

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 3.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(
            SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 3.5, 1.0,
                      1.0, 0, None, Vector3(), 0, 0, False, []))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(2, 1, 0), Quaternion(), -1, 5.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(
            SoMessage(None, None, Vector3(2, 1.5, 0), Quaternion(), -1, 1.0,
                      0.0, 1.0, 0, None, Vector3(), 0, 0, False, []))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), -1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'
            el.parent_frame = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_newest(self):
        """
        test store_data method aggregation option = newest
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.NEW})
        testlist = []

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), 1, 3.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), -1, 5.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), -1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg)

        for el in testlist:
            el.header.frame_id = 'None'
            el.parent_frame = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_ev(self):
        """
        test store_data method, evaporation of received data
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX})
        testlist = {'None': []}
        now = rospy.Time.now()

        # not to be stored - ev time is zero, no goal radius
        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), 1, 4.0,
                        0.0, 0.3, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        # received data is older than stored one, will be evaporated and than compared
        msg = SoMessage(Header(None, now - rospy.Duration(10), None), None,
                        Vector3(2, 2, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        now - rospy.Duration(10), Vector3(), 0, 0, False, [])
        result = SoMessage(Header(None, now - rospy.Duration(10), 'None'),
                           'None', Vector3(2, 2, 0), Quaternion(), 1,
                           4.0 * (0.8 ** 2), 1.0, 0.8, 5, now, Vector3(), 0, 0,
                           False, [])
        testlist['None'].append(result)
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now - rospy.Duration(15), None), None,
                        Vector3(2, 2, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        now - rospy.Duration(15), Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        self.assertEqual(bffr._static, testlist)

    def test_store_data_neighbors(self):
        """
        test store_data method, store neighbor data and own pos
        """
        bffr = SoBuffer(aggregation={'DEFAULT':AGGREGATION.MAX}, id='robot3')
        testlist = {'robot': {'robot1': [], 'robot2': []}}
        ownpos = []
        now = rospy.Time.now()

        # replaced by fourth robot1 message
        msg = SoMessage(Header(None, now - rospy.Duration(10), 'robot'), 'robot1',
                        Vector3(2.1, 2.2, 0), Quaternion(), 1, 4.0, 1.0, 0.8,
                        5, None, Vector3(), 0, 0, True, [])
        bffr.store_data(msg)

        # position older than newest stored position --> ignore
        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot'), 'robot1',
                        Vector3(2, 2, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        None, Vector3(), 0, 0, True, [])
        bffr.store_data(msg)

        # add to position list
        msg = SoMessage(Header(None, now - rospy.Duration(5), 'robot'), 'robot1',
                        Vector3(2, 2, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        None, Vector3(), 0, 0, True, [])
        testlist['robot']['robot1'].append(msg)
        bffr.store_data(msg)

        # add to position list
        msg = SoMessage(Header(None, now, 'robot'), 'robot1', Vector3(2, 2, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 0.8, 5, None, Vector3(), 0, 0, True, [])
        testlist['robot']['robot1'].append(msg)
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot'), 'robot2',
                        Vector3(2, 2, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        None, Vector3(), 0, 0, True, [])
        testlist['robot']['robot2'].append(msg)
        bffr.store_data(msg)

        # own position
        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot'), 'robot3',
                        Vector3(2, 2, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        None, Vector3(), 0, 0, True, [])
        ownpos.append(msg)
        bffr.store_data(msg)

        self.assertEqual(bffr._moving, testlist)
        self.assertEqual(bffr._own_pos, ownpos)

    def test_store_data_neighbors_False(self):
        """
        test store_data method, neighbor gradients will not be stored
        :return:
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX},
                        moving_storage_size=0, id='')
        now = rospy.Time.now()

        # replaced by fourth robot1 message
        msg = SoMessage(Header(None, now - rospy.Duration(10), 'robot1'), None,
                        Vector3(2.1, 2.2, 0), Quaternion(), 1, 4.0, 1.0, 0.8,
                        5, None, Vector3(), 0, 0, True, [])
        bffr.store_data(msg)

        # position older than newest stored position --> ignore
        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot1'), None,
                        Vector3(2, 2, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        None, Vector3(), 0, 0, True, [])
        bffr.store_data(msg)

        # add to position list
        msg = SoMessage(Header(None, now - rospy.Duration(5), 'robot1'), None,
                        Vector3(2, 2, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        None, Vector3(), 0, 0, True, [])
        bffr.store_data(msg)

        # add to position list
        msg = SoMessage(Header(None, now, 'robot1'), None, Vector3(2, 2, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 0.8, 5, None, Vector3(), 0, 0, True, [])
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot2'), None,
                        Vector3(2, 2, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        None, Vector3(), 0, 0, True, [])
        bffr.store_data(msg)

        # own position
        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot3'), None,
                        Vector3(2, 2, 0), Quaternion(), 1, 4.0, 1.0, 0.8, 5,
                        None, Vector3(), 0, 0, True, [])
        bffr.store_data(msg)

        self.assertEqual(bffr._moving, {})
        self.assertEqual(bffr._own_pos, [])

    def test_store_data_various_aggregations(self):
        """
        test store data method with different aggregation options for different frame IDs
        :return:
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX,
                                     'grad': AGGREGATION.AVG})
        testlist = {'None': [], 'grad': []}
        now = rospy.Time.now()

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(2, 2, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(4, 5, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(3, 3, 0),
                        Quaternion(), 1, 3.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(5, 5, 0),
                        Quaternion(), 1, 4.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist['grad'].append(SoMessage(Header(None, now, 'grad'), 'None',
                                          Vector3(4.5, 5, 0), Quaternion(), 1,
                                          4.0, 1.0, 1.0,
                                          0, None, Vector3(), 0, 0, False, []))
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(2, 2, 0),
                        Quaternion(), 1, 5.0,
                        1.0, 1.0, 0, None, Vector3(), 0, 0, False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg)

        self.assertEqual(bffr._static, testlist)

    def test_aggregation_newparent(self):
        """
        test aggregation_newparent method
        :return:
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.NEWPARENT})

        now = rospy.Time.now()

        testlist = {'robot': []}

        # replaced by fourth robot1 message
        msg = SoMessage(Header(None, now - rospy.Duration(10), 'robot'), 'robot1',
                        Vector3(2.1, 2.2, 0), Quaternion(), 1, 4.0, 1.0, 0.8,
                        5, None, Vector3(), 0, 0, False, [])
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now - rospy.Duration(5), 'robot'), 'robot5',
                        Vector3(2.1, 2.2, 0), Quaternion(), 1, 4.0, 1.0, 0.8,
                        5, None, Vector3(), 0, 0, False, [])
        testlist['robot'].append(msg)
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'robot'), 'robot3',
                        Vector3(2.1, 2.2, 0), Quaternion(), 1, 4.0, 1.0, 0.8,
                        5, None, Vector3(), 0, 0, False, [])
        testlist['robot'].append(msg)
        bffr.store_data(msg)

        msg = SoMessage(Header(None, now, 'robot'), 'robot1',
                        Vector3(2.1, 2.2, 0), Quaternion(), 1, 4.0, 1.0, 0.8,
                        5, None, Vector3(), 0, 0, False, [])
        testlist['robot'].append(msg)
        bffr.store_data(msg)

        self.assertEqual(bffr._static, testlist)

# run tests - start roscore before running tests
if __name__ == "__main__":
    rospy.init_node('test')
    unittest.main()
