"""
Created on 14.11.2016

@author: kaiser

Unit test for sobuffer.py
"""
import unittest
import numpy as np
from copy import deepcopy
import rospy
from so_data.msg import SoMessage
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
from so_data.sobuffer import SoBuffer, AGGREGATION


class SoBufferTest(unittest.TestCase):
    """
    testing of soBuffer methods
    """
    maxDiff = None

    def __init__(self, *args, **kwargs):
        super(SoBufferTest, self).__init__(*args, **kwargs)
        self.now = rospy.Time(secs=500000)

    def test_agent_list(self):
        """
        test agent list function
        :return:
        """
        bffr = SoBuffer(view_distance=2.0)

        self.assertEqual(bffr.agent_list([bffr.pose_frame], time=self.now), [])

        # 2 neighbors within view, one outside view
        bffr._moving = {
            'robot': {
                'robot1': [SoMessage(),
                           SoMessage(None, None, Vector3(2, 2, 0),
                                     Quaternion(), Vector3(), 1, 1.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot2': [SoMessage(),
                           SoMessage(None, None, Vector3(5, 6, 0),
                                     Quaternion(), Vector3(), 1, 2.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot3': [SoMessage(),
                           SoMessage(),
                           SoMessage(None, None, Vector3(1, 2, 0),
                                     Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot4': []
            }
        }

        bffr._static = {
            'None': [SoMessage(None, None, Vector3(5, 6, 5), Quaternion(),
                               Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False,
                               []), SoMessage()],
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 1, 1), Quaternion(), Vector3(),
                      1, 1.0, 1.0, 1.0, 0, None, False, [])]

        result = [
            SoMessage(None, None, Vector3(1, 2, 0), Quaternion(), Vector3(), 1,
                      4.0, 1.0, 1.0, 0, None, True, []),
            SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(), 1,
                      1.0, 1.0, 1.0, 0, None, True, [])]

        self.assertEqual(bffr.agent_list([bffr.pose_frame], time=self.now), result)
        self.assertEqual(bffr.agent_list([bffr.pose_frame], time=self.now), result)

    # EVAPORATION
    def test_evaporation_buffer(self):
        """
        test evaporation of buffer data using evaporate_buffer method
        """

        bffr = SoBuffer(aggregation=AGGREGATION.MAX, min_diffusion=1.0)
        now = rospy.Time(secs=500000)

        data = {'None': [  # message has goal radius - should be kept
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                      0.8, 5, now - rospy.Duration(45), False, []),
            # evaporation time is zero cases
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                      0.8, 0, now - rospy.Duration(45), False, []),
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 0.0,
                      1.0, 0, now - rospy.Duration(45), False, []),
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                      1.0, 0, now - rospy.Duration(45), False, []),
            # messages without goal radius - will be sorted out based on
            # min diffusion
            SoMessage(Header(None, now - rospy.Duration(20), 'None'), 'None',
                      Vector3(2, 2, 0), Quaternion(), Vector3(), 1, 4.0, 0.0,
                      0.75, 5, now - rospy.Duration(20), False, []),
            SoMessage(Header(None, now - rospy.Duration(5), 'None'), 'None',
                      Vector3(5, 5, 0), Quaternion(), Vector3(), 1, 4.0, 0.0,
                      0.8, 3, now - rospy.Duration(5), False, []),
            SoMessage(Header(None, now, 'None'), 'None', Vector3(6, 6, 0),
                      Quaternion(), Vector3(), 1, 4.0, 0.0, 0.8, 5, now, False,
                      [])
        ], 'gradient': [
            SoMessage(Header(None, now - rospy.Duration(45), 'gradient'),
                      'None', Vector3(1, 1, 0), Quaternion(), Vector3(), 1,
                      4.0, 0.0, 0.8, 5, now - rospy.Duration(45), False, []),
            SoMessage(Header(None, now - rospy.Duration(15), 'gradient'),
                      'None', Vector3(3, 3, 0), Quaternion(), Vector3(), 1,
                      4.0, 0.0, 0.6, 5, now - rospy.Duration(15), False, [])
        ], 'robo': [
            SoMessage(Header(None, now - rospy.Duration(10), 'robo'), 'None',
                      Vector3(4, 4, 0), Quaternion(), Vector3(), 1, 4.0, 0.0,
                      0.8, 4, now - rospy.Duration(10), False, [])]}

        bffr._static = deepcopy(data)
        bffr._evaporate_buffer(time=now)

        data = {'None': [
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0 *
                      (0.8 ** 9), 1.0, 0.8, 5, now, False, []),
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 0.0, 1.0,
                      0.8, 0, now - rospy.Duration(45), False, []),
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 0.0,
                      1.0, 0, now - rospy.Duration(45), False, []),
            SoMessage(Header(None, now - rospy.Duration(45), 'None'), 'None',
                      Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                      1.0, 0, now - rospy.Duration(45), False, []),
            SoMessage(Header(None, now - rospy.Duration(20), 'None'), 'None',
                      Vector3(2, 2, 0), Quaternion(), Vector3(), 1,
                      4.0 * (0.75 ** 4), 0.0, 0.75, 5, now, False, []),
            SoMessage(Header(None, now - rospy.Duration(5), 'None'), 'None',
                      Vector3(5, 5, 0), Quaternion(), Vector3(), 1, 4.0 * 0.8,
                      0.0, 0.8, 3, now - rospy.Duration(2), False, []),
            SoMessage(Header(None, now, 'None'), 'None', Vector3(6, 6, 0),
                      Quaternion(), Vector3(), 1, 4.0, 0.0, 0.8, 5, now,
                      False, [])],
            'gradient': [],
            'robo': [
                SoMessage(Header(None, now - rospy.Duration(10), 'robo'),
                          'None', Vector3(4, 4, 0), Quaternion(), Vector3(), 1,
                          4.0 * (0.8 ** 2), 0.0, 0.8, 4,
                          now - rospy.Duration(2), False, [])
            ]}

        self.assertEqual(bffr._static, data)

    def test_evaporation_msg(self):
        """
        test the evaporation of one specified message
        :return:
        """
        bffr = SoBuffer(min_diffusion=1.0)
        now = rospy.Time(secs=500000)

        # with goal radius --> should be kept
        msg = SoMessage(Header(None, now - rospy.Duration(45), 'None'), None,
                        Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                        0.8, 5, now - rospy.Duration(45), False, [])
        result = SoMessage(Header(None, now - rospy.Duration(45), 'None'),
                           None, Vector3(1, 1, 0), Quaternion(), Vector3(), 1,
                           4.0 * (0.8 ** 9), 1.0, 0.8, 5, now, False, [])
        self.assertEqual(bffr._evaporate_msg(msg, now), result)

        # following tests do not seem to be valid!
        # ########################################
        # without goal radius --> should be deleted
        # msg = SoMessage(Header(None, now - rospy.Duration(45), 'None'), None,
        #                 Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 0.0,
        #                 0.8, 5, now - rospy.Duration(45), False, [])
        # msg = bffr._evaporate_msg(msg)
        # self.assertEqual(msg, None)

        # without goal radius & ev time is 0, ev factor < 1
        # --> should be deleted
        # msg = SoMessage(Header(None, now - rospy.Duration(45), 'None'), None,
        #                 Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 0.0,
        #                 0.8, 0, now - rospy.Duration(45), False, [])
        # msg = bffr._evaporate_msg(msg)
        # self.assertEqual(msg, None)

        # without goal radius & ev time is 0, ev factor == 1.0
        # --> kept as it is
        msg = SoMessage(Header(None, now - rospy.Duration(45), 'None'), None,
                        Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 0.0,
                        1.0, 0, now - rospy.Duration(45), False, [])
        result = SoMessage(Header(None, now - rospy.Duration(45), 'None'),
                           None, Vector3(1, 1, 0), Quaternion(), Vector3(), 1,
                           4.0, 0.0, 1.0, 0, now - rospy.Duration(45), False,
                           [])
        self.assertEqual(bffr._evaporate_msg(msg, now), result)

        # with goal radius & ev time is 0, ev factor < 1.0
        msg = SoMessage(Header(None, now - rospy.Duration(45), 'None'), None,
                        Vector3(1, 1, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                        0.0, 0, now - rospy.Duration(45), False, [])
        result = SoMessage(Header(None, now - rospy.Duration(45), 'None'),
                           None, Vector3(1, 1, 0), Quaternion(), Vector3(), 1,
                           0.0, 1.0, 0.0, 0, now - rospy.Duration(45), False,
                           [])
        self.assertEqual(bffr._evaporate_msg(msg, now), result)

    # STORE DATA IN SoBUFFER
    def test_store_data_max(self):
        """
        test store_data method aggregation option = max
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX},
                        aggregation_distance=1.0)
        now = rospy.Time(secs=500000)

        testlist = []

        msg = SoMessage(None, None, Vector3(2, 0.8, 0), Quaternion(),
                        Vector3(), 1, 4.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        1, 5.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        # store max value within aggregation distance!
        msg = SoMessage(None, None, Vector3(2, 1.5, 0), Quaternion(),
                        Vector3(), 1, 6.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        # keep max of both
        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 3.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        # only value at this postion / area
        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        for el in testlist:
            el.header.frame_id = 'None'
            el.parent_frame = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_max_aggregation_distance(self):
        """
        test store_data method aggregation option = max,
        aggregation distance = 0.0
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX},
                        aggregation_distance=0.0)
        now = rospy.Time(secs=500000)
        testlist = []

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        1, 5.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        # store max value within aggregation distance!
        msg = SoMessage(None, None, Vector3(2, 1, 0), Quaternion(), Vector3(),
                        1, 6.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        # keep max of both
        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 3.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        # only value at this postion / area
        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

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
        now = rospy.Time(secs=500000)

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(2, 2, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0, 0, None,
                        False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(3, 3, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0, 0, None,
                        False, [])
        testlist['grad'].append(msg)
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(3, 3, 0),
                        Quaternion(), Vector3(), 1, 3.0, 1.0, 1.0, 0, None,
                        False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(5, 5, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0, 0, None,
                        False, [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(2, 2, 0),
                        Quaternion(), Vector3(), 1, 5.0, 1.0, 1.0, 0, None,
                        False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg, now)

        self.assertEqual(bffr._static, testlist)

    def test_store_data_fids_selection(self):
        """
        test store_data method, only store gradients with specified frameids
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX},
                        store_all=False, framestorage=['grad', 'silk'])
        testlist = {'grad': [], 'silk': []}
        now = rospy.Time(secs=500000)

        msg = SoMessage(Header(None, now, 'pheromone'), 'None',
                        Vector3(2, 2, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                        1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(3, 3, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0, 0, None,
                        False, [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 3.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(5, 5, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0, 0, None,
                        False, [])
        testlist['grad'].append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'silk'), 'None', Vector3(5, 5, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0, 0, None,
                        False, [])
        testlist['silk'].append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        1, 5.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        self.assertEqual(bffr._static, testlist)

    def test_store_data_min(self):
        """
        test store_data method aggregation option = min
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MIN},
                        store_all=True)
        now = rospy.Time(secs=500000)
        testlist = []

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 3.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        1, 5.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

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
        now = rospy.Time(secs=500000)
        testlist = []

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 3.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(
            SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(), 1,
                      3.5, 1.0, 1.0, 0, None, False, []))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(2, 1, 0), Quaternion(), Vector3(),
                        -1, 5.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(
            SoMessage(None, None, Vector3(2, 1.5, 0), Quaternion(), Vector3(),
                      -1, 1.0, 0.0, 1.0, 0, None, False, []))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), Vector3(),
                        -1, 4.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        for el in testlist:
            el.header.frame_id = 'None'
            el.parent_frame = 'None'

        self.assertEqual(bffr._static, {'None': testlist})

    def test_store_data_newest(self):
        """
        test store_data method aggregation option = newest
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.NEW})
        now = rospy.Time(secs=500000)
        testlist = []

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(3, 3, 0), Quaternion(), Vector3(),
                        1, 3.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), Vector3(),
                        1, 4.0, 1.0, 1.0, 0, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        -1, 5.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(None, None, Vector3(5, 5, 0), Quaternion(), Vector3(),
                        -1, 4.0, 1.0, 1.0, 0, None, False, [])
        testlist.append(deepcopy(msg))
        bffr.store_data(msg, now)

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
        now = rospy.Time(secs=500000)

        # not to be stored - ev time is zero, no goal radius
        msg = SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                        1, 4.0, 0.0, 0.3, 0, None, False, [])
        bffr.store_data(msg, now)

        # received data is older than stored one, will be evaporated
        # and than compared
        msg = SoMessage(Header(None, now - rospy.Duration(10), None), None,
                        Vector3(2, 2, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                        0.8, 5, now - rospy.Duration(10), False, [])
        result = SoMessage(Header(None, now - rospy.Duration(10), 'None'),
                           'None', Vector3(2, 2, 0), Quaternion(), Vector3(),
                           1, 4.0 * (0.8 ** 2), 1.0, 0.8, 5, now, False, [])
        testlist['None'].append(result)
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now - rospy.Duration(15), None), None,
                        Vector3(2, 2, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                        0.8, 5, now - rospy.Duration(15), False, [])
        bffr.store_data(msg, now)

        self.assertEqual(bffr._static, testlist)

    def test_store_data_neighbors(self):
        """
        test store_data method, store neighbor data and own pos
        """
        bffr = SoBuffer(aggregation={'DEFAULT':AGGREGATION.MAX}, id='robot3')
        testlist = {'robot': {'robot1': [], 'robot2': []}}
        ownpos = []
        now = rospy.Time(secs=500000)

        # replaced by fourth robot1 message
        msg = SoMessage(Header(None, now - rospy.Duration(10), 'robot'),
                        'robot1', Vector3(2.1, 2.2, 0), Quaternion(),
                        Vector3(), 1, 4.0, 1.0, 0.8, 5, None, True, [])
        bffr.store_data(msg, now)

        # position older than newest stored position --> ignore
        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot'),
                        'robot1', Vector3(2, 2, 0), Quaternion(), Vector3(), 1,
                        4.0, 1.0, 0.8, 5, None, True, [])
        bffr.store_data(msg, now)

        # add to position list
        msg = SoMessage(Header(None, now - rospy.Duration(5), 'robot'),
                        'robot1', Vector3(2, 2, 0), Quaternion(), Vector3(), 1,
                        4.0, 1.0, 0.8, 5, None, True, [])
        testlist['robot']['robot1'].append(msg)
        bffr.store_data(msg, now)

        # add to position list
        msg = SoMessage(Header(None, now, 'robot'), 'robot1', Vector3(2, 2, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 0.8, 5, None,
                        True, [])
        testlist['robot']['robot1'].append(msg)
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot'),
                        'robot2', Vector3(2, 2, 0), Quaternion(), Vector3(), 1,
                        4.0, 1.0, 0.8, 5, None, True, [])
        testlist['robot']['robot2'].append(msg)
        bffr.store_data(msg, now)

        # own position
        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot'),
                        'robot3', Vector3(2, 2, 0), Quaternion(), Vector3(), 1,
                        4.0, 1.0, 0.8, 5, None, True, [])
        ownpos.append(msg)
        bffr.store_data(msg, now)

        self.assertEqual(bffr._moving, testlist)
        self.assertEqual(bffr._own_pos, ownpos)

    def test_store_data_neighbors_False(self):
        """
        test store_data method, neighbor gradients will not be stored
        :return:
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX},
                        moving_storage_size=0, id='')
        now = rospy.Time(secs=500000)

        # replaced by fourth robot1 message
        msg = SoMessage(Header(None, now - rospy.Duration(10), 'robot1'), None,
                        Vector3(2.1, 2.2, 0), Quaternion(), Vector3(), 1, 4.0,
                        1.0, 0.8, 5, None, True, [])
        bffr.store_data(msg, now)

        # position older than newest stored position --> ignore
        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot1'), None,
                        Vector3(2, 2, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                        0.8, 5, None, True, [])
        bffr.store_data(msg, now)

        # add to position list
        msg = SoMessage(Header(None, now - rospy.Duration(5), 'robot1'), None,
                        Vector3(2, 2, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                        0.8, 5, None, True, [])
        bffr.store_data(msg, now)

        # add to position list
        msg = SoMessage(Header(None, now, 'robot1'), None, Vector3(2, 2, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 0.8, 5, None,
                        True, [])
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot2'), None,
                        Vector3(2, 2, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                        0.8, 5, None, True, [])
        bffr.store_data(msg, now)

        # own position
        msg = SoMessage(Header(None, now - rospy.Duration(15), 'robot3'), None,
                        Vector3(2, 2, 0), Quaternion(), Vector3(), 1, 4.0, 1.0,
                        0.8, 5, None, True, [])
        bffr.store_data(msg, now)

        self.assertEqual(bffr._moving, {})
        self.assertEqual(bffr._own_pos, [])

    def test_store_data_various_aggregations(self):
        """
        test store data method with different aggregation options for
        different frame IDs
        :return:
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.MAX,
                                     'grad': AGGREGATION.AVG})
        testlist = {'None': [], 'grad': []}
        now = rospy.Time(secs=500000)

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(2, 2, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0, 0, None,
                        False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(4, 5, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0, 0, None,
                        False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(3, 3, 0),
                        Quaternion(), Vector3(), 1, 3.0, 1.0, 1.0, 0, None,
                        False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'grad'), 'None', Vector3(5, 5, 0),
                        Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0, 0, None,
                        False, [])
        testlist['grad'].append(SoMessage(Header(None, now, 'grad'), 'None',
                                          Vector3(4.5, 5, 0), Quaternion(),
                                          Vector3(), 1, 4.0, 1.0, 1.0, 0, None,
                                          False, []))
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'None'), 'None', Vector3(2, 2, 0),
                        Quaternion(), Vector3(), 1, 5.0, 1.0, 1.0, 0, None,
                        False, [])
        testlist['None'].append(deepcopy(msg))
        bffr.store_data(msg, now)

        self.assertEqual(bffr._static, testlist)

    def test_aggregation_newparent(self):
        """
        test aggregation_newparent method
        :return:
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.NEWPARENT})

        now = rospy.Time(secs=500000)

        testlist = {'robot': []}

        # replaced by fourth robot1 message
        msg = SoMessage(Header(None, now - rospy.Duration(10), 'robot'),
                        'robot1', Vector3(2.1, 2.2, 0), Quaternion(),
                        Vector3(), 1, 4.0, 1.0, 0.8, 5, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now - rospy.Duration(5), 'robot'),
                        'robot5', Vector3(2.1, 2.2, 0), Quaternion(),
                        Vector3(), 1, 4.0, 1.0, 0.8, 5, None, False, [])
        testlist['robot'].append(msg)
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'robot'), 'robot3',
                        Vector3(2.1, 2.2, 0), Quaternion(), Vector3(), 1, 4.0,
                        1.0, 0.8, 5, None, False, [])
        testlist['robot'].append(msg)
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'robot'), 'robot1',
                        Vector3(2.1, 2.2, 0), Quaternion(), Vector3(), 1, 4.0,
                        1.0, 0.8, 5, None, False, [])
        testlist['robot'].append(msg)
        bffr.store_data(msg, now)

        self.assertEqual(bffr._static, testlist)

    def test_aggregation_newframe(self):
        """
        test aggregation_newframe method
        :return:
        """
        bffr = SoBuffer(aggregation={'DEFAULT': AGGREGATION.NEWFRAME})

        now = rospy.Time(secs=500000)

        testlist = {'robot': []}

        # replaced by fourth robot1 message
        msg = SoMessage(Header(None, now - rospy.Duration(10), 'robot'),
                        'robot1', Vector3(2.1, 2.2, 0), Quaternion(),
                        Vector3(), 1, 4.0, 1.0, 0.8, 5, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now - rospy.Duration(5), 'robot'),
                        'robot5', Vector3(2.1, 2.2, 0), Quaternion(),
                        Vector3(), 1, 4.0, 1.0, 0.8, 5, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'robot'), 'robot3',
                        Vector3(2.1, 2.2, 0), Quaternion(), Vector3(), 1, 4.0,
                        1.0, 0.8, 5, None, False, [])
        bffr.store_data(msg, now)

        msg = SoMessage(Header(None, now, 'robot'), 'robot1',
                        Vector3(2.1, 2.2, 0), Quaternion(), Vector3(), 1, 4.0,
                        1.0, 0.8, 5, None, False, [])
        testlist['robot'].append(msg)
        bffr.store_data(msg, now)

        self.assertEqual(bffr._static, testlist)

    def test_repulsive_gradients(self):
        """
        test repulsive gradients method
        """
        bffr = SoBuffer(view_distance=2.0)

        # 2 neighbors within view, one outside view
        bffr._moving = {
            'robot': {
                'robot1': [SoMessage(),
                           SoMessage(None, None, Vector3(2, 2, 0),
                                     Quaternion(), Vector3(), -1, 1.0, 1.0,
                                     1.0, 0, None, True, [])],
                'robot2': [SoMessage(),
                           SoMessage(None, None, Vector3(5, 6, 0),
                                     Quaternion(), Vector3(), 1, 2.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot3': [SoMessage(),
                           SoMessage(),
                           SoMessage(None, None, Vector3(1, 2, 0),
                                     Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot4': []
            }
        }

        bffr._static = {
            'None': [SoMessage(None, None, Vector3(2, 1, 0), Quaternion(),
                               Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False,
                               []), SoMessage()],
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 1, 1), Quaternion(), Vector3(), 1,
                      1.0, 1.0, 1.0, 0, None, False, [])]

        result = [
            SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                      -1, 1.0, 1.0, 1.0, 0, None, True, [])]

        self.assertEqual(bffr.repulsive_gradients(static=False, time=self.now), result)

        result = [SoMessage(None, None, Vector3(2, 1, 0), Quaternion(),
                            Vector3(), -1, 1.0, 1.0, 1.0, 0, None, False, []),
                  SoMessage(None, None, Vector3(2, 2, 0), Quaternion(),
                            Vector3(), -1, 1.0, 1.0, 1.0, 0, None, True, [])]

        self.assertEqual(bffr.repulsive_gradients(time=self.now), result)

    def test_attractive_gradients(self):
        """
        test attractive gradients method
        """
        bffr = SoBuffer(view_distance=2.0)

        # 2 neighbors within view, one outside view
        bffr._moving = {
            'robot': {
                'robot1': [SoMessage(),
                           SoMessage(None, None, Vector3(2, 2, 0),
                                     Quaternion(), Vector3(), 1, 1.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot2': [SoMessage(),
                           SoMessage(None, None, Vector3(5, 6, 0),
                                     Quaternion(), Vector3(), 1, 2.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot3': [SoMessage(),
                           SoMessage(),
                           SoMessage(None, None, Vector3(1, 2, 0),
                                     Quaternion(), Vector3(), -1, 4.0, 1.0,
                                     1.0, 0, None, True, [])],
                'robot4': []
            }
        }

        bffr._static = {
            'None': [SoMessage(None, None, Vector3(2, 1, 0), Quaternion(),
                               Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False,
                            []), SoMessage()],
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 1, 1), Quaternion(), Vector3(), 1,
                      1.0, 1.0, 1.0, 0, None, False, [])]

        result = [
            SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(), 1,
                      1.0, 1.0, 1.0, 0, None, True, [])]

        self.assertEqual(bffr.attractive_gradients(static=False, time=self.now), result)

        result = [SoMessage(None, None, Vector3(2, 1, 0), Quaternion(),
                            Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False, []),
                  SoMessage(None, None, Vector3(2, 2, 0), Quaternion(),
                            Vector3(), 1, 1.0, 1.0, 1.0, 0, None, True, [])]

        self.assertEqual(bffr.attractive_gradients(time=self.now), result)

    def test_gradients(self):
        """
        test gradients method
        """
        bffr = SoBuffer(view_distance=2.0)

        # 2 neighbors within view, one outside view
        bffr._moving = {
            'robot': {
                'robot1': [SoMessage(),
                           SoMessage(None, None, Vector3(2, 2, 0),
                                     Quaternion(), Vector3(), 1, 1.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot2': [SoMessage(),
                           SoMessage(None, None, Vector3(5, 6, 0),
                                     Quaternion(), Vector3(), 1, 2.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot3': [SoMessage(),
                           SoMessage(),
                           SoMessage(None, None, Vector3(1, 2, 0),
                                     Quaternion(), Vector3(), -1, 4.0, 1.0,
                                     1.0, 0, None, True, [])],
                'robot4': []
            }
        }

        bffr._static = {
            'None': [SoMessage(None, None, Vector3(2, 1, 0), Quaternion(),
                               Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False,
                               []), SoMessage()],
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 1, 1), Quaternion(), Vector3(), 1,
                      1.0, 1.0, 1.0, 0, None, False, [])]

        result = [
            SoMessage(None, None, Vector3(1, 2, 0), Quaternion(), Vector3(),
                      -1, 4.0, 1.0, 1.0, 0, None, True, []),
            SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(), 1,
                      1.0, 1.0, 1.0, 0, None, True, [])
        ]

        self.assertEqual(bffr.gradients(static=False, time=self.now), result)

        result = [SoMessage(None, None, Vector3(2, 1, 0), Quaternion(),
                            Vector3(), 1, 1.0, 1.0, 1.0, 0, None, False, []),
                  SoMessage(None, None, Vector3(1, 2, 0), Quaternion(),
                            Vector3(), -1, 4.0, 1.0, 1.0, 0, None, True, []),
                  SoMessage(None, None, Vector3(2, 2, 0), Quaternion(),
                            Vector3(), 1, 1.0, 1.0, 1.0, 0, None, True, [])]

        self.assertEqual(bffr.gradients(time=self.now), result)

    def test_max_attractive_gradient(self):
        """
        test max attractive and strongest gradient methods
        """
        bffr = SoBuffer(view_distance=2.0)

        bffr._static = {
            'None': [SoMessage(None, None, Vector3(2, 1, 3), Quaternion(),
                               Vector3(), 1, 1.0, 0.1, 1.0, 0, None, False,
                               []),
                     SoMessage(None, None, Vector3(3, 3, 0), Quaternion(),
                               Vector3(), -1, 0.2, 0.5, 1.0, 0, None, False,
                               []),
                     SoMessage(None, None, Vector3(2, 2, 0), Quaternion(),
                               Vector3(), 1, 5.0, 0.2, 1.0, 0, None, False,
                               []),
                     ],
            'Center': [SoMessage(None, None, Vector3(1, 1, 1), Quaternion(),
                                 Vector3(), -1, 8.0, 1.0, 1.0, 0, None, False,
                                 []),
                       SoMessage(None, None, Vector3(3, 1, 2), Quaternion(),
                                 Vector3(), -1, 3.0, 0.1, 1.0, 0, None, False,
                                 [])]
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 1, 1), Quaternion(), Vector3(), 1,
                      1.0, 0, 1.0, 0, None, False, [])]

        self.assertEqual(bffr.max_attractive_gradient(time=self.now),
                         SoMessage(None, None, Vector3(2, 2, 0), Quaternion(),
                                   Vector3(), 1, 5.0, 0.2, 1.0, 0, None, False,
                                   []))

        self.assertEqual(bffr.min_attractive_gradient(time=self.now),
                         SoMessage(None, None, Vector3(2, 1, 3), Quaternion(),
                                   Vector3(), 1, 1.0, 0.1, 1.0, 0, None, False,
                                   []))

        self.assertEqual(bffr.strongest_gradient(time=self.now),
                         SoMessage(None, None, Vector3(1, 1, 1), Quaternion(),
                                   Vector3(), -1, 8.0, 1.0, 1.0, 0, None,
                                   False, []))

    def test_max_min_reach_gradient(self):
        """
        test max reach attractive gradient methods
        """
        bffr = SoBuffer(view_distance=2.0)

        bffr._static = {
            'None': [SoMessage(None, None, Vector3(2, 1, 0), Quaternion(),
                               Vector3(), 1, 2.0, 6.0, 1.0, 0, None, False,
                               []),
                     SoMessage(None, None, Vector3(3, 3, 0), Quaternion(),
                               Vector3(), -1, 0.2, 0.5, 1.0, 0, None, False,
                               []),
                     SoMessage(None, None, Vector3(2, 2, 0), Quaternion(),
                               Vector3(), 1, 5.0, 1.0, 1.0, 0, None, False,
                               []),
                     ],
            'Center': [SoMessage(None, None, Vector3(1, 1, 1), Quaternion(),
                                 Vector3(), -1, 8.0, 1.0, 1.0, 0, None, False,
                                 []),
                       SoMessage(None, None, Vector3(3, 1, 2), Quaternion(),
                                 Vector3(), -1, 3.0, 0.1, 1.0, 0, None, False,
                                 [])]
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 1, 1), Quaternion(), Vector3(), 1,
                      1.0, 0, 1.0, 0, None, False, [])]

        self.assertEqual(bffr.max_reach_attractive_gradient(time=self.now),
                         SoMessage(None, None, Vector3(2, 1, 0), Quaternion(),
                                   Vector3(), 1, 2.0, 6.0, 1.0, 0, None, False,
                                   []))

        self.assertEqual(bffr.min_reach_attractive_gradient(time=self.now),
                         SoMessage(None, None, Vector3(2, 2, 0), Quaternion(),
                                   Vector3(), 1, 5.0, 1.0, 1.0, 0, None, False,
                                   []))

    def test_agent_set(self):
        """
        test agent set function
        :return:
        """
        bffr = SoBuffer(view_distance=2.0)

        self.assertEqual(bffr.agent_set([bffr.pose_frame], time=self.now), [])

        # 2 neighbors within view, one outside view
        bffr._moving = {
            'robot': {
                'robot1': [SoMessage(None, None, Vector3(0, 2, 0),
                                     Quaternion(), Vector3(), 1, 1.0, 1.0, 1.0,
                                     0, None, True, []),
                           SoMessage(None, None, Vector3(2, 2, 0),
                                     Quaternion(), Vector3(), 1, 1.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot2': [SoMessage(),
                           SoMessage(None, None, Vector3(5, 6, 0),
                                     Quaternion(), Vector3(), 1, 2.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot3': [SoMessage(),
                           SoMessage(),
                           SoMessage(None, None, Vector3(1, 2, 0),
                                     Quaternion(), Vector3(), 1, 4.0, 1.0, 1.0,
                                     0, None, True, [])],
                'robot4': []
            }
        }

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 1, 1), Quaternion(), Vector3(), 1,
                      1.0, 1.0, 1.0, 0, None, False, [])]

        result = [
            [SoMessage(None, None, Vector3(1, 2, 0), Quaternion(), Vector3(),
                       1, 4.0, 1.0, 1.0, 0, None, True, [])],
            [SoMessage(None, None, Vector3(0, 2, 0), Quaternion(), Vector3(),
                       1, 1.0, 1.0, 1.0, 0, None, True, []),
             SoMessage(None, None, Vector3(2, 2, 0), Quaternion(), Vector3(),
                       1, 1.0, 1.0, 1.0, 0, None, True, [])]]

        self.assertEqual(bffr.agent_set([bffr.pose_frame], time=self.now), result)

    def test_static_list_angle(self):
        """
        test static_list_angle method
        :return:
        """

        bffr = SoBuffer(view_distance=2.0)

        bffr._own_pos = [
            SoMessage(None, None, Vector3(1, 1, 0), Quaternion(),
                      Vector3(1, 0, 0), 1, 1.0, 1.0, 1.0, 0, None, False, [])]

        bffr._static = {
            'None': [SoMessage(None, None, Vector3(2, 1, 3), Quaternion(),
                               Vector3(), 1, 1.0, 0.1, 1.0, 0, None, False,
                               []),
                     SoMessage(None, None, Vector3(3, 3, 0), Quaternion(),
                               Vector3(), -1, 0.2, 0.5, 1.0, 0, None, False,
                               []),
                     SoMessage(None, None, Vector3(2, 2, 0), Quaternion(),
                               Vector3(), 1, 5.0, 0.2, 1.0, 0, None, False,
                               []),
                     ],
            'Center': [SoMessage(None, None, Vector3(1, 1.8, 0), Quaternion(),
                                 Vector3(), -1, 8.0, 1.0, 1.0, 0, None, False,
                                 []),
                       SoMessage(None, None, Vector3(0, 0, 0), Quaternion(),
                                 Vector3(), -1, 3.0, 0.1, 1.0, 0, None, False,
                                 [])]
        }

        result = [SoMessage(None, None, Vector3(1, 1.8, 0), Quaternion(),
                            Vector3(), -1, 8.0, 1.0, 1.0, 0, None, False, []),
                  SoMessage(None, None, Vector3(0, 0, 0), Quaternion(),
                            Vector3(), -1, 3.0, 0.1, 1.0, 0, None, False, [])]

        self.assertEqual(bffr.static_list_angle(['Center'], np.pi, time=self.now), result)

        self.assertEqual(bffr.static_list_angle(['Center'], np.pi/2, time=self.now),
                         [SoMessage(None, None, Vector3(1, 1.8, 0),
                                    Quaternion(), Vector3(), -1, 8.0, 1.0, 1.0,
                                    0, None, False, [])])

        self.assertEqual(bffr.static_list_angle(['Center'], 0, time=self.now), [])


# run tests - start roscore before running tests
if __name__ == "__main__":
    unittest.main()
