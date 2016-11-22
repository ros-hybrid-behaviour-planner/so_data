'''
Created on 03.11.2016

@author: kaiser
'''

import rospy 
import soBuffer
from so_data.msg import *
import numpy as np
from behaviour_components.sensors import SimpleTopicSensor

class SoListener():
    '''
    This class is the listener to request and receive data from
    the soData topic 
    '''
    def __init__(self, pose_sensor):
        '''
        Constructor
        Creates subscriber to receive data from soData 
        '''
        self.buffer = soBuffer.SoBuffer(pose_sensor)
        self._sub = rospy.Subscriber("soData", soMessage, self.callback)

    def callback(self, msg):
        '''
        :param data: message received from soData topic
        :return:
        '''
        #rospy.loginfo(msg)
        self.buffer.store_data(msg)

    def print_data(self):
        rospy.loginfo(self.buffer.get_data())


    @property
    def gradient(self):
        return self.buffer.get_current_gradient()



class GradientSensor(SimpleTopicSensor):
    """
    "PassThrough" because the sensor just forwards the received msg
    """
    def __init__(self, name, pose_sensor, topic="/soData", message_type = None,  initial_value = None, create_log = False):
        super(GradientSensor, self).__init__(topic=topic, name = name, message_type = message_type, initial_value = initial_value, create_log=create_log)

        self.buffer = soBuffer.SoBuffer(pose_sensor)

    def update(self, newValue):
        '''
        This method is to refresh the _value.
        '''
        self.buffer.store_data(newValue)
        self._latestValue = self.buffer.get_current_gradient()

    def subscription_callback(self, msg):
        self.update(msg)
        rospy.logdebug("%s received sensor message: %s of type %s", self._name, self._value, type(self._value))
        if self._iShouldCreateLog:
            self._logFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._value))
            self._logFile.flush()
