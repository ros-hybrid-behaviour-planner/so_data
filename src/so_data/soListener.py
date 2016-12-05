'''
Created on 03.11.2016

@author: kaiser
'''

import rospy 
import soBuffer
from behaviour_components.sensors import SimpleTopicSensor


class GradientSensor(SimpleTopicSensor):
    """
    "PassThrough" because the sensor just forwards the received msg
    """
    def __init__(self, name, id='', topic=None, message_type=None,  initial_value=None, create_log = False):
        # buffer to store and handle gradient data
        self._buffer = soBuffer.SoBuffer(id=id)

        super(GradientSensor, self).__init__(topic=topic, name = name, message_type = message_type, initial_value = initial_value, create_log=create_log)


    def subscription_callback(self, msg):
        tmp = self._buffer.get_current_gradient(msg.position)
        self.update(tmp)
        rospy.logdebug("%s received sensor message: %s of type %s", self._name, self._value, type(self._value))
        if self._iShouldCreateLog:
            self._logFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._value))
            self._logFile.flush()

