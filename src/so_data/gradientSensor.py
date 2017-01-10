"""
Created on 03.11.2016

@author: kaiser
"""

import rospy 
import soBuffer
from behaviour_components.sensors import SimpleTopicSensor


class GradientSensor(SimpleTopicSensor):
    """
    Gradient Sensor storing received gradient messages and calculating values for the activation calculation
    """
    def __init__(self, name, topic=None, message_type=None, initial_value=None, create_log=False,
                 sensor_type='gradient', sensor_buffer=None):
        """
        :param name: sensor name
        :param topic: pose topic of agent
        :param message_type: message type of pose topic
        :param initial_value: initial value of pose topic / sensor
        :param create_log: True/False
        :param sensor_type: defines type of sensor
                            options:
                                * gradient: returns movement vector
                                * bool: returns True/False based on if nearest attractive gradient is reached
        :param sensor_buffer: soBuffer object
        """
        self._buffer = sensor_buffer
        self._sensor_type = sensor_type

        super(GradientSensor, self).__init__(topic=topic, name=name, message_type=message_type,
                                             initial_value=initial_value, create_log=create_log)

    def subscription_callback(self, msg):
        """
        sets gradient sensor value based on own position
        :param msg: received message, own position
        :return:
        """
        if self._sensor_type == 'gradient' or not self._sensor_type:
            tmp = self._buffer.get_current_gradient(msg.position)
            self.update(tmp)
        elif self._sensor_type == 'bool_attractive':
            tmp = self._buffer.get_goal_reached(msg.position)
            self.update(tmp)
        elif self._sensor_type == 'bool_all':
            tmp = self._buffer.get_no_potential(msg.position)
            self.update(tmp)

        rospy.logdebug("%s received sensor message: %s of type %s", self._name, self._value, type(self._value))
        if self._iShouldCreateLog:
            self._logFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._value))
            self._logFile.flush()

