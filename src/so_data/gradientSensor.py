"""
Created on 03.11.2016

@author: kaiser
"""

import rospy 
import soBuffer
from behaviour_components.sensors import Sensor


class GradientSensor(Sensor):
    """
    Gradient Sensor storing received gradient messages and calculating values
    for the activation calculation
    """
    def __init__(self, name, initial_value=None, sensor_type='gradient',
                 sensor_buffer=None):
        """
        :param name: sensor name
        :param initial_value: initial value of pose topic / sensor
        :param sensor_type: defines type of sensor
                            options:
                                * gradient: returns movement vector
                                * bool: returns True/False based on if nearest
                                attractive gradient is reached
        :param sensor_buffer: soBuffer object
        """
        self._buffer = sensor_buffer
        self._sensor_type = sensor_type

        super(GradientSensor, self).__init__(name=name,
                                             initial_value=initial_value)

    def sync(self):
        """
        syncs sensor value setting self._value
        :return:
        """
        if self._sensor_type == 'gradient' or not self._sensor_type:
            self._value = self._buffer.get_current_gradient()
        elif self._sensor_type == 'distance':
            self._value = self._buffer.get_attractive_distance()
        elif self._sensor_type == 'bool_reached_attractive':
            self._value = self._buffer.get_goal_reached()
        elif self._sensor_type == 'bool_all':
            self._value = self._buffer.get_no_potential()
        elif self._sensor_type == 'neighbors':
            self._value = self._buffer.get_neighbors_bool()
        elif self._sensor_type == 'views_attractive':
            self._value = self._buffer.get_attractive_gradients_view()

        return self._value
