'''
Created on 03.11.2016

@author: kaiser
'''
import rospy 
from so_data.msg import * 

class SoBroadcaster():
    '''
    This class is the broadcaster to send data from one robot to the data space 
    '''
    def __init__(self, time=0):
        '''
        Constructor
        Creates publisher to broadcast data to soData as an soMessage 
        '''
        self._previous = rospy.Time.now()
        self._desired = time
        self._pub = rospy.Publisher('soData', soMessage, queue_size = 1)


    def send_data(self, message):
        '''
        sends data to soData topic
        '''

        if isinstance(message, soMessage):
            #to send data only in certain time frames (does not work perfectly) ToDO
            if rospy.Time.now() - self._previous >= rospy.Duration(self._desired):
                self._pub.publish(message)
                self._previous = rospy.Time.now()
        else:
            rospy.loginfo("Wrong message type")

