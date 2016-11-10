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
    def __init__(self):
        '''
        Constructor
        Creates publisher to broadcast data to soData as an soMessage 
        '''
	self._pub = rospy.Publisher('soData', soMessage, queue_size = 100)


    def sendSoData(self, message):
        '''
        sends data to soData topic
        '''
        if isinstance(message, soMessage):
            self._pub.publish(message)
        else:
            rospy.loginfo("Wrong message type")