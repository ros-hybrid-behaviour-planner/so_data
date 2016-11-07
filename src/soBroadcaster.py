#!/usr/bin/env python 

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

   
#define here input as soMessage or sth like that and check if publisher has to be somehow global made or so 
    def sendSoData(self, message):
        if isinstance(message, soMessage):
	    rospy.loginfo(message)
            self._pub.publish(message)
	else:
            rospy.loginfo("Wrong message type") 

#test broadcaster - only for testing purposes   
if __name__ == '__main__':
    rospy.init_node('soBroadcaster', anonymous=True)
    try:
        talk = SoBroadcaster()
        msg = soMessage()
        msg.data = "test" 
	talk.sendSoData(msg)
	strg = "hallo"
	talk.sendSoData(strg)
	rospy.sleep(5.0)
        msg.data = "test" 
	talk.sendSoData(msg)
	rospy.sleep(5.0) 
	talk.sendSoData(msg)
	rospy.sleep(5.0) 
	talk.sendSoData(msg)
        rospy.sleep(5.0) 
	talk.sendSoData(msg)
    except rospy.ROSInterruptException: pass 

    rospy.spin()
