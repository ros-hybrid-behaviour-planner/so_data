#!/usr/bin/env python 

''' 
Created on 03.11.2016

@author: kaiser
'''

import rospy 
import soBuffer 
from so_data.msg import * 

class SoListener():
    '''
    This class is the liestener to request and receive data from 
    the soData topic 
    '''
    def __init__(self):
        '''
        Constructor
        Creates subscriber to receive data from soData 
        '''
        self.buffer = soBuffer.SoBuffer(2.0)

    def run(self):
        self._sub = rospy.Subscriber("soData", soMessage, self.callback)
        rospy.spin()
    
    #integrate buffer in callback function 
    def callback(self, data):
        self.buffer.storeData(data) 
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data) #print everything in future 

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True) 
    listener = SoListener()
    listener.run()
