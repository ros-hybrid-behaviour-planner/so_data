''' 
Created on 07.11.2016

@author: kaiser
'''

import rospy 

class SoBuffer():
    '''
    This class is the buffer for the send data
    '''
    def __init__(self, duration):
        #how long the data is valid 
        self._duration = duration   
        #store data      
        self.data = []

    def storeData(self, msg):
        self.data.append(msg)

    def getData(self):
        return self.data

    def getLastGradient(self):
        if self.data:
            return self.data[-1]