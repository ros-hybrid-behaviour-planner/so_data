#! /usr/bin/env python2
"""
Created on 11.01.2017

@author: kaiser
"""

import rospy
from so_data.soBroadcaster import SoBroadcaster
from so_data.msg import soMessage
from geometry_msgs.msg import Vector3
from abc import ABCMeta, abstractmethod

class TopicGradientTf(object):
    """
    abstract base class to transform values of a topic to a soMessage
    """
    __metaclass__ = ABCMeta

    def __init__(self, topic, message_type, id, p=Vector3(), attraction=-1,
                 diffusion=1.0, goal_radius=1.0, ev_factor=1, ev_time=0,
                 angle_x=0, angle_y=0, direction=Vector3(), moving=True,
                 payload=[]):
        rospy.Subscriber(topic, message_type, self.callback)

        self._id = id
        self.p = p
        self.attraction = attraction
        self.diffusion = diffusion
        self.goal_radius = goal_radius
        self.ev_factor = ev_factor
        self.ev_time = ev_time
        self.angle_x = angle_x
        self.angle_y = angle_y
        self.direction = direction
        self.moving = moving
        self.payload = payload

        self._broadcast = SoBroadcaster()

    @abstractmethod
    def callback(self, msg):
        """
        abstract method: should assign received data to at least one
        soMessage attribute and call self._broadcast.send_data(self.msg)

        template:

        msg = soMessage()

        # gradient parameters by class variables
        msg.header.frame_id = self._id
        msg.header.stamp = rospy.Time.now()  # current time
        msg.p = self.p
        msg.attraction = self.attraction
        msg.diffusion = self.diffusion
        msg.goal_radius = self.goal_radius
        msg.ev_factor = self.ev_factor
        msg.ev_time = self.ev_time
        msg.angle_x = self.angle_x
        msg.angle_y = self.angle_y
        msg.direction = self.direction
        msg.moving = self.moving
        msg.payload = self.payload

        self._broadcast.send_data(msg)


        :return:
        """
        pass


class PoseTopicGradientTf(TopicGradientTf):
    """
    class to transform pose topic (geometry_msgs.msg.Pose) to a soMessage
    """
    def __init__(self, topic, message_type, id, **kwargs):
        super(PoseTopicGradientTf, self).__init__(topic, message_type, id,
                                                  **kwargs)

    def callback(self, pose):
        msg = soMessage()

        # update by received msg
        msg.p.x = pose.position.x
        msg.p.y = pose.position.y
        msg.p.z = pose.position.z

        # set current time
        msg.header.stamp = rospy.Time.now()

        # gradient parameters by class variables
        msg.header.frame_id = self._id
        msg.attraction = self.attraction
        msg.diffusion = self.diffusion
        msg.goal_radius = self.goal_radius
        msg.ev_factor = self.ev_factor
        msg.ev_time = self.ev_time
        msg.angle_x = self.angle_x
        msg.angle_y = self.angle_y
        msg.direction = self.direction
        msg.moving = self.moving
        msg.payload = self.payload

        self._broadcast.send_data(msg)



# ROS Node to spread gradients - still #TODO
if __name__ == '__main__':
    """
    Node to convert topics to gradients
    """

    rospy.init_node(rospy.get_param("~nodeName", 'poseGradientTf'))
    rate = rospy.Rate(rospy.get_param("~frequency", 1))

    # alle parameter fuer transform so machen?!?

    self.tf = PoseTopicGradientTf(topic, message_type)


    while not rospy.is_shutdown():
        rate.sleep()
