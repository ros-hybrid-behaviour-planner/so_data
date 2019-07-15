#! /usr/bin/env python2
"""
Created on 18.01.2017

@author: kaiser

Module to transform pose topic messages (geometry.msgs.poseStamped) to
SoMessages
"""

import copy
import rospy
import geometry_msgs.msg
from so_data.topicgradienttf import TopicGradientTf


class PoseStampedTopicGradientTf(TopicGradientTf):
    """
    class to transform pose topic (geometry_msgs.msg.PoseStamped) to a
    soMessage for use in a ROS node
    """
    def __init__(self, topic,frame, id, message_type=geometry_msgs.msg.PoseStamped,
                 **kwargs):
        """
        initialization
        :param topic: topic to subscribe to
        :param id: soMessage id used for parent frame
        :param message_type: message type of topic
        :param kwargs: keyword arguments to specify other params of SoMessage
        """
        super(PoseStampedTopicGradientTf, self).__init__(topic, frame, id,
                                                         message_type,
                                                         **kwargs)

    def callback(self, pose):
        """
        implementation of callback, assigns pose msg to gradient center
        sets current_msg variable
        :param pose:geometry pose message of pose topic
        :return:
        """
        # deepcopy needed, otherwise leading to errors
        msg = copy.deepcopy(self.create_msg())

        # update data based on received information
        msg.p.x = pose.pose.position.x
        msg.p.y = pose.pose.position.y
        msg.p.z = pose.pose.position.z

        msg.q = pose.pose.orientation

        # update self._current_gradient
        self._current_msg = msg


class CallbackPoseStampedTopicGradientTf(PoseStampedTopicGradientTf):
    """
    class to transform poseStamped topic to soMessage & send it within callback
    """
    def __init__(self, topic,frame, id, **kwargs):
        """
        initialization
        :param topic: topic to subscribe to
        :param id: soMessage id used for parent frame
        :param kwargs: keyword arguments to specify other params of SoMessage
        """
        super(CallbackPoseStampedTopicGradientTf, self).__init__(topic,frame,
                                                                 id, **kwargs)

    def callback(self, pose):
        """
        implementation of callback, assigns pose msg to gradient center
        & sends gradient
        :param pose: geometry pose message of pose topic
        :return:
        """
        # create gradient
        super(CallbackPoseStampedTopicGradientTf, self).callback(pose)

        # send gradient
        self.send()


# ROS Node to spread gradients
# sample setup with default values + poseStamped specified in topic
if __name__ == '__main__':
    """
    Node to convert topics to gradients
    """
    rospy.init_node("transform_poseStamped")

    rate = rospy.Rate(rospy.get_param("~frequency", 1))

    tf = PoseStampedTopicGradientTf(rospy.get_param("~topic", 'turtle_1_pose'),
                                    rospy.get_param("~id", 'robot1'))

    while not rospy.is_shutdown():
        # send message
        tf.send()
        # sleep
        rate.sleep()
