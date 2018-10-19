#! /usr/bin/env python2
"""
Created on 18.01.2017

@author: kaiser

Module to transform pose topic messages (geometry.msgs.pose) to SoMessages
"""

import copy
import rospy
import geometry_msgs.msg
from so_data.topicgradienttf import TopicGradientTf


class PoseTopicGradientTf(TopicGradientTf):
    """
    class to transform pose topic (geometry_msgs.msg.Pose) to a soMessage for
    use in a ROS node
    """
    def __init__(self, topic, frame, id, message_type=geometry_msgs.msg.Pose,
                 **kwargs):
        """
        initialization
        :param topic: topic to subscribe to
        :param frame: header frame id
        :param id: soMessage id used for parent frame
        :param message_type: message type of topic
        :param kwargs: keyword arguments to specify other params of SoMessage
        """
        super(PoseTopicGradientTf, self).__init__(topic, frame, id,
                                                  message_type, **kwargs)

    def callback(self, pose):
        """
        implementation of callback, assigns pose msg to gradient center
        :param pose:geometry pose message of pose topic
        :return:
        """
        # deepcopy needed, otherwise leading to errors
        msg = copy.deepcopy(self.create_msg())

        # update data based on received information
        msg.p.x = pose.position.x
        msg.p.y = pose.position.y
        msg.p.z = pose.position.z

        msg.q = pose.orientation

        # update self._current_gradient
        self._current_msg = msg


class CallbackPoseTopicGradientTf(PoseTopicGradientTf):
    """
    class to transform pose topic to soMessage & send it within callback
    """
    def __init__(self, topic, frame, id, **kwargs):
        """
        initialization
        :param topic: topic to subscribe to
        :param frame: header frame id
        :param id: soMessage id used for parent frame
        :param kwargs: keyword arguments to set other params of TopicGradientTf
        """
        super(CallbackPoseTopicGradientTf, self).__init__(topic, frame, id, **kwargs)

    def callback(self, pose):
        """
        implementation of callback, assigns pose msg to gradient center
        & spreads gradient
        :param pose: geometry pose message of pose topic
        :return:
        """
        # create gradient
        super(CallbackPoseTopicGradientTf, self).callback(pose)

        # send gradient
        self.send()


# ROS Node to spread gradients
# sample setup with default values + pose specified in topic
if __name__ == '__main__':
    """
    Node to convert pose topics to gradients
    """
    rospy.init_node("transform_pose")

    rate = rospy.Rate(rospy.get_param("~frequency", 1))

    tf = PoseTopicGradientTf(rospy.get_param("~topic", 'turtle_1_pose'),
                             rospy.get_param("~id", 'robot1'))

    while not rospy.is_shutdown():
        # send message
        tf.send()
        # sleep
        rate.sleep()
