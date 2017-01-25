#! /usr/bin/env python2
"""
Created on 18.01.2017

@author: kaiser
"""

from so_data.topicGradientTf import TopicGradientTf
import copy
import rospy
import geometry_msgs.msg
from diagnostic_msgs.msg import KeyValue
import numpy as np
import tf.transformations


class PoseTopicGradientTf(TopicGradientTf):
    """
    class to transform pose topic (geometry_msgs.msg.Pose) to a soMessage for
    use in a ROS node
    """
    def __init__(self, topic, message_type, id, **kwargs):
        super(PoseTopicGradientTf, self).__init__(topic, message_type, id,
                                                  **kwargs)

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

        # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
        #     pose.orientation.x,
        #     pose.orientation.y,
        #     pose.orientation.z,
        #     pose.orientation.w])
        # pld = KeyValue('yaw', "%.9f" % yaw)
        # msg.payload.append(pld)
        # msg.payload.append(KeyValue('pitch', "%.9f" % pitch))
        # msg.payload.append(KeyValue('roll', "%.9f" % roll))

        # update self._current_gradient
        self._current_msg = msg


class CallbackPoseTopicGradientTf(PoseTopicGradientTf):
    """
    class to transform pose topic to soMessage & send it within callback
    """
    def __init__(self, topic, message_type, id, **kwargs):
        super(CallbackPoseTopicGradientTf, self).__init__(topic, message_type,
                                                          id, **kwargs)

    def callback(self, pose):
        """
        implementation of callback, assigns pose msg to gradient center
        & sends gradient
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
    Node to convert topics to gradients
    """
    rospy.init_node("transform_")

    rate = rospy.Rate(rospy.get_param("~frequency", 1))

    tf = PoseTopicGradientTf(rospy.get_param("~topic", 'turtle_1_pose'),
                             geometry_msgs.msg.Pose,
                             rospy.get_param("~id", 'robot1'))

    while not rospy.is_shutdown():
        # send message
        tf.send()
        # sleep
        rate.sleep()
