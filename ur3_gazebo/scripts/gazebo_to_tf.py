#!/usr/bin/python
import rospy
import rospkg

from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import tf
import rospy

from ur_control import conversions


class GazeboToTf:
    """ Class to handle ROS-Gazebo model respawn """

    def __init__(self):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.tf_publisher = tf.TransformBroadcaster()

    def callback(self, data):
        for i in range(len(data.name)):
            # get model state of all objects
            self.tf_publisher.sendTransform(conversions.from_point(data.pose[i].position),
                                            conversions.from_quaternion(data.pose[i].orientation),
                                            rospy.Time.now(),
                                            data.name[i],
                                            "world")
if __name__ == '__main__':
    rospy.init_node('gazebo_to_tf')
    g2tf = GazeboToTf()
    rospy.spin()


