#!/usr/bin/env python

import roslib
roslib.load_manifest('AJTemplateBot')
import rospy
import tf
import turtlesim.msg
from geometry_msgs.msg import Point

position = Point(0,0,0)
def listener (p):
    global position
    position = p

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.Subscriber('Position', Point, listener)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        br.sendTransform(
            (position.x,position.y,position.z),
            (0,0,0,1),
            rospy.Time.now(),
            "target",
            "base_link"
        )
        rate.sleep()

