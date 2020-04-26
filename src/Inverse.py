#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import numpy as np

base_pub = None
rviz_pub = None
base_angle = 0.0

rviz_messsage_seq = 0

def get_base(position):
    if position.x == 0:
        if position.y > 0 :
           return np.pi /2 
        else:
            return -np.pi /2 

    theta = np.arctan(position.y / position.x)

    if position.x < 0 :
        if position.y >= 0:
            theta += np.pi
        else:
            theta -= np.pi
    
    return theta

def publish_on_rviz(rviz_messsage_seq):
    global rviz_pub
    global base_angle

    message = JointState()
    message.header.seq = rviz_messsage_seq
    message.header.stamp = rospy.get_rostime()
    message.header.frame_id = ''

    message.name = ['base_link__link_01', 'link_01__link_02', 'link_02__link_03', 'left_gripper_joint', 'right_gripper_joint']
    message.position = [base_angle, 0, 0, 0, 0]
    message.velocity = []
    message.effort = []

    rviz_messsage_seq += 1

    rviz_pub.publish(message)




def listener (data):
    global base_pub
    global base_angle

    data = data

    base_angle = get_base(data)

    base_pub.publish(base_angle)



def init():
    global base_pub
    global rviz_pub

    rospy.init_node('Inverse', anonymous=True)

    
    base_pub = rospy.Publisher('/AJBot/base_rotation_controller/command', Float64, queue_size=10)
    rviz_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rospy.Subscriber('Position', Point, listener)

    rate = rospy.Rate(10)
    rviz_messsage_seq = 0
    while not rospy.is_shutdown():
        rviz_messsage_seq += 1
        publish_on_rviz(rviz_messsage_seq)
        rate.sleep()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass