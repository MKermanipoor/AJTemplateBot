#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import numpy as np

rviz_pub = None
rviz_messsage_seq = 0
q = np.array([0,0,0], dtype=float)

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

def cos(a):
    return np.cos(a)

def sin(a):
    return np.sin(a)

def publish_on_rviz(rviz_messsage_seq):
    global rviz_pub
    global q

    base_angle = q[0]
    shoulder_angle = q[1]
    elbow_angle = q[2]

    message = JointState()
    message.header.seq = rviz_messsage_seq
    message.header.stamp = rospy.get_rostime()
    message.header.frame_id = ''

    message.name = ['base_link__link_01', 'link_01__link_02', 'link_02__link_03', 'left_gripper_joint', 'right_gripper_joint']
    message.position = [base_angle, shoulder_angle, elbow_angle, 0, 0]
    message.velocity = []
    message.effort = []

    rviz_messsage_seq += 1

    rviz_pub.publish(message)

def get_position(q):
    T01 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.5],
        [0, 0, 0, 1],
    ])

    T12 = np.array([
        [-np.cos(q[0]), np.sin(q[0]), 0, 0],
        [-np.sin(q[0]), -np.cos(q[0]), 0, 0],
        [0, 0, 1, 0.4],
        [0, 0, 0, 1],
    ]).dot(np.array([
        [1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ]))

    T23 = np.array([
        [-np.sin(q[1]), -np.cos(q[1]), 0, 0],
        [np.cos(q[1]), -np.sin(q[1]), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]).dot(np.array([
        [1, 0, 0, 0.8],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]))

    T34 = np.array([
        [np.cos(q[2]), -np.sin(q[2]), 0, 0],
        [np.sin(q[2]), np.cos(q[2]), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]).dot(np.array([
        [1, 0, 0, 0.8],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]))

    return T01.dot(T12).dot(T23).dot(T34).dot(np.array([0, 0, 0, 1]))[:3]

def get_jacoboan(q):

    b, s, e = q

    jacobian = np.zeros((3,3))

    jacobian[0][0] = 0.4 * (cos(b + s) - cos(b - s) + cos(b + s + e) - cos(b - s - e))
    jacobian[0][1] = 0.4 * (cos(b + s) + cos(b - s) + cos(b + s + e) + cos(b - s - e))
    jacobian[0][2] = 0.4 * (cos(b + s + e) + cos(b - s - e))

    jacobian[1][0] = 0.4 * (sin(b + s) - sin(b - s) + sin(b + s + e) - sin(b - s - e))
    jacobian[1][1] = 0.4 * (sin(b + s) + sin(b - s) + sin(b + s + e) + sin(b - s - e))
    jacobian[1][2] = 0.4 * (sin(b + s + e) + sin(b - s - e))
    
    jacobian[2][0] = 0
    jacobian[2][1] = -0.8 * (sin(s) + sin(s + e))
    jacobian[2][2] = -0.8 * (sin(s + e))

    return jacobian

    
def bounding(number):


    return min(0.2, max(-0.2, number))


def listener (data):
    global q

    q[0] = get_base(data)
    target = np.array([data.x, data.y, data.z])

    rate = rospy.Rate(0.3)
    
    for i in range(20):
        now_p = get_position(q)
        d = target - now_p

        dd = d / 5

        jac = get_jacoboan(q)
        dq = np.linalg.pinv(jac).dot(dd)

        rospy.loginfo("[%f\t,%f\t,%f]", dq[0], dq[1], dq[2])
        dq[1] = min(0.2, max(-0.2, dq[1]))
        dq[2] = min(0.2, max(-0.2, dq[2]))
        rospy.loginfo("[%f\t,%f\t,%f]", dq[0], dq[1], dq[2])

        q = q + dq
        rospy.loginfo("[%f\t,%f\t,%f]", q[0], q[1], q[2])

        q[1] = min(0.5, max(0, q[1]))
        q[2] = min(0.75, max(0, q[2]))
        rospy.loginfo("[%f\t,%f\t,%f]", q[0], q[1], q[2])

        rospy.loginfo("\n\n")
        rate.sleep()


def init():
    global rviz_pub

    rospy.init_node('Inverse', anonymous=True)
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