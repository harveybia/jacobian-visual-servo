#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def joint_angles_cb(data):
    global joint_angles_pub
    joint_angles_6 = JointState()
    for i in range(3):
        joint_angles_6.position.extend([data.position[i * 3],
                                        data.position[i * 3 + 2]])
        joint_angles_6.name.extend(['joint_' + repr(i*2), 'joint_' + repr(i*2 + 1)])

    joint_angles_pub.publish(joint_angles_6)

def joint_cmd_cb(data):
    global joint_cmd_pub, passive_joint_states
    cmd_9 = JointState()
    for i in range(3):
        cmd_9.position.append(data.position[0 + 2 * i])
        cmd_9.position.append(passive_joint_states[i])
        cmd_9.position.append(data.position[1 + 2 * i])
        cmd_9.name.extend(['joint_' + repr(i * 3), 'joint_' + repr(i * 3 + 1),
                           'joint_' + repr(i * 3 + 2)])
    joint_cmd_pub.publish(cmd_9)

def passive_joint_cmd_cb(data):
    global passive_joint_states
    assert(len(data.position) == 3)
    for i in range(3):
        passive_joint_states[i] = data.position[i]


if __name__ == "__main__":
    rospy.init_node("snake_6_mapper")
    rospy.Subscriber("/snake_arm/joint_states", JointState, joint_angles_cb)
    rospy.Subscriber("/snake_6/joint_cmd", JointState, joint_cmd_cb)
    rospy.Subscriber("/snake_6/passive_joint_cmd", JointState, passive_joint_cmd_cb)

    joint_angles_pub = rospy.Publisher("/snake_6/joint_states", JointState,
                                       queue_size=10)
    joint_cmd_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    passive_joint_states = [0, 0, 0]

    rospy.spin()