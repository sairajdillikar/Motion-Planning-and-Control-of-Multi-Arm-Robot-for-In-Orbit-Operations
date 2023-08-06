#!/usr/bin/env python3
# import rospy
# from sensor_msgs.msg import JointState

# def callback(data):
#     # Print out the position, velocity, and effort of each joint
#     for i in range(len(data.name)):
#         joint_name = data.name[i]
#         joint_position = data.position[i]
#         joint_velocity = data.velocity[i]
#         joint_effort = data.effort[i]
#         print("Joint {}: position={}, velocity={}, effort={}".format(
#             joint_name, joint_position, joint_velocity, joint_effort))

 

# def joint_state_subscriber():
#     rospy.init_node('joint_state_subscriber', anonymous=True)
#     rospy.Subscriber("joint_states", JointState, callback)
#     rospy.spin()

 

# if __name__ == '__main__':
#     joint_state_subscriber()

 

import rospy
from sensor_msgs.msg import JointState

 

def joint_state_callback(data):
    joint_names = data.name
    joint_positions = data.position
    joint_name = "arm_01_joint_04"
    for i in range(len(joint_names)):
        if joint_names[i] == joint_name:
            print("Joint Position: %f" % joint_positions[i])



if __name__ == '__main__':
    rospy.init_node('joint_state_listener')
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.spin()