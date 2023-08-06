#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import JointState
import math
from spatialmath.base import *
from spatialmath import SE3
import spatialmath.base.symbolic as sym
import numpy as np
import roboticstoolbox as rtb
 

def joint_state_callback(data):
    joint_names = data.name
    joint_positions = data.position
    joint_name = "arm_01_joint_04"
    for i in range(len(joint_names)):
        if joint_names[i] == joint_name:
            print("Joint Position: %f" % joint_positions[i])



# Link_1 = rtb.DHLink(0.5, math.pi/2, 0, 0)
# Link_2 = rtb.DHLink(0,    0,   0, 0.4)
# Link_3 = rtb.DHLink(0,    0,   0, 0.4)
# Link_4 = 
# Link_5 = 
# Link_6 
# mario = rtb.DHRobot([Link_1 ,Link_2,Link_3])
# mario

# Selection a point to get inverse kinematics solution as angles 
# print("point-> x: %2.2f ,y: %2.2f ,z: %2.2f" %(1.5,2.5,2.3) )
# point = SE3( 0.4453 , 0.5307 , 0.9  )
# point_sol = bazu_robot.ikine_LM(point) 
# print(point_sol)

if __name__ == '__main__':
    rospy.init_node('joint_state_listener')
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.spin()