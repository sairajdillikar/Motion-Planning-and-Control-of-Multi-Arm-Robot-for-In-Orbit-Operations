#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Float64
# import math

# def talker():
#     pub_joint4 = rospy.Publisher('/arm_01_joint_04_position_controller/command', Float64, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(50) 
#     start = 0.0
#     while not rospy.is_shutdown():    
#         pos = math.sin(start)        
#         rospy.loginfo("started publishing")
#         pub_joint4.publish(pos)
#         start+=0.025

#     rate.sleep()


# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass









# import rospy
# from std_msgs.msg import Float64
# import math

# JOINT_NAMES = ['arm_01_joint_01', 'arm_01_joint_02', 'arm_01_joint_03', 'arm_01_joint_04', 'arm_01_joint_05', 'arm_01_joint_06', 'arm_02_joint_01', 'arm_02_joint_02', 'arm_02_joint_03', 'arm_02_joint_04', 'arm_02_joint_05', 'arm_02_joint_06', 'arm_03_joint_01', 'arm_03_joint_02', 'arm_03_joint_03', 'arm_03_joint_04', 'arm_03_joint_05', 'arm_03_joint_06']

# def talker():
#     pubs = {}
#     for joint_name in JOINT_NAMES:
#         pub = rospy.Publisher('/' + joint_name + '_position_controller/command', Float64, queue_size=10)
#         pubs[joint_name] = pub

#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(50)
#     start = 0.0
#     while not rospy.is_shutdown():
#         for joint_name in JOINT_NAMES:
#             pos = math.sin(start)
#             rospy.loginfo("Publishing to " + joint_name + " joint")
#             pubs[joint_name].publish(pos)
#         start += 0.025
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass











import rospy
from std_msgs.msg import Float64
import math

STANDING_ARMS = ['arm_01', 'arm_02', 'arm_03'] # The two arms used for standing up
JOINT_NAMES = ['joint_01', 'joint_02', 'joint_03', 'joint_04', 'joint_05', 'joint_06'] # Joint names for each arm

def talker():
    pubs = {}
    for arm in STANDING_ARMS:
        for joint_name in JOINT_NAMES:
            pub = rospy.Publisher('/' + arm + '_' + joint_name + '_position_controller/command', Float64, queue_size=10)
            pubs[arm + '_' + joint_name] = pub

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50)
    start = 0.0
    while not rospy.is_shutdown():
        for arm in STANDING_ARMS:
            for joint_name in JOINT_NAMES:
                pos = math.sin(start)
                rospy.loginfo("Publishing to " + arm + " " + joint_name + " joint")
                pubs[arm + '_' + joint_name].publish(pos)
        start += 0.025
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass













# import rospy
# from std_msgs.msg import Float64
# import math

# # STANDING_ARMS = ['arm_01', 'arm_02'] # The two arms used for standing up
# # JOINT_NAMES = ['joint_01', 'joint_02', 'joint_03', 'joint_04', 'joint_05', 'joint_06'] # Joint names for each arm

# def talker():
#     pubs = {}
#     # for arm in STANDING_ARMS:
#     #     for joint_name in JOINT_NAMES:
#     #         pub = rospy.Publisher('/' + arm + '_' + joint_name + '_position_controller/command', Float64, queue_size=10)
#     #         pubs[arm + '_' + joint_name] = pub

#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(50)
#     start = 0.0
#     while not rospy.is_shutdown():
        
#         pub_arm1_joint4 = rospy.Publisher('/arm_01_joint_04_position_controller/command', Float64, queue_size=10)
#         pub_arm1_joint5 = rospy.Publisher('/arm_01_joint_05_position_controller/command', Float64, queue_size=10)
#         pub_arm2_joint4 = rospy.Publisher('/arm_02_joint_04_position_controller/command', Float64, queue_size=10)
#         pub_arm2_joint5 = rospy.Publisher('/arm_02_joint_05_position_controller/command', Float64, queue_size=10)
#         pub_arm3_joint4 = rospy.Publisher('/arm_03_joint_04_position_controller/command', Float64, queue_size=10)
#         pub_arm3_joint5 = rospy.Publisher('/arm_03_joint_05_position_controller/command', Float64, queue_size=10)

#         # rospy.init_node('talker', anonymous=True)

#         # Publish angle values of 0.7854 radians for joint 4 and joint 5 of both arm 1 and arm 2 only once
#         angle = math.pi / 4
#         pub14 = pub_arm1_joint4.publish(angle)
#         pub15 = pub_arm1_joint5.publish(angle)
#         pub24 = pub_arm2_joint4.publish(angle)
#         pub25 = pub_arm2_joint5.publish(angle)
#         pub34 = pub_arm3_joint4.publish(angle)
#         pub35 = pub_arm3_joint5.publish(angle)
#         angle = angle + math.pi / 4
#         rospy.loginfo("Published angle values for joint 4 and joint 5 of arm 1 arm 2 and arm 3") 



#         # for arm in STANDING_ARMS:
#         #     for joint_name in JOINT_NAMES:
#         #         pos = math.sin(start)
#         #         rospy.loginfo("Publishing to " + arm + " " + joint_name + " joint")
#         #         pubs[arm + '_' + joint_name].publish(pos)
#         # start += 0.025
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass











# import rospy
# from std_msgs.msg import Float64
# import math

# # STANDING_ARMS = ['arm_01', 'arm_02'] # The two arms used for standing up
# # JOINT_NAMES = ['joint_01', 'joint_02', 'joint_03', 'joint_04', 'joint_05', 'joint_06'] # Joint names for each arm

# def talker():
#     pubs = {}
#     # for arm in STANDING_ARMS:
#     #     for joint_name in JOINT_NAMES:
#     #         pub = rospy.Publisher('/' + arm + '_' + joint_name + '_position_controller/command', Float64, queue_size=10)
#     #         pubs[arm + '_' + joint_name] = pub

#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(50)
#     start = 0.0
#     pub_arm1_joint4 = rospy.Publisher('/arm_01_joint_04_position_controller/command', Float64, queue_size=10)
#     pub_arm1_joint5 = rospy.Publisher('/arm_01_joint_05_position_controller/command', Float64, queue_size=10)
#     pub_arm2_joint4 = rospy.Publisher('/arm_02_joint_04_position_controller/command', Float64, queue_size=10)
#     pub_arm2_joint5 = rospy.Publisher('/arm_02_joint_05_position_controller/command', Float64, queue_size=10)
#     pub_arm3_joint4 = rospy.Publisher('/arm_03_joint_04_position_controller/command', Float64, queue_size=10)
#     pub_arm3_joint5 = rospy.Publisher('/arm_03_joint_05_position_controller/command', Float64, queue_size=10)

#     while not rospy.is_shutdown():
        
        
#         # rospy.init_node('talker', anonymous=True)

#         # Publish angle values of 0.7854 radians for joint 4 and joint 5 of both arm 1 and arm 2 only once
#         angle = math.pi / 4
#         pub14 = pub_arm1_joint4.publish(angle)
#         pub15 = pub_arm1_joint5.publish(angle)
#         pub24 = pub_arm2_joint4.publish(angle)
#         pub25 = pub_arm2_joint5.publish(angle)
#         pub34 = pub_arm3_joint4.publish(angle)
#         pub35 = pub_arm3_joint5.publish(angle)
#         # angle = angle + math.pi / 4
#         rospy.loginfo("Published angle values for joint 4 and joint 5 of arm 1 arm 2 and arm 3") 



#         # for arm in STANDING_ARMS:
#         #     for joint_name in JOINT_NAMES:
#         #         pos = math.sin(start)
#         #         rospy.loginfo("Publishing to " + arm + " " + joint_name + " joint")
#         #         pubs[arm + '_' + joint_name].publish(pos)
#         # start += 0.025
#         rate.sleep()


# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass
