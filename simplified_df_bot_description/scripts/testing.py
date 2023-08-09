#!/usr/bin/env python3





import rospy
import math
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose

rospy.init_node('moveit_demo')

robot = RobotCommander()
rocket_group = MoveGroupCommander("Arm_1")
groot_group = MoveGroupCommander("Arm_2")

rocket_waypoints = []
groot_waypoints = []

# Define Circular Path for Rocket Arm
radius = 0.7
center_x1 = -1.4
center_y1 = 0
circle_height = 0.3
num_circle_points = 100

for i in range(num_circle_points):
    angle = 2 * math.pi * i / num_circle_points
    poser = Pose()
    poser.position.x = center_x1 + (radius * math.cos(angle))
    poser.position.y = center_y1 + (radius * math.sin(angle))
    poser.position.z = circle_height
    poser.orientation.w = 1.0
    rocket_waypoints.append(poser)
    # rocket_group.set_pose_target(poser)
    # rocket_success = rocket_group.go(wait=True)

# Define Square Path for Groot Arm
center_x2 = 0.5
center_y2 = 1
square_side = 0.4
square_height = 0.3
num_square_points = 4

for i in range(num_square_points):
    poseg = Pose()
    if i == 0:  # Bottom-Left
        poseg.position.x = center_x2 - square_side / 2
        poseg.position.y = center_y2 - square_side / 2
    elif i == 1:  # Bottom-Right
        poseg.position.x = center_x2 + square_side / 2
        poseg.position.y = center_y2 - square_side / 2
    elif i == 2:  # Top-Right
        poseg.position.x = center_x2 + square_side / 2
        poseg.position.y = center_y2 + square_side / 2
    elif i == 3:  # Top-Left
        poseg.position.x = center_x2 - square_side / 2
        poseg.position.y = center_y2 + square_side / 2
    poseg.position.z = square_height
    poseg.orientation.w = 1.0
    groot_waypoints.append(poseg)
    # groot_group.set_pose_target(poseg)
    # groot_success = groot_group.go(wait=True)

# Set the waypoints for both arms
rocket_group.set_pose_targets(rocket_waypoints)
groot_group.set_pose_targets(groot_waypoints)

# # Plan and execute the trajectories
rocket_success = rocket_group.go(wait=True)
groot_success = groot_group.go(wait=True)

if rocket_success and groot_success:
    rospy.loginfo("Both trajectories executed successfully")
else:
    rospy.loginfo("Error occurred while executing trajectories")

rospy.sleep(1)










# import rospy
# import math
# from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
# from geometry_msgs.msg import Pose
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import random

# rospy.init_node('moveit_demo')

# robot = RobotCommander()
# arm_group = MoveGroupCommander("Arm_1")

# # Set planning parameters
# arm_group.set_planning_time(5.0)  # Maximum allowed time for planning
# arm_group.set_num_planning_attempts(10)  # Number of planning attempts

# # Define the target pose for the arm
# target_pose = Pose()
# target_pose.position.x = -0.5
# target_pose.position.y = 0
# target_pose.position.z = 0.8

# # target_pose.orientation.x = 0.30
# # target_pose.orientation.y = 0.30
# # target_pose.orientation.z = 0.20
# # target_pose.orientation.w = 1

# # Euler angles (in radians)
# roll = 0.0
# pitch = math.pi / 2 
# yaw = 0 

# # Convert Euler angles to quaternion
# quaternion = quaternion_from_euler(roll, pitch, yaw)

# target_pose.orientation.x = quaternion[0]
# target_pose.orientation.y = quaternion[1]
# target_pose.orientation.z = quaternion[2]
# target_pose.orientation.w = quaternion[3]


# # Set the target pose for the arm
# arm_group.set_pose_target(target_pose)

# # Plan and execute the trajectory
# success = arm_group.go(wait=True)

# # Get the current pose
# current_pose = arm_group.get_current_pose().pose

# # Print the current pose
# rospy.loginfo("Current Pose:")
# rospy.loginfo("Position: x={}, y={}, z={}".format(current_pose.position.x,
#                                                   current_pose.position.y,
#                                                   current_pose.position.z))
# rospy.loginfo("Orientation: x={}, y={}, z={}, w={}".format(current_pose.orientation.x,
#                                                            current_pose.orientation.y,
#                                                            current_pose.orientation.z,
#                                                            current_pose.orientation.w))

# if success:
#     rospy.loginfo("Arm trajectory executed successfully")
# else:
#     rospy.loginfo("Error occurred while executing the arm trajectory")

# rospy.sleep(1)















# import rospy
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64
# from std_srvs.srv import Empty
# import math
# from control_msgs.srv import (QueryTrajectoryState, QueryTrajectoryStateRequest, QueryTrajectoryStateResponse)
# import time

# import sys
# import copy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg

# from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
# from copy import deepcopy
# from tf.transformations import quaternion_from_euler

# from geometry_msgs.msg import Pose
# import rospkg


# rospy.init_node('demo_moveit')

# moveit_commander.roscpp_initialize(sys.argv)

# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()


# group = moveit_commander.MoveGroupCommander("Arm1_and_Arm2")
# group.set_named_target("home_pose")
# plan1 = group.plan()
# group.go(wait=True)

# for i in range(10):
#     group = moveit_commander.MoveGroupCommander("Arm1_and_Arm2")
#     group.set_named_target("first_pose")
#     plan1 = group.plan()
#     group.go(wait=True)

#     group = moveit_commander.MoveGroupCommander("Arm1_and_Arm2")
#     group.set_named_target("home_pose")
#     plan1 = group.plan()
#     group.go(wait=True)

# moveit_commander.roscpp_shutdown()
