#!/usr/bin/env python3


import rospy
import math
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
from visualization_msgs.msg import Marker

class traj_gen(object):
    
    def __init__(self, arm_group):

        self.arm_group = arm_group

    def move_to_pose(self, px, py, pz, 
                     proll, ppitch, pyaw):

        target_pose = Pose()
        target_pose.position.x = px
        target_pose.position.y = py
        target_pose.position.z = pz

        roll = (math.pi/180) * proll
        pitch = (math.pi/180) * ppitch
        yaw = (math.pi/180) * pyaw
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True)

    def path_trace(self, radius, total_vertices, 
                   cx, cy, cz, 
                   pt_roll, pt_pitch, pt_yaw):

        arm_waypoints = []
        radius = radius
        num_circle_points = total_vertices
        center_x1 = cx
        center_y1 = cy
        circle_height = cz
        
        for i in range(num_circle_points):
            angle = 2 * math.pi * i / num_circle_points
            poser = Pose()
            poser.position.x = center_x1 + (radius * math.cos(angle))
            poser.position.y = center_y1 + (radius * math.sin(angle))
            poser.position.z = circle_height

            roll = (math.pi/180) * pt_roll
            pitch = (math.pi/180) * pt_pitch
            yaw = (math.pi/180) * pt_yaw
            quaternion = quaternion_from_euler(roll, pitch, yaw)

            poser.orientation.x = quaternion[0]
            poser.orientation.y = quaternion[1]
            poser.orientation.z = quaternion[2]
            poser.orientation.w = quaternion[3]

            arm_waypoints.append(poser)

        for j in range(2):
            for i in range(len(arm_waypoints)):
                # print("%s value is : %s" % (i , arm_waypoints[i]))
                self.arm_group.set_pose_target(arm_waypoints[i])
                self.arm_group.go(wait=True)


if __name__ == "__main__":
    try:

        rospy.init_node('demo_irp')

        robot = RobotCommander()
        arm1_group = MoveGroupCommander("Arm_1")
        arm1_group.set_planning_time(5.0) 
        arm1_group.set_num_planning_attempts(10) 
        arm1_group.set_max_velocity_scaling_factor(1)
        arm1_group.set_max_acceleration_scaling_factor(1)

        arm2_group = MoveGroupCommander("Arm_2")
        arm2_group.set_planning_time(5.0) 
        arm2_group.set_num_planning_attempts(10) 
        arm2_group.set_max_velocity_scaling_factor(1)
        arm2_group.set_max_acceleration_scaling_factor(1)

        arm_1 = traj_gen(arm1_group)
        arm_2 = traj_gen(arm2_group)

        # arm_1.move_to_pose(-0.85, 0, 0.4,
        #                 0, 90, 0)

        arm_1.move_to_pose(-0.55, 0, 0.8,
                        0, 90, 0)
        arm_1.path_trace(0.24, 10,
                        -0.55, 0, 0.8, 
                        0, 90, 0)

        arm_2.move_to_pose(0.3, 0.57, 0.8,
                           95, -27, -76)
        arm_2.path_trace(0.24, 3, 
                         0.3, 0.57, 0.8,
                         95, -27, -76)

    except rospy.ROSInterruptException:
        pass

