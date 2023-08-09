#!/usr/bin/env python3

# import rospy
# from moveit_commander import MoveGroupCommander
# from geometry_msgs.msg import Pose
# import random
# import math

# rospy.init_node('moveit_demo')

# rocket_group = MoveGroupCommander("Arm_1")
# groot_group = MoveGroupCommander("Arm_2")

# rocket_joint_names = rocket_group.get_active_joints()
# groot_joint_names = groot_group.get_active_joints()

# rocket_joint_limits = [
#     (-2.5, 2.5),  # Joint 1
#     (-2.5, 2.5),  # Joint 2
#     (-2.5, 2.5),  # Joint 3
#     (-2.5, 2.5),  # Joint 4
#     (-2.5, 2.5),  # Joint 5
#     (-2.5, 2.5)   # Joint 6
# ]

# groot_joint_limits = [
#     (-2.5, 2.5),  # Joint 1
#     (-2.5, 2.5),  # Joint 2
#     (-2.5, 2.5),  # Joint 3
#     (-2.5, 2.5),  # Joint 4
#     (-2.5, 2.5),  # Joint 5
#     (-2.5, 2.5)   # Joint 6
# ]

# # Set the maximum planning time and number of planning attempts
# rocket_group.set_planning_time(15.0)
# rocket_group.set_num_planning_attempts(20)
# groot_group.set_planning_time(15.0)
# groot_group.set_num_planning_attempts(20)

# for i in range(100):
#     rocket_joint_values = []
#     groot_joint_values = []

#     # Generate random joint values within the joint limits for each arm
#     for j in range(len(rocket_joint_names)):
#         joint_limit = rocket_joint_limits[j]
#         joint_value = random.uniform(joint_limit[0], joint_limit[1])
#         rocket_joint_values.append(joint_value)

#         joint_limit = groot_joint_limits[j]
#         joint_value = random.uniform(joint_limit[0], joint_limit[1])
#         groot_joint_values.append(joint_value)

#     # Set the joint values for each arm
#     rocket_group.set_joint_value_target(rocket_joint_values)
#     groot_group.set_joint_value_target(groot_joint_values)

#     # Plan and execute the trajectories for each arm
#     rocket_success = rocket_group.go(wait=True)
#     groot_success = groot_group.go(wait=True)

#     if rocket_success and groot_success:
#         rospy.loginfo("Trajectories executed successfully")
#     else:
#         rospy.loginfo("Error occurred while executing trajectories")

# rospy.sleep(1)














import rospy
import math
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
from visualization_msgs.msg import Marker


def main_func(arm_group, 
              Px, Py, Pz, R, P, Y, 
              shape_radius, total_vertices):
    rospy.init_node('moveit_demo')

    robot = RobotCommander()
    arm_group = MoveGroupCommander(arm_group)

    arm_group.set_planning_time(5.0)  # Maximum allowed time for planning
    arm_group.set_num_planning_attempts(10)  # Number of planning attempts
    arm_group.set_max_velocity_scaling_factor(1)
    arm_group.set_max_acceleration_scaling_factor(1)

    target_pose = Pose()
    target_pose.position.x = Px
    target_pose.position.y = Py
    target_pose.position.z = Pz

    roll = (math.pi/180) * R
    pitch = (math.pi/180) * P
    yaw = (math.pi/180) * Y

    # euler = euler_from_quaternion([0.612, -0.3535, -7.544, 0.7071])
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2] 
    # rol = euler[0] * (180/math.pi)
    # pitc = euler[1] * (180/math.pi)
    # ya = euler[2] * (180/math.pi)
    # print([rol, pitc, ya])

    quaternion = quaternion_from_euler(roll, pitch, yaw)

    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]

    arm_group.set_pose_target(target_pose)
    arm_group.go(wait=True)



    arm1_waypoints = []
    radius = shape_radius
    center_x1 = Px
    center_y1 = Py
    circle_height = Pz
    num_circle_points = total_vertices

    for i in range(num_circle_points):
        angle = 2 * math.pi * i / num_circle_points
        poser = Pose()
        poser.position.x = center_x1 + (radius * math.cos(angle))
        poser.position.y = center_y1 + (radius * math.sin(angle))
        poser.position.z = circle_height

        roll = (math.pi/180) * R
        pitch = (math.pi/180) * P
        yaw = (math.pi/180) * Y

        quaternion = quaternion_from_euler(roll, pitch, yaw)

        poser.orientation.x = quaternion[0]
        poser.orientation.y = quaternion[1]
        poser.orientation.z = quaternion[2]
        poser.orientation.w = quaternion[3]

        arm1_waypoints.append(poser)

    # publish_path(arm1_waypoints)

    for j in range(2):
        for i in range(len(arm1_waypoints)):
            # print("%s value is : %s" % (i , arm1_waypoints[i]))
            arm_group.set_pose_target(arm1_waypoints[i])
            arm_group.go(wait=True)

if __name__=="__main__":

    main_func("Arm_1",
              -0.5, 0, 0.8,
              0, 90, 0,
              0.24, 50)
    
    main_func("Arm_2", 
              0.3, 0.57, 0.8, 
              95, -27, -76,
              0.24, 4)

    rospy.sleep(1)























# import rospy
# import math
# from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import Point
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import random
# from visualization_msgs.msg import Marker


# def publish_path(positions):
#     marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

#     marker = Marker()
#     marker.header.frame_id = "base_link"
#     # marker.header.frame_id = arm_group.get_end_effector_link()
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "my_namespace"
#     marker.id = 0
#     marker.type = Marker.LINE_STRIP
#     marker.action = Marker.ADD
#     marker.scale.x = 0.02  # Line width
#     marker.color.a = 1.0
#     marker.color.r = 1.0
#     marker.color.g = 0.0
#     marker.color.b = 0.0

#     for position in positions:
#         marker_point = Point()
#         marker_point.x = position.position.x
#         marker_point.y = position.position.y
#         marker_point.z = position.position.z
#         marker.points.append(marker_point)

#     marker_publisher.publish(marker)


# rospy.init_node('moveit_demo')

# robot = RobotCommander()
# arm_group = MoveGroupCommander("Arm_1")

# arm_group.set_planning_time(5.0)  # Maximum allowed time for planning
# arm_group.set_num_planning_attempts(10)  # Number of planning attempts
# arm_group.set_max_velocity_scaling_factor(1)
# arm_group.set_max_acceleration_scaling_factor(1)

# target_pose = Pose()
# target_pose.position.x = -0.5
# target_pose.position.y = 0
# target_pose.position.z = 0.8

# roll = (math.pi/180) * 0
# pitch = (math.pi/180) * 90
# yaw = (math.pi/180) * 0

# # euler = euler_from_quaternion([0.612, -0.3535, -7.544, 0.7071])
# # roll = euler[0]
# # pitch = euler[1]
# # yaw = euler[2] 
# # rol = euler[0] * (180/math.pi)
# # pitc = euler[1] * (180/math.pi)
# # ya = euler[2] * (180/math.pi)
# # print([rol, pitc, ya])

# quaternion = quaternion_from_euler(roll, pitch, yaw)

# target_pose.orientation.x = quaternion[0]
# target_pose.orientation.y = quaternion[1]
# target_pose.orientation.z = quaternion[2]
# target_pose.orientation.w = quaternion[3]

# arm_group.set_pose_target(target_pose)
# arm_group.go(wait=True)



# arm1_waypoints = []
# radius = 0.24
# center_x1 = -0.5
# center_y1 = 0
# circle_height = 0.8
# num_circle_points = 50

# for i in range(num_circle_points):
#     angle = 2 * math.pi * i / num_circle_points
#     poser = Pose()
#     poser.position.x = center_x1 + (radius * math.cos(angle))
#     poser.position.y = center_y1 + (radius * math.sin(angle))
#     poser.position.z = circle_height

#     roll = (math.pi/180) * 0
#     pitch = (math.pi/180) * 90
#     yaw = (math.pi/180) * 0

#     quaternion = quaternion_from_euler(roll, pitch, yaw)

#     poser.orientation.x = quaternion[0]
#     poser.orientation.y = quaternion[1]
#     poser.orientation.z = quaternion[2]
#     poser.orientation.w = quaternion[3]

#     arm1_waypoints.append(poser)

# # publish_path(arm1_waypoints)

# for j in range(3):
#     for i in range(len(arm1_waypoints)):
#         # print("%s value is : %s" % (i , arm1_waypoints[i]))
#         arm_group.set_pose_target(arm1_waypoints[i])
#         arm_group.go(wait=True)

# rospy.sleep(1)





























# import rospy
# import math
# from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import Point
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import random
# from visualization_msgs.msg import Marker


# # def publish_path(positions):
# #     marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

# #     marker = Marker()
# #     marker.header.frame_id = "base_link"
# #     # marker.header.frame_id = arm_group.get_end_effector_link()
# #     marker.header.stamp = rospy.Time.now()
# #     marker.ns = "my_namespace"
# #     marker.id = 0
# #     marker.type = Marker.LINE_STRIP
# #     marker.action = Marker.ADD
# #     marker.scale.x = 0.02  # Line width
# #     marker.color.a = 1.0
# #     marker.color.r = 1.0
# #     marker.color.g = 0.0
# #     marker.color.b = 0.0

# #     for position in positions:
# #         marker_point = Point()
# #         marker_point.x = position.position.x
# #         marker_point.y = position.position.y
# #         marker_point.z = position.position.z
# #         marker.points.append(marker_point)

# #     marker_publisher.publish(marker)

# class traj_gen(object):
    
#     def __init__(self, arm):
#         super(traj_gen, self).__init__()

#         rospy.init_node('demo_irp')

#         robot = RobotCommander()
#         arm_group = MoveGroupCommander(arm)

#         arm_group.set_planning_time(5.0)  # Maximum allowed time for planning
#         arm_group.set_num_planning_attempts(10)  # Number of planning attempts
#         arm_group.set_max_velocity_scaling_factor(1)
#         arm_group.set_max_acceleration_scaling_factor(1)

#         self.arm_group = arm_group

#     def move_to_pose(self, x, y, z, roll, pitch, yaw):

#         arm_group = self.arm_group

#         target_pose = Pose()
#         target_pose.position.x = x
#         target_pose.position.y = y
#         target_pose.position.z = z

#         roll = (math.pi/180) * roll
#         pitch = (math.pi/180) * pitch
#         yaw = (math.pi/180) * yaw
#         quaternion = quaternion_from_euler(roll, pitch, yaw)

#         target_pose.orientation.x = quaternion[0]
#         target_pose.orientation.y = quaternion[1]
#         target_pose.orientation.z = quaternion[2]
#         target_pose.orientation.w = quaternion[3]

#         arm_group.set_pose_target(target_pose)
#         arm_group.go(wait=True)

#     def path_trace(self, radius, total_vertices, cx, cy, cz, roll, pitch, yaw):

#         arm_group = self.arm_group

#         arm_waypoints = []
#         radius = radius
#         num_circle_points = total_vertices
#         center_x1 = cx
#         center_y1 = cy
#         circle_height = cz
        
#         for i in range(num_circle_points):
#             angle = 2 * math.pi * i / num_circle_points
#             poser = Pose()
#             poser.position.x = center_x1 + (radius * math.cos(angle))
#             poser.position.y = center_y1 + (radius * math.sin(angle))
#             poser.position.z = circle_height

#             roll = (math.pi/180) * roll
#             pitch = (math.pi/180) * pitch
#             yaw = (math.pi/180) * yaw
#             quaternion = quaternion_from_euler(roll, pitch, yaw)

#             poser.orientation.x = quaternion[0]
#             poser.orientation.y = quaternion[1]
#             poser.orientation.z = quaternion[2]
#             poser.orientation.w = quaternion[3]

#             arm_waypoints.append(poser)

#         # publish_path(arm1_waypoints)

#         for j in range(4):
#             for i in range(len(arm_waypoints)):
#                 # print("%s value is : %s" % (i , arm1_waypoints[i]))
#                 arm_group.set_pose_target(arm_waypoints[i])
#                 arm_group.go(wait=True)


# if __name__ == "__main__":

#     arm_1 = traj_gen("Arm_1")
#     arm_2 = traj_gen("Arm_2")

#     # arm_1.move_to_pose(-0.55, 0, 0.925,
#     #                    0, 90, 0)
#     # arm_1.path_trace(0.24,  4,
#     #                  -0.5, 0, 0.8, 
#     #                  0, 90, 0)

#     arm_2.move_to_pose(0.3, 0.57, 0.8,
#                        95, -76, -27)
#     arm_2.path_trace(0.24, 3, 
#                      0.3, 0.57, 0.8,
#                      95, -76, -27)

#     # arm_2.move_to_pose(0.28, 0.51, 0.90,
#     #                    90, -70, 30)




#     rospy.sleep(1)


















# def arm_straight():
#     # Define the target pose for the arm
#     target_pose = Pose()
#     target_pose.position.x = -0.5
#     target_pose.position.y = 0
#     target_pose.position.z = 0.8

#     # target_pose.orientation.x = 0.30
#     # target_pose.orientation.y = 0.30
#     # target_pose.orientation.z = 0.20
#     # target_pose.orientation.w = 1

#     # Euler angles (in radians)
#     roll = 0.0
#     pitch = math.pi / 2 
#     yaw = 0 

#     # Convert Euler angles to quaternion
#     quaternion = quaternion_from_euler(roll, pitch, yaw)

#     target_pose.orientation.x = quaternion[0]
#     target_pose.orientation.y = quaternion[1]
#     target_pose.orientation.z = quaternion[2]
#     target_pose.orientation.w = quaternion[3]

#     arm_group.set_pose_target(target_pose)
#     arm_group.go(wait=True)

# def circular_path():
#     robot = RobotCommander()
#     rocket_group = MoveGroupCommander("Arm_1")
    
#     rocket_waypoints = []

#     # Define Circular Path for Rocket Arm
#     radius = 0.7
#     center_x1 = -1.4
#     center_y1 = 0
#     circle_height = 0.3
#     num_circle_points = 100

#     for i in range(num_circle_points):
#         angle = 2 * math.pi * i / num_circle_points
#         poser = Pose()
#         poser.position.x = center_x1 + (radius * math.cos(angle))
#         poser.position.y = center_y1 + (radius * math.sin(angle))
#         poser.position.z = circle_height
#         poser.orientation.w = 1.0
#         rocket_waypoints.append(poser)
    
#     rocket_group.set_pose_targets(rocket_waypoints)
#     rocket_group.go(wait=True)

# if __name__ == "__main__":

#     rospy.init_node('moveit_demo')

#     robot = RobotCommander()
#     arm_group = MoveGroupCommander("Arm_1")

#     # Set planning parameters
#     arm_group.set_planning_time(5.0)  # Maximum allowed time for planning
#     arm_group.set_num_planning_attempts(10)  # Number of planning attempts

#     arm_straight()
#     circular_path()


