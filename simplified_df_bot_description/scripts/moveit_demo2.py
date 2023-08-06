#!/usr/bin/env python3






import rospy
import random
import tf.transformations as tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_demo')

    rng_group_interface = moveit_commander.MoveGroupCommander("Arm1_and_Arm2")
    arm1_group_interface = moveit_commander.MoveGroupCommander("Arm_1")
    arm2_group_interface = moveit_commander.MoveGroupCommander("Arm_2")

    rng_group_interface.set_max_velocity_scaling_factor(1.0)
    rng_group_interface.set_max_acceleration_scaling_factor(1.0)
    rng_group_interface.set_planning_time(5.0)
    rng_group_interface.set_num_planning_attempts(20)

    # kinematic_model = robot.get_robot_model()
    kinematic_state = rng_group_interface.get_current_state()
    arm1_joint_names = arm1_group_interface.get_active_joints()
    arm2_joint_names = arm2_group_interface.get_active_joints()

    random.seed()

    arm1_pose = geometry_msgs.msg.Pose()
    arm2_pose = geometry_msgs.msg.Pose()

    arm1_q = tf.quaternion_from_euler(0.0, 0.0, 0.0)  # edit this
    arm2_q = tf.quaternion_from_euler(0.0, 0.0, 0.0)

    for i in range(10):
        random_x = random.uniform(-10, 10) * 0.005
        random_y = random.uniform(-10, 10) * 0.005
        random_z = random.uniform(-10, 10) * 0.005

        arm1_pose.position.x = -0.4123891794664267 + random_x  
        arm1_pose.position.y = 0.4812721410480939 + random_y
        arm1_pose.position.z = 0.5010054793908278 + random_z
        arm2_pose.position.x = -0.24922596187320153 + random_x
        arm2_pose.position.y = 0.5832570486483674 + random_y
        arm2_pose.position.z = 0.5003571141282077 + random_z
        
        arm1_pose.orientation.w = 0.23738517897343436
        arm1_pose.orientation.x = 0.11480451543556341
        arm1_pose.orientation.y = -0.41838517544784903
        arm1_pose.orientation.z = -0.8691501855288467
        arm2_pose.orientation.w = 0.22514798408245928
        arm2_pose.orientation.x = 0.00046480793594759116
        arm2_pose.orientation.y = -3.2008558286359785e-05
        arm2_pose.orientation.z = 0.974324467614685

        timeout = rospy.Duration(0.1)

        arm1_pose_set = arm1_group_interface.set_pose_target(arm1_pose)
        arm2_pose_set = arm2_group_interface.set_pose_target(arm2_pose)

        arm1_group_interface.go(wait=True)
        arm2_group_interface.go(wait=True)


if __name__ == '__main__':
    main()




















# import rospy
# import random
# import tf.transformations as tf
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# import sys

# def main():
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('moveit_demo')

#     rng_group_interface = moveit_commander.MoveGroupCommander("Arm1_and_Arm2")
#     arm1_group_interface = moveit_commander.MoveGroupCommander("Arm_1")
#     arm2_group_interface = moveit_commander.MoveGroupCommander("Arm_2")

#     rng_group_interface.set_max_velocity_scaling_factor(1.0)
#     rng_group_interface.set_max_acceleration_scaling_factor(1.0)
#     rng_group_interface.set_planning_time(5.0)
#     rng_group_interface.set_num_planning_attempts(20)

#     # kinematic_model = robot.get_robot_model()
#     kinematic_state = rng_group_interface.get_current_state()
#     arm1_joint_names = arm1_group_interface.get_active_joints()
#     arm2_joint_names = arm2_group_interface.get_active_joints()

#     random.seed()

#     arm1_pose = geometry_msgs.msg.Pose()
#     arm2_pose = geometry_msgs.msg.Pose()

#     arm1_q = tf.quaternion_from_euler(0.0, 0.0, 0.0)  # edit this
#     arm2_q = tf.quaternion_from_euler(0.0, 0.0, 0.0)

#     for i in range(10):
#         random_x = random.uniform(-10, 10) * 0.005
#         random_y = random.uniform(-10, 10) * 0.005
#         random_z = random.uniform(-10, 10) * 0.005

#         arm1_pose.position.x = -0.4123891794664267 + random_x  
#         arm1_pose.position.y = 0.4812721410480939 + random_y
#         arm1_pose.position.z = 0.5010054793908278 + random_z
#         arm2_pose.position.x = -0.24922596187320153 + random_x
#         arm2_pose.position.y = 0.5832570486483674 + random_y
#         arm2_pose.position.z = 0.5003571141282077 + random_z

#         # x_rotation = random.uniform(-30, 30) * 0.01
#         # y_rotation = random.uniform(-30, 30) * 0.01
#         # z_rotation = random.uniform(-30, 30) * 0.01

#         # arm1_q = tf.quaternion_multiply(
#         #     tf.quaternion_from_euler(x_rotation, y_rotation, z_rotation),
#         #     arm1_q
#         # )

#         # arm2_q = tf.quaternion_multiply(
#         #     tf.quaternion_from_euler(x_rotation, y_rotation, z_rotation),
#         #     arm2_q
#         # )

#         # arm1_pose.orientation.w = arm1_q[3]
#         # arm1_pose.orientation.x = arm1_q[0]
#         # arm1_pose.orientation.y = arm1_q[1]
#         # arm1_pose.orientation.z = arm1_q[2]
#         # arm2_pose.orientation.w = arm2_q[3]
#         # arm2_pose.orientation.x = arm2_q[0]
#         # arm2_pose.orientation.y = arm2_q[1]
#         # arm2_pose.orientation.z = arm2_q[2]
        
#         arm1_pose.orientation.w = 0.23738517897343436
#         arm1_pose.orientation.x = 0.11480451543556341
#         arm1_pose.orientation.y = -0.41838517544784903
#         arm1_pose.orientation.z = -0.8691501855288467
#         arm2_pose.orientation.w = 0.22514798408245928
#         arm2_pose.orientation.x = 0.00046480793594759116
#         arm2_pose.orientation.y = -3.2008558286359785e-05
#         arm2_pose.orientation.z = 0.974324467614685

#         timeout = rospy.Duration(0.1)

#         arm1_pose_set = arm1_group_interface.set_pose_target(arm1_pose)
#         arm2_pose_set = arm2_group_interface.set_pose_target(arm2_pose)

#         arm1_group_interface.go(wait=True)
#         arm2_group_interface.go(wait=True)

#         # print(arm1_pose_set)

#         # rock_joint_tar = arm1_group_interface.get_joint_value_target()
#         # groo_joint_tar = arm2_group_interface.get_joint_value_target()

#         # # arm1_pose_set = arm1_group_interface.get_current_joint_values()
#         # # arm2_pose_set = arm2_group_interface.get_current_joint_values()

#         # arm1_joint_values = []
#         # arm2_joint_values = []
#         # arm1_joint_values = arm1_group_interface.get_current_joint_values()
#         # arm2_joint_values = arm2_group_interface.get_current_joint_values()
#         # # arm1_joint_values = arm1_group_interface.set_pose_target(arm1_pose)
#         # # arm2_joint_values = arm2_group_interface.set_pose_target(arm2_pose)


#         # joint_goal = rng_group_interface.get_current_joint_values()

#         # joint_goal[0] = arm1_joint_values[0]
#         # joint_goal[1] = arm1_joint_values[1]
#         # joint_goal[2] = arm1_joint_values[2]
#         # joint_goal[3] = arm1_joint_values[3]
#         # joint_goal[4] = arm1_joint_values[4]
#         # joint_goal[5] = arm1_joint_values[5]
        
#         # joint_goal[6] = arm2_joint_values[0]
#         # joint_goal[7] = arm2_joint_values[1]
#         # joint_goal[8] = arm2_joint_values[2]
#         # joint_goal[9] = arm2_joint_values[3]
#         # joint_goal[10] = arm2_joint_values[4]
#         # joint_goal[11] = arm2_joint_values[5]



#         # # something = arm1_group_interface.get_active_joints()
#         # # print(something)

#         # # rng_joint_values = rng_group_interface.get_current_joint_values()

#         # rng_joint_values = rng_group_interface.go(joint_goal, wait=True)

#         # print(" ")

#     #     if arm1_pose_set and arm2_pose_set:

#     #         arm1_joint_values = arm1_group_interface.get_current_joint_values()
#     #         arm2_joint_values = arm2_group_interface.get_current_joint_values()

#     #         for j in range(len(arm1_joint_names)):
#     #             rospy.loginfo("Joint %s: %f", arm1_joint_names[j], arm1_joint_values[j])
#     #             rospy.loginfo("Joint %s: %f", arm2_joint_names[j], arm2_joint_values[j])
#     #     else:
#     #         rospy.loginfo("Did not find IK solution")

#     #     rng_group_interface.set_joint_value_target(arm1_joint_values, arm1_joint_names)
#     #     rng_group_interface.set_joint_value_target(arm2_joint_values, arm2_joint_names)

#     #     my_plan = rng_group_interface.plan()
#     #     success = (my_plan.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
        
#     #     if not success:
#     #         rospy.loginfo("Plan did not succeed")

#     #     rng_group_interface.execute(my_plan)

#     # rospy.loginfo("Demo finished.")

# if __name__ == '__main__':
#     main()
































# import rospy
# import random
# import tf.transformations as tf
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# import sys

# def main():
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('moveit_demo')

#     robot = moveit_commander.RobotCommander()
#     rng_group_interface = moveit_commander.MoveGroupCommander("Arm1_and_Arm2")
#     arm1_group_interface = moveit_commander.MoveGroupCommander("Arm_1")
#     arm2_group_interface = moveit_commander.MoveGroupCommander("Arm_2")

#     rng_group_interface.set_max_velocity_scaling_factor(1.0)
#     rng_group_interface.set_max_acceleration_scaling_factor(1.0)
#     rng_group_interface.set_planning_time(15.0)
#     rng_group_interface.set_num_planning_attempts(20)

#     kinematic_model = robot.get_robot_model()
#     kinematic_state = robot.get_current_state()
#     arm1_joint_model_group = kinematic_model.get_joint_model_group("Arm_1")
#     arm2_joint_model_group = kinematic_model.get_joint_model_group("Arm_2")

#     arm1_joint_names = arm1_joint_model_group.variable_names
#     arm2_joint_names = arm2_joint_model_group.variable_names

#     random.seed()

#     arm1_pose = geometry_msgs.msg.Pose()
#     arm2_pose = geometry_msgs.msg.Pose()

#     arm1_q = tf.quaternion_from_euler(0.0, 0.0, 0.0)
#     arm2_q = tf.quaternion_from_euler(0.0, 0.0, 0.0)

#     for i in range(100):
#         random_x = random.uniform(-10, 10) * 0.01
#         random_y = random.uniform(-10, 10) * 0.01
#         random_z = random.uniform(-10, 10) * 0.01

#         arm1_pose.position.x = -0.015463195119993365 + random_x
#         arm1_pose.position.y = 0.02029402510664674 + random_y
#         arm1_pose.position.z = 1.658157440477098 + random_z
#         arm2_pose.position.x = -0.01565011581780207 + random_x
#         arm2_pose.position.y = -0.019683543216663102 + random_y
#         arm2_pose.position.z = 1.657396455658871 + random_z

#         x_rotation = random.uniform(-30, 30) * 0.01
#         y_rotation = random.uniform(-30, 30) * 0.01
#         z_rotation = random.uniform(-30, 30) * 0.01

#         arm1_q = tf.quaternion_multiply(
#             tf.quaternion_from_euler(x_rotation, y_rotation, z_rotation),
#             arm1_q
#         )

#         arm2_q = tf.quaternion_multiply(
#             tf.quaternion_from_euler(x_rotation, y_rotation, z_rotation),
#             arm2_q
#         )

#         arm1_pose.orientation.w = arm1_q[3]
#         arm1_pose.orientation.x = arm1_q[0]
#         arm1_pose.orientation.y = arm1_q[1]
#         arm1_pose.orientation.z = arm1_q[2]
#         arm2_pose.orientation.w = arm2_q[3]
#         arm2_pose.orientation.x = arm2_q[0]
#         arm2_pose.orientation.y = arm2_q[1]
#         arm2_pose.orientation.z = arm2_q[2]

#         timeout = rospy.Duration(0.1)
#         arm1_found_ik = kinematic_state.set_fromIK(arm1_joint_model_group, arm1_pose, timeout)
#         arm2_found_ik = kinematic_state.set_fromIK(arm2_joint_model_group, arm2_pose, timeout)

#         if arm1_found_ik and arm2_found_ik:
#             arm1_joint_values = kinematic_state.get_joint_values(arm1_joint_names)
#             arm2_joint_values = kinematic_state.get_joint_values(arm2_joint_names)

#             for i in range(len(arm1_joint_names)):
#                 rospy.loginfo("Joint %s: %f", arm1_joint_names[i], arm1_joint_values[i])
#                 rospy.loginfo("Joint %s: %f", arm2_joint_names[i], arm2_joint_values[i])
#         else:
#             rospy.loginfo("Did not find IK solution")

#         rng_group_interface.set_joint_value_target(arm1_joint_values, arm1_joint_names)
#         rng_group_interface.set_joint_value_target(arm2_joint_values, arm2_joint_names)

#         my_plan = rng_group_interface.plan()
#         success = (my_plan.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
#         if not success:
#             rospy.loginfo("Plan did not succeed")

#         rng_group_interface.execute(my_plan)

#     rospy.loginfo("Demo finished.")

# if __name__ == '__main__':
#     main()

























# import rospy
# import random
# import tf.transformations as tf
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# import sys

# def main():
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('moveit_demo')

#     robot = moveit_commander.RobotCommander()
#     rng_group_interface = moveit_commander.MoveGroupCommander("Arm1_and_Arm2")
#     arm1_group_interface = moveit_commander.MoveGroupCommander("Arm_1")
#     arm2_group_interface = moveit_commander.MoveGroupCommander("Arm_2")

#     rng_group_interface.set_max_velocity_scaling_factor(1.0)
#     rng_group_interface.set_max_acceleration_scaling_factor(1.0)
#     rng_group_interface.set_planning_time(15.0)
#     rng_group_interface.set_num_planning_attempts(20)

#     random.seed()

#     arm1_pose = geometry_msgs.msg.Pose()
#     arm2_pose = geometry_msgs.msg.Pose()

#     arm1_q = tf.quaternion_from_euler(0.0, 0.0, 0.0)
#     arm2_q = tf.quaternion_from_euler(0.0, 0.0, 0.0)

#     for i in range(100):
#         random_x = random.uniform(-10, 10) * 0.01
#         random_y = random.uniform(-10, 10) * 0.01
#         random_z = random.uniform(-10, 10) * 0.01

#         arm1_pose.position.x = -0.015463195119993365 + random_x
#         arm1_pose.position.y = 0.02029402510664674 + random_y
#         arm1_pose.position.z = 1.658157440477098 + random_z
#         arm2_pose.position.x = -0.01565011581780207 + random_x
#         arm2_pose.position.y = -0.019683543216663102 + random_y
#         arm2_pose.position.z = 1.657396455658871 + random_z

#         x_rotation = random.uniform(-30, 30) * 0.01
#         y_rotation = random.uniform(-30, 30) * 0.01
#         z_rotation = random.uniform(-30, 30) * 0.01

#         arm1_q = tf.quaternion_multiply(
#             tf.quaternion_from_euler(x_rotation, y_rotation, z_rotation),
#             arm1_q
#         )

#         arm2_q = tf.quaternion_multiply(
#             tf.quaternion_from_euler(x_rotation, y_rotation, z_rotation),
#             arm2_q
#         )

#         arm1_pose.orientation.w = arm1_q[3]
#         arm1_pose.orientation.x = arm1_q[0]
#         arm1_pose.orientation.y = arm1_q[1]
#         arm1_pose.orientation.z = arm1_q[2]
#         arm2_pose.orientation.w = arm2_q[3]
#         arm2_pose.orientation.x = arm2_q[0]
#         arm2_pose.orientation.y = arm2_q[1]
#         arm2_pose.orientation.z = arm2_q[2]

#         timeout = rospy.Duration(0.1)
#         arm1_found_ik = rng_group_interface.set_pose_target(arm1_pose)
#         arm2_found_ik = rng_group_interface.set_pose_target(arm2_pose)

#         if arm1_found_ik and arm2_found_ik:
#             rng_group_interface.plan()
#             success = rng_group_interface.go(wait=True)
#             if success:
#                 rospy.loginfo("Successfully executed the plan")
#             else:
#                 rospy.loginfo("Failed to execute the plan")
#         else:
#             rospy.loginfo("Did not find IK solution")

#     rospy.loginfo("Demo finished.")

# if __name__ == '__main__':
#     main()

