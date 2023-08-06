#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach,AttachRequest,AttachResponse
from gazebo_msgs.srv import SpawnModel,SpawnModelRequest,SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler

# sdf_cube = """<?xml version="1.0" ?>
# <sdf version="1.4">
#   <model name="MODELNAME">
#     <static>true</static>
#     <link name="link">
#       <inertial>
#         <mass>1.0</mass>
#         <inertia>
#           <ixx>0.01</ixx>
#           <ixy>0.0</ixy>
#           <ixz>0.0</ixz>
#           <iyy>0.01</iyy>
#           <iyz>0.0</iyz>
#           <izz>0.01</izz>
#         </inertia>
#       </inertial>
#       <collision name="stairs_collision0">
#         <pose>0 0 0 0 0 0</pose>
#         <geometry>
#           <box>
#             <size>SIZEXYZ</size>
#           </box>
#         </geometry>
#         <surface>
#           <bounce />
#           <friction>
#             <ode>
#               <mu>1.0</mu>
#               <mu2>1.0</mu2>
#             </ode>
#           </friction>
#           <contact>
#             <ode>
#               <kp>10000000.0</kp>
#               <kd>1.0</kd>
#               <min_depth>0.0</min_depth>
#               <max_vel>0.0</max_vel>
#             </ode>
#           </contact>
#         </surface>
#       </collision>
#       <visual name="stairs_visual0">
#         <pose>0 0 0 0 0 0</pose>
#         <geometry>
#           <box>
#             <size>SIZEXYZ</size>
#           </box>
#         </geometry>
#         <material>
#           <script>
#             <uri>file://media/materials/scripts/gazebo.material</uri>
#             <name>Gazebo/Wood</name>
#           </script>
#         </material>
#       </visual>
#       <velocity_decay>
#         <linear>0.000000</linear>
#         <angular>0.000000</angular>
#       </velocity_decay>
#       <self_collide>0</self_collide>
#       <kinematic>0</kinematic>
#       <gravity>1</gravity>
#     </link>
#   </model>
# </sdf>
# """



sdf_cube = """<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="MODELNAME">
    <static>true</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="stairs_collision0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <surface>
          <bounce>1</bounce>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>100.0</kp>
              <kd>0.5</kd>
              <min_depth>0.0</min_depth>
              <max_vel>0.0</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="stairs_visual0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.000000</linear>
        <angular>0.000000</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>
"""


def create_cube_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """Create a SpawnModelRequest with the parameters of the cube given.
    modelname: name of the model for gazebo
    px py pz: position of the cube (and it's collision cube)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the cube"""
    cube = deepcopy(sdf_cube)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    cube = cube.replace('SIZEXYZ', size_str)
    # Replace modelname
    cube = cube.replace('MODELNAME', str(modelname))

    pose_str = str(round(px, 3)) + " " + str(round(py, 3)) + " " + str(round(pz, 3)) + " " + \
               str(round(rr, 3)) + " " + str(round(rp, 3)) + " " + str(round(ry, 3))
    cube = cube.replace('POSEXYZRPY', pose_str)

    # print(cube)

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = cube
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


if __name__ == '__main__':
    rospy.init_node('spawn_model_and_attach')
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")
  
    # Spawn object custom
    rospy.loginfo("Spawning cube1")
    req3 = create_cube_request("cube1",
                              -0.109796, 0.188449, 0.41,  # position
                              0.0, 0.0, 0.564658,  # rotation
                              0.2, 0.2, 0.2)  # size
    spawn_srv.call(req3)
    rospy.sleep(1.0)

    # rospy.loginfo("Spawning cube2")
    # req3 = create_cube_request("cube2",
    #                           -0.109796, 0.188449, 0.41,  # position
    #                           0.0, 0.0, 0.564658,  # rotation
    #                           0.2, 0.2, 0.2)  # size
    # spawn_srv.call(req3)
    # rospy.sleep(1.0)

    # rospy.init_node('attach_models')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    rospy.loginfo("Attaching cube1 and Arm1_link_06")
    req = AttachRequest()
    req.model_name_1 = "simplified_df_bot"
    req.link_name_1 = "link_06_1"
    req.model_name_2 = "cube1"
    req.link_name_2 = "link"

    attach_srv.call(req)

    # rospy.loginfo("Attaching cube1 and Arm1_link_06")
    # req = AttachRequest()
    # req.model_name_1 = "simplified_df_bot"
    # req.link_name_1 = "link_06__2__1"
    # req.model_name_2 = "cube1"
    # req.link_name_2 = "link"

    # attach_srv.call(req)