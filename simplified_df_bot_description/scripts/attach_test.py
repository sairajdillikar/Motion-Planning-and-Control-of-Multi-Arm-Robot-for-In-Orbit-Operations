#!/usr/bin/env python3






# import rospy
# from gazebo_ros_link_attacher.srv import Attach,AttachRequest,AttachResponse
# from gazebo_msgs.srv import SpawnModel,SpawnModelRequest,SpawnModelResponse
# from copy import deepcopy
# from tf.transformations import quaternion_from_euler

# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64
# from std_srvs.srv import Empty
# import math
# from control_msgs.srv import (QueryTrajectoryState, QueryTrajectoryStateRequest, QueryTrajectoryStateResponse)
# import time

# from geometry_msgs.msg import Pose
# import rospkg



# # sdf_cube = """<?xml version="1.0" ?>
# # <sdf version="1.4">
# #   <model name="MODELNAME">
# #     <static>true</static>
# #     <link name="link">
# #       <inertial>
# #         <mass>1.0</mass>
# #         <inertia>
# #           <ixx>0.01</ixx>
# #           <ixy>0.0</ixy>
# #           <ixz>0.0</ixz>
# #           <iyy>0.01</iyy>
# #           <iyz>0.0</iyz>
# #           <izz>0.01</izz>
# #         </inertia>
# #       </inertial>
# #       <collision name="stairs_collision0">
# #         <pose>0 0 0 0 0 0</pose>
# #         <geometry>
# #           <box>
# #             <size>SIZEXYZ</size>
# #           </box>
# #         </geometry>
# #         <surface>
# #           <bounce />
# #           <friction>
# #             <ode>
# #               <mu>1.0</mu>
# #               <mu2>1.0</mu2>
# #             </ode>
# #           </friction>
# #           <contact>
# #             <ode>
# #               <kp>10000000.0</kp>
# #               <kd>1.0</kd>
# #               <min_depth>0.0</min_depth>
# #               <max_vel>0.0</max_vel>
# #             </ode>
# #           </contact>
# #         </surface>
# #       </collision>
# #       <visual name="stairs_visual0">
# #         <pose>0 0 0 0 0 0</pose>
# #         <geometry>
# #           <box>
# #             <size>SIZEXYZ</size>
# #           </box>
# #         </geometry>
# #         <material>
# #           <script>
# #             <uri>file://media/materials/scripts/gazebo.material</uri>
# #             <name>Gazebo/Wood</name>
# #           </script>
# #         </material>
# #       </visual>
# #       <velocity_decay>
# #         <linear>0.000000</linear>
# #         <angular>0.000000</angular>
# #       </velocity_decay>
# #       <self_collide>0</self_collide>
# #       <kinematic>0</kinematic>
# #       <gravity>1</gravity>
# #     </link>
# #   </model>
# # </sdf>
# # """




# sdf_cube = """<?xml version="1.0" ?>
# <sdf version="1.4">
#   <model name="MODELNAME">
#     <static>true</static>
#     <link name="cube_link">
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
    
#     <joint name="fixing_to_world" type="fixed">
#         <parent link="dummy_link"/>
#         <child link="cube_link"/>
#         <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
#     </joint>

#   </model>
# </sdf>
# """

# sdf_srs = """
# <?xml version="1.0" ?>
# <sdf version="1.4">
#   <model name="MODELNAME">
#     <static>0</static>
#     <link name="base_link">
#       <inertial>
#         <mass>0.01</mass>
#         <inertia>
#           <ixx>0.01</ixx>
#           <ixy>0.0</ixy>
#           <ixz>0.0</ixz>
#           <iyy>0.01</iyy>
#           <iyz>0.0</iyz>
#           <izz>0.01</izz>
#         </inertia>
#       </inertial>
      
#       <collision name="collision">
#         <pose>0 0 0 0 0 0</pose>
#         <!-- <pose>4.0923159 0 2.95549512 0 0 0</pose> -->
#         <geometry>
#           <mesh>
#             <!-- <scale>0.001 0.001 0.001</scale> -->
#             <scale>0.0003 0.0003 0.0003</scale>

#             <!-- Add your PC Username here -->
#             <uri>/home/peraspera/test_ws/src/srs_modules_description/meshes/base_link.stl</uri>
            
#           </mesh>
#         </geometry>
#       </collision>
        
#       <visual name="visual">
#         <pose>0 0 0 0 0 0</pose>
#         <!-- <pose>4.0923159 0 2.95549512 0 0 0</pose> -->
#         <geometry>
#           <mesh>
#             <!-- <scale>0.001 0.001 0.001</scale> -->
#             <scale>0.0003 0.0003 0.0003</scale>

#             <!-- Add your PC Username here -->
#             <uri>/home/peraspera/test_ws/src/srs_modules_description/meshes/base_link.stl</uri>

#           </mesh>
#         </geometry>

#         <material>
#           <script>
#             <name>Gazebo/Silver</name>
#             <uri>file://media/materials/scripts/gazebo.material</uri>
#           </script>
#         </material>
#       </visual>
        
#     </link>
#   </model>
# </sdf>
# """



# def create_cube_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
#     """Create a SpawnModelRequest with the parameters of the cube given.
#     modelname: name of the model for gazebo
#     px py pz: position of the cube (and it's collision cube)
#     rr rp ry: rotation (roll, pitch, yaw) of the model
#     sx sy sz: size of the cube"""
#     cube = deepcopy(sdf_srs)
#     # Replace size of model
#     size_str = str(round(sx, 3)) + " " + str(round(sy, 3)) + " " + str(round(sz, 3))
#     cube = cube.replace('SIZEXYZ', size_str)
#     # Replace modelname
#     cube = cube.replace('MODELNAME', str(modelname))

#     pose_str = str(round(px, 3)) + " " + str(round(py, 3)) + " " + str(round(pz, 3)) + " " + \
#                str(round(rr, 3)) + " " + str(round(rp, 3)) + " " + str(round(ry, 3))
#     # cube = cube.replace('POSEXYZRPY', pose_str)

#     # print(cube)

#     req = SpawnModelRequest()
#     req.model_name = modelname
#     req.model_xml = cube
#     req.initial_pose.position.x = px
#     req.initial_pose.position.y = py
#     req.initial_pose.position.z = pz

#     q = quaternion_from_euler(rr, rp, ry)
#     req.initial_pose.orientation.x = q[0]
#     req.initial_pose.orientation.y = q[1]
#     req.initial_pose.orientation.z = q[2]
#     req.initial_pose.orientation.w = q[3]

#     return req

# rospack = rospkg.RosPack()
# package_path = rospack.get_path('srs_modules_description')
# file_path = package_path + '/meshes/srs_module.sdf'
# model_xml_path = open(file_path, 'r').read()

# def spawn_srs(i, main_body):
#     spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
#     spawn_model_client(
#     model_name='srs_module_' + str(i),
#     model_xml=model_xml_path,
#     # Enter the directory name to your system's workspace directory along with the username
#     # model_xml=open('/home/peraspera/mario_ws/src/srs_modules_description/meshes/srs_module.sdf', 'r').read(),
#     robot_namespace='/spawn_srs_ns',
#     initial_pose=Pose(),
#     reference_frame= main_body #'world' # Change the world frame to the main SRS 5 singlebody
#     )

# def attach_models(model_1, link_1, model_2, link_2):
#     rospy.loginfo("Attaching {} and {}".format(model_1, model_2))
#     attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
#     attach_srv.wait_for_service()
#     req = AttachRequest()
#     req.model_name_1 = model_1
#     req.link_name_1 = link_1
#     req.model_name_2 = model_2
#     req.link_name_2 = link_2
#     attach_srv.call(req)

# def detach_models(model_1, link_1, model_2, link_2):
#     rospy.loginfo("Detaching {} and {}".format(model_1, model_2))
#     attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
#     attach_srv.wait_for_service()
#     req = AttachRequest()
#     req.model_name_1 = model_1
#     req.link_name_1 = link_1
#     req.model_name_2 = model_2
#     req.link_name_2 = link_2
#     attach_srv.call(req)



# if __name__ == '__main__':
#     rospy.init_node('spawn_model_and_attach')
#     spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
#     rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
#     spawn_srv.wait_for_service()
#     rospy.loginfo("Connected to service!")
  
#     # # Spawn object custom
#     # rospy.loginfo("Spawning cube1")
#     # req3 = create_cube_request("cube1",
#     #                           -0.241382, 0.105095, 0.41,  # position
#     #                           0.0, 0.0, 0.564658,  # rotation
#     #                           0.2, 0.2, 0.2)  # size
#     # # spawn_srv.call(req3)
#     # rospy.sleep(1.0)

#     # rospy.loginfo("Spawning cube2")
#     # req3 = create_cube_request("cube2",
#     #                           0.001580, 0.262748, 0.41,  # position
#     #                           0.0, 0.0, 0.564658,  # rotation
#     #                           0.2, 0.2, 0.2)  # size
#     # # spawn_srv.call(req3)
#     # rospy.sleep(1.0)



#     # req1 = create_cube_request("srs_1",
#     #                           -0.241382, 0.105095, 0.41,  # position
#     #                           1.5707, 0.0, 0.524461,  # rotation
#     #                           0.2, 0.2, 0.2)
#     # spawn_srv.call(req1)





# # Spawn 5 SRS 


#     req1 = create_cube_request("srs_1",
#                               -0.724398, 0.264021, 0.409962,
#                               1.5707, 0.0, 0.524461,  # rotation
#                               0.2, 0.2, 0.2)
#     spawn_srv.call(req1)


#     req1 = create_cube_request("srs_2",
#                               -0.303509, 0.505222, 0.409962, 
#                               1.5707, 0.0, 0.524461,  # rotation
#                               0.2, 0.2, 0.2)
#     spawn_srv.call(req1)


#     req1 = create_cube_request("srs_3",
#                               0.119184, 0.755238, 0.409962, 
#                               1.5707, 0.0, 0.524461,  # rotation
#                               0.2, 0.2, 0.2)
#     spawn_srv.call(req1)


#     req1 = create_cube_request("srs_4",
#                               0.533501, 1.005666, 0.409962, 
#                               1.5707, 0.0, 0.524461,  # rotation
#                               0.2, 0.2, 0.2)
#     spawn_srv.call(req1)


#     req1 = create_cube_request("srs_5",
#                               0.930640, 1.233808, 0.409962, 
#                               1.5707, 0.0, 0.524461,  # rotation
#                               0.2, 0.2, 0.2)
#     spawn_srv.call(req1)







#     # spawn_srs(0, "")






#     attach_models("simplified_df_bot", "link_06_1", "srs_1", "base_link")
    
#     attach_models("simplified_df_bot", "link_06__2__1", "srs_3", "base_link")


#     # detach_models("simplified_df_bot", "link_06_1", "srs_1", "base_link")
    
#     # detach_models("simplified_df_bot", "link_06__2__1", "srs_3", "base_link")


















































import rospy
from gazebo_ros_link_attacher.srv import Attach,AttachRequest,AttachResponse
from gazebo_msgs.srv import SpawnModel,SpawnModelRequest,SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
import math
from control_msgs.srv import (QueryTrajectoryState, QueryTrajectoryStateRequest, QueryTrajectoryStateResponse)
import time

from geometry_msgs.msg import Pose
import rospkg

sdf_srs = """
<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="MODELNAME">
    <static>0</static>
    <link name="base_link">
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      
      <collision name="collision">
        <pose>0 0 0 0 0 0</pose>
        <!-- <pose>4.0923159 0 2.95549512 0 0 0</pose> -->
        <geometry>
          <mesh>
            <!-- <scale>0.001 0.001 0.001</scale> -->
            <scale>0.0003 0.0003 0.0003</scale>

            <!-- Add your PC Username here -->
            <uri>/home/peraspera/test2_ws/src/srs_modules_description/meshes/base_link.stl</uri>
            
          </mesh>
        </geometry>
      </collision>
        
      <visual name="visual">
        <pose>0 0 0 0 0 0</pose>
        <!-- <pose>4.0923159 0 2.95549512 0 0 0</pose> -->
        <geometry>
          <mesh>
            <!-- <scale>0.001 0.001 0.001</scale> -->
            <scale>0.0003 0.0003 0.0003</scale>

            <!-- Add your PC Username here -->
            <uri>/home/peraspera/test2_ws/src/srs_modules_description/meshes/base_link.stl</uri>

          </mesh>
        </geometry>

        <material>
          <script>
            <name>Gazebo/Silver</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
        
    </link>
  </model>
</sdf>
"""

def create_srs_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    cube = deepcopy(sdf_srs)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + str(round(sy, 3)) + " " + str(round(sz, 3))
    cube = cube.replace('SIZEXYZ', size_str)
    # Replace modelname
    cube = cube.replace('MODELNAME', str(modelname))

    pose_str = str(round(px, 3)) + " " + str(round(py, 3)) + " " + str(round(pz, 3)) + " " + \
               str(round(rr, 3)) + " " + str(round(rp, 3)) + " " + str(round(ry, 3))
    # cube = cube.replace('POSEXYZRPY', pose_str)

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

def attach_models(model_1, link_1, model_2, link_2):
    rospy.loginfo("Attaching {} and {}".format(model_1, model_2))
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    req = AttachRequest()
    req.model_name_1 = model_1
    req.link_name_1 = link_1
    req.model_name_2 = model_2
    req.link_name_2 = link_2
    attach_srv.call(req)

def detach_models(model_1, link_1, model_2, link_2):
    rospy.loginfo("Detaching {} and {}".format(model_1, model_2))
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    attach_srv.wait_for_service()
    req = AttachRequest()
    req.model_name_1 = model_1
    req.link_name_1 = link_1
    req.model_name_2 = model_2
    req.link_name_2 = link_2
    attach_srv.call(req)



if __name__ == '__main__':
    rospy.init_node('spawn_model_and_attach')
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")
  
# Spawn 5 SRS 

    # req1 = create_srs_request("srs_1",
    #                           -0.724398, 0.264021, 0.409962,
    #                           1.5707, 0.0, 0.524461,  # rotation
    #                           0.2, 0.2, 0.2)
    # spawn_srv.call(req1)


    # req1 = create_srs_request("srs_2",
    #                           -0.303509, 0.505222, 0.409962, 
    #                           1.5707, 0.0, 0.524461,  # rotation
    #                           0.2, 0.2, 0.2)
    # spawn_srv.call(req1)


    # req1 = create_srs_request("srs_3",
    #                           0.119184, 0.755238, 0.409962, 
    #                           1.5707, 0.0, 0.524461,  # rotation
    #                           0.2, 0.2, 0.2)
    # spawn_srv.call(req1)


    # req1 = create_srs_request("srs_4",
    #                           0.533501, 1.005666, 0.409962, 
    #                           1.5707, 0.0, 0.524461,  # rotation
    #                           0.2, 0.2, 0.2)
    # spawn_srv.call(req1)


    # req1 = create_srs_request("srs_5",
    #                           0.930640, 1.233808, 0.409962, 
    #                           1.5707, 0.0, 0.524461,  # rotation
    #                           0.2, 0.2, 0.2)
    # spawn_srv.call(req1)

    attach_models("simplified_df_bot", "link_06_1", "srs_1", "base_link")
    
    attach_models("simplified_df_bot", "link_06__2__1", "srs_3", "base_link")

    # detach_models("simplified_df_bot", "link_06_1", "srs_1", "base_link")
    
    # detach_models("simplified_df_bot", "link_06__2__1", "srs_3", "base_link")
