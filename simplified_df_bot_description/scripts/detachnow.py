#!/usr/bin/env python3

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
    # rospy.init_node('spawn_model_and_attach')

    # attach_models("simplified_df_bot", "link_06_1", "srs_1", "base_link")
    
    # attach_models("simplified_df_bot", "link_06__2__1", "srs_3", "base_link")

    # detach_models("simplified_df_bot", "link_06_1", "srs_1", "base_link")
    
    # detach_models("simplified_df_bot", "link_06__2__1", "srs_3", "base_link")



    detach_models("srs_1", "base_link", "simplified_df_bot", "link_06_1")
    
    detach_models("srs_3", "base_link", "simplified_df_bot", "link_06__2__1")