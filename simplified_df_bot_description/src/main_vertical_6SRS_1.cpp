#include <ros/ros.h>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
#include <geometry_msgs/Point.h>

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>

#include <boost/thread/recursive_mutex.hpp>

#include <ignition/math/Pose3.hh>

#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

#include <cstdlib>
#include <iostream>
#include <thread> 
#include <gazebo_msgs/ModelState.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo_msgs/DeleteModel.h>
#include <atomic> 
#include <ros/duration.h>

void performIKandExecute(moveit::planning_interface::MoveGroupInterface& rng_group_interface,
                         moveit::core::RobotStatePtr& kinematic_state,
                         const moveit::core::JointModelGroup* arm1_joint_model_group,
                         const moveit::core::JointModelGroup* arm2_joint_model_group,
                         const std::vector<std::string>& arm1_joint_names,
                         const std::vector<std::string>& arm2_joint_names,
                         const geometry_msgs::Pose& arm1_pose,
                         const geometry_msgs::Pose& arm2_pose,
                         double timeout)
{
    std::vector<double> arm1_joint_values;
    std::vector<double> arm2_joint_values;

    bool arm1_found_ik = kinematic_state->setFromIK(arm1_joint_model_group, arm1_pose, timeout);
    bool arm2_found_ik = kinematic_state->setFromIK(arm2_joint_model_group, arm2_pose, timeout);

    if (arm1_found_ik && arm2_found_ik)
    {
        kinematic_state->copyJointGroupPositions(arm1_joint_model_group, arm1_joint_values);
        kinematic_state->copyJointGroupPositions(arm2_joint_model_group, arm2_joint_values);
    }
    else
    {
        ROS_INFO("Did not find IK solution");
        return; 
    }

    rng_group_interface.setJointValueTarget(arm1_joint_names, arm1_joint_values);
    rng_group_interface.setJointValueTarget(arm2_joint_names, arm2_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (rng_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
        ROS_INFO("Plan did not succeed");
        return; 
    }

    rng_group_interface.execute(my_plan);
}

// SDF without collision tag
std::string getSDFString()
{
    return R"(
<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="MODELNAME">
    <static>STATIC_SRS</static>
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
        
      <visual name="visual">
        <pose>0 0 0 0 0 0</pose>
        <!-- <pose>4.0923159 0 2.95549512 0 0 0</pose> -->
        <geometry>
          <mesh>
            <!-- <scale>0.001 0.001 0.001</scale> -->
            <scale>0.0003 0.0003 0.0003</scale>

            <!-- Add your PC Username here -->
            <uri>/home/peraspera/test3_ws/src/srs_modules_description/meshes/base_link.stl</uri>

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
)";
}

gazebo_msgs::SpawnModel createSRSRequest(const std::string& modelname, int modelstatic, double px, double py, double pz,
                                         double rr, double rp, double ry, double sx, double sy, double sz)
{
    std::string cube = getSDFString();
    std::string size_str = std::to_string(round(sx * 1000) / 1000) + " " +
                            std::to_string(round(sy * 1000) / 1000) + " " +
                            std::to_string(round(sz * 1000) / 1000);
    size_t sizePos = cube.find("SIZEXYZ");
    if (sizePos != std::string::npos)
        cube.replace(sizePos, 7, size_str);
    
    size_t modelNamePos = cube.find("MODELNAME");
    if (modelNamePos != std::string::npos)
        cube.replace(modelNamePos, 9, modelname);
  
    std::string model_static = (modelstatic == 1) ? "true" : "false";
    size_t modelStatic = cube.find("STATIC_SRS");
    if (modelStatic != std::string::npos)
        cube.replace(modelStatic, 10, model_static);

    gazebo_msgs::SpawnModel req;
    req.request.model_name = modelname;
    req.request.model_xml = cube;
    req.request.initial_pose.position.x = px;
    req.request.initial_pose.position.y = py;
    req.request.initial_pose.position.z = pz;

    tf::Quaternion quat;
    quat.setRPY(rr, rp, ry);
    req.request.initial_pose.orientation.x = quat.x();
    req.request.initial_pose.orientation.y = quat.y();
    req.request.initial_pose.orientation.z = quat.z();
    req.request.initial_pose.orientation.w = quat.w();

    return req;
}

std::string getORIGINString()
{
    return R"(
<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="MODELNAME">
    <static>STATIC_SRS</static>
    <link name="origin_base_link">
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
    </link>
  </model>
</sdf>
)";
}

gazebo_msgs::SpawnModel createORIGINRequest(const std::string& modelname, int modelstatic, double px, double py, double pz,
                                         double rr, double rp, double ry, double sx, double sy, double sz)
{
    std::string cube = getORIGINString();
    std::string size_str = std::to_string(round(sx * 1000) / 1000) + " " +
                            std::to_string(round(sy * 1000) / 1000) + " " +
                            std::to_string(round(sz * 1000) / 1000);
    size_t sizePos = cube.find("SIZEXYZ");
    if (sizePos != std::string::npos)
        cube.replace(sizePos, 7, size_str);
    
    size_t modelNamePos = cube.find("MODELNAME");
    if (modelNamePos != std::string::npos)
        cube.replace(modelNamePos, 9, modelname);
  
    std::string model_static = (modelstatic == 1) ? "true" : "false";
    size_t modelStatic = cube.find("STATIC_SRS");
    if (modelStatic != std::string::npos)
        cube.replace(modelStatic, 10, model_static);

    gazebo_msgs::SpawnModel req;
    req.request.model_name = modelname;
    req.request.model_xml = cube;
    req.request.initial_pose.position.x = px;
    req.request.initial_pose.position.y = py;
    req.request.initial_pose.position.z = pz;

    tf::Quaternion quat;
    quat.setRPY(rr, rp, ry);
    req.request.initial_pose.orientation.x = quat.x();
    req.request.initial_pose.orientation.y = quat.y();
    req.request.initial_pose.orientation.z = quat.z();
    req.request.initial_pose.orientation.w = quat.w();
    
    return req;
}

void attachModels(const std::string& model_1, const std::string& link_1,
                  const std::string& model_2, const std::string& link_2)
{
    ROS_INFO_STREAM("Attaching " << model_1 << " and " << model_2);
    ros::NodeHandle nh;
    ros::ServiceClient attachClient = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    attachClient.waitForExistence();

    gazebo_ros_link_attacher::AttachRequest attach_req;
    gazebo_ros_link_attacher::AttachResponse attach_res;
    attach_req.model_name_1 = model_1;
    attach_req.link_name_1 = link_1;
    attach_req.model_name_2 = model_2;
    attach_req.link_name_2 = link_2;
    if (attachClient.call(attach_req,attach_res))
    {
            if (attach_res.ok)
            {
              ROS_INFO("Attach was successful");
            }
            else
            {
              ROS_ERROR("Attach failed");
            }
    }
    else
    {
        ROS_ERROR("Failed to call attach service");
    }
}

void detachModels(const std::string& model_1, const std::string& link_1,
                  const std::string& model_2, const std::string& link_2)
{
    ROS_INFO_STREAM("Detaching " << model_1 << " and " << model_2);
    ros::NodeHandle nh;
    ros::ServiceClient detachClient = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    detachClient.waitForExistence();

    gazebo_ros_link_attacher::AttachRequest detach_req;
    gazebo_ros_link_attacher::AttachResponse detach_res;
    detach_req.model_name_1 = model_1;
    detach_req.link_name_1 = link_1;
    detach_req.model_name_2 = model_2;
    detach_req.link_name_2 = link_2;
    if (detachClient.call(detach_req,detach_res))
    {
        if (detach_res.ok)
        {
          ROS_INFO("Detach was successful");
        }
        else
        {
          ROS_ERROR("Detach failed");
        }
    }
    else
    {
      ROS_ERROR("Failed to call detach service");
    }
}

std::atomic<bool> stopBroadcasting(false);

void broadcastTf()
{
  while (!stopBroadcasting.load())
    {
      ros::NodeHandle nh;

      tf2_ros::TransformBroadcaster tfBroadcaster;

      ros::Rate rate(10.0);
      
      while (ros::ok() && !stopBroadcasting.load())
      {
        ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

        gazebo_msgs::GetWorldProperties worldPropertiesRequest;

        if (client.call(worldPropertiesRequest))
        {
          std::vector<std::string>& modelNames = worldPropertiesRequest.response.model_names;

          for (const std::string& modelName : modelNames)
          {
            ros::ServiceClient getStateClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

            gazebo_msgs::GetModelState modelStateRequest;
            modelStateRequest.request.model_name = modelName;
            modelStateRequest.request.relative_entity_name = "dummy_link";

            if (getStateClient.call(modelStateRequest))
            {
              geometry_msgs::TransformStamped transformStamped;
              transformStamped.header.stamp = ros::Time::now();
              transformStamped.header.frame_id = "dummy_link"; 
              transformStamped.child_frame_id = modelName; 
              transformStamped.transform.translation.x = modelStateRequest.response.pose.position.x;
              transformStamped.transform.translation.y = modelStateRequest.response.pose.position.y;
              transformStamped.transform.translation.z = modelStateRequest.response.pose.position.z;
              transformStamped.transform.rotation = modelStateRequest.response.pose.orientation;

              tfBroadcaster.sendTransform(transformStamped);
            }
            else
            {
              ROS_WARN_STREAM("Failed to get model state for " << modelName << " from Gazebo");
            }
          }
        }
        else
        {
          ROS_WARN("Failed to get list of model properties from Gazebo");
        }
        
        rate.sleep();
      }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void stopBroadcastingTf()
{
    stopBroadcasting.store(true);
    ROS_INFO("Stoping TF Broadcast!");
}

void startBroadcastingTf()
{
    stopBroadcasting.store(false);
    ROS_INFO("Starting TF Broadcast!");
}

double offset_srs_z = 0.0;

void arm1listenAndExecute(const std::string& target_model,
                          geometry_msgs::Pose& arm1_pose,
                          geometry_msgs::Pose& arm2_pose,
                          moveit::planning_interface::MoveGroupInterface& rng_group_interface,
                          moveit::core::RobotStatePtr& kinematic_state,
                          const moveit::core::JointModelGroup* arm1_joint_model_group,
                          const moveit::core::JointModelGroup* arm2_joint_model_group,
                          const std::vector<std::string>& arm1_joint_names,
                          const std::vector<std::string>& arm2_joint_names)
{
  arm1_pose.position.z = arm1_pose.position.z - offset_srs_z - 0.0385;
  arm2_pose.position.z = arm2_pose.position.z - offset_srs_z - 0.0385;
  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                  arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);

  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform("dummy_link", target_model, ros::Time(0));
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failed to get the transform: %s",ex.what());
    }

    arm1_pose.position.x = transformStamped.transform.translation.x + 0.05;  
    arm1_pose.position.y = transformStamped.transform.translation.y - 0.1;
    arm1_pose.position.z = transformStamped.transform.translation.z + offset_srs_z;
    arm1_pose.orientation.x = 0.0024521816620031254;
    arm1_pose.orientation.y = -0.023608258310407845;
    arm1_pose.orientation.z = -0.45595118842272925;
    arm1_pose.orientation.w = 0.8896882323154255;
    
    // double offset = 0.04;
    // arm2_pose.position.x = arm1_pose.position.x - offset;
    // arm2_pose.position.y = arm1_pose.position.y - offset;
    arm2_pose.position.z = arm2_pose.position.z - offset_srs_z;
    

    std::vector<double> arm1_joint_values;
    std::vector<double> arm2_joint_values;

    bool arm1_found_ik = kinematic_state->setFromIK(arm1_joint_model_group, arm1_pose, 0.1);
    bool arm2_found_ik = kinematic_state->setFromIK(arm2_joint_model_group, arm2_pose, 0.1);

    if (arm1_found_ik && arm2_found_ik)
    {
        kinematic_state->copyJointGroupPositions(arm1_joint_model_group, arm1_joint_values);
        kinematic_state->copyJointGroupPositions(arm2_joint_model_group, arm2_joint_values);

        rng_group_interface.setJointValueTarget(arm1_joint_names, arm1_joint_values);
        rng_group_interface.setJointValueTarget(arm2_joint_names, arm2_joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (rng_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
        {
            ROS_INFO("Plan did not succeed");
            continue; 
        }

        rng_group_interface.execute(my_plan);
        
        break;
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }

  }
}

void arm2listenAndExecute(const std::string& target_model,
                          geometry_msgs::Pose& arm1_pose,
                          geometry_msgs::Pose& arm2_pose,
                          moveit::planning_interface::MoveGroupInterface& rng_group_interface,
                          moveit::core::RobotStatePtr& kinematic_state,
                          const moveit::core::JointModelGroup* arm1_joint_model_group,
                          const moveit::core::JointModelGroup* arm2_joint_model_group,
                          const std::vector<std::string>& arm1_joint_names,
                          const std::vector<std::string>& arm2_joint_names)
{
  arm1_pose.position.z = arm1_pose.position.z - offset_srs_z - 0.0385;
  arm2_pose.position.z = arm2_pose.position.z - offset_srs_z - 0.0385;
  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                  arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);

  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform("dummy_link", target_model, ros::Time(0));
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failed to get the transform: %s",ex.what());
    }
    
    arm2_pose.position.x = transformStamped.transform.translation.x + 0.05;
    arm2_pose.position.y = transformStamped.transform.translation.y - 0.1;
    arm2_pose.position.z = transformStamped.transform.translation.z + offset_srs_z;
    arm2_pose.orientation.x = -0.0020125940390603404;
    arm2_pose.orientation.y = -0.0064678588946449666;
    arm2_pose.orientation.z = 0.5783232927314214;
    arm2_pose.orientation.w = 0.8157795568355705;
    
    // double offset = 0.04;
    // arm2_pose.position.x = arm1_pose.position.x - offset;
    // arm2_pose.position.y = arm1_pose.position.y - offset;
    arm1_pose.position.z = arm1_pose.position.z - offset_srs_z;

    std::vector<double> arm1_joint_values;
    std::vector<double> arm2_joint_values;

    bool arm1_found_ik = kinematic_state->setFromIK(arm1_joint_model_group, arm1_pose, 0.1);
    bool arm2_found_ik = kinematic_state->setFromIK(arm2_joint_model_group, arm2_pose, 0.1);

    if (arm1_found_ik && arm2_found_ik)
    {
        kinematic_state->copyJointGroupPositions(arm1_joint_model_group, arm1_joint_values);
        kinematic_state->copyJointGroupPositions(arm2_joint_model_group, arm2_joint_values);

        rng_group_interface.setJointValueTarget(arm1_joint_names, arm1_joint_values);
        rng_group_interface.setJointValueTarget(arm2_joint_names, arm2_joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (rng_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
        {
            ROS_INFO("Plan did not succeed");
            continue; 
        }

        rng_group_interface.execute(my_plan);
        
        break;
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }

  }
}

void slideLeft(geometry_msgs::Pose& arm1_pose,
                geometry_msgs::Pose& arm2_pose,
                moveit::planning_interface::MoveGroupInterface& rng_group_interface,
                moveit::core::RobotStatePtr& kinematic_state,
                const moveit::core::JointModelGroup* arm1_joint_model_group,
                const moveit::core::JointModelGroup* arm2_joint_model_group,
                const std::vector<std::string>& arm1_joint_names,
                const std::vector<std::string>& arm2_joint_names)
{
  // 2 pose push to left with both arms

  arm1_pose.position.x = -1.0113947125688079;  
  arm1_pose.position.y = 0.15177069705823298;
  arm1_pose.position.z = 0.45897744227684395;
  arm1_pose.orientation.x = 0.028055443555131683;
  arm1_pose.orientation.y = 0.02667994658345721;
  arm1_pose.orientation.z = -0.45506457735317785;
  arm1_pose.orientation.w = 0.8896163796691258;
  
  arm2_pose.position.x = -0.16511375148312735;
  arm2_pose.position.y = 0.7895738623656268;
  arm2_pose.position.z = 0.416936620107478;
  arm2_pose.orientation.x = -0.0020125940390603404;
  arm2_pose.orientation.y = -0.0064678588946449666;
  arm2_pose.orientation.z = 0.5783232927314214;
  arm2_pose.orientation.w = 0.8157795568355705;

  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                    arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);
}

void spawnSRSMainModels(int num_models)
{
    ros::NodeHandle nh;
    ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    spawnClient.waitForExistence();
    ROS_INFO("Connected to /gazebo/spawn_sdf_model service!");

    // std::vector<std::string> srs_names = {"srs_main", "srs_main2", "srs_main3", "srs_main4", "srs_main5", "srs_main6"};
    // double positions[6][3] = {{-0.724398, 0.264021, 1.409962},
    //                           {-0.303509, 0.505222, 1.409962},
    //                           {0.119184, 0.755238, 1.409962},
    //                           {0.533501, 1.005666, 1.409962},
    //                           {0.930640, 1.233808, 1.409962},
    //                           {1.930640, 1.233808, 1.409962}};

    std::vector<std::string> srs_names;
    std::vector<std::vector<double>> positions;
    for (int i = 0; i < num_models; i++)
    {
        double x, y, z;

        if (i % 2)
        {
          x = -1.122129;
          y = 1.735128;
        }
        else
        {
          x = -1.584185;
          y = 1.448481;
        }

        if (i % 2 == 0) {
          z = 10.155 + (i / 2) * 0.475;  
        } 
        else {
          z = 10.155 + ((i - 1) / 2) * 0.475;
        }
        
        std::string model_name = "srs_main" + std::to_string(i + 1);

        srs_names.push_back(model_name);
        positions.push_back({x, y, z});
    }

    for (size_t i = 0; i < srs_names.size(); i++)
    {
        gazebo_msgs::SpawnModel req = createORIGINRequest(srs_names[i], 1, positions[i][0], positions[i][1], positions[i][2],
                                                         1.5707, 0.0, 0.524461, 0.2, 0.2, 0.2);

        if (spawnClient.call(req))
        {
            ROS_INFO_STREAM(srs_names[i] << " spawned successfully!");
        }
        else
        {
            ROS_ERROR_STREAM("Failed to spawn " << srs_names[i] << "!");
            return;
        }
    }
  
  for (int i = 0; i < num_models; i++)
  {
      attachModels(srs_names[i], "origin_base_link", "srs_" + std::to_string(i + 1), "base_link");
  }

}

void spawnSRSModels(int num_models)
{
  ros::NodeHandle nh;
  ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
  spawnClient.waitForExistence();
  ROS_INFO("Connected to /gazebo/spawn_sdf_model service!");

    std::vector<std::string> srs_names;
    std::vector<std::vector<double>> positions;
    for (int i = 0; i < num_models; i++)
    {
        double x, y, z;

        if (i % 2)
        {
          x = -0.122129;
          y = 0.735128;
        }
        else
        {
          x = -0.584185;
          y = 0.448481;
        }

        if (i % 2 == 0) {
          z = 0.155 + (i / 2) * 0.475;  
        } 
        else {
          z = 0.155 + ((i - 1) / 2) * 0.475;
        }
        
        std::string model_name = "srs_" + std::to_string(i + 1);

        srs_names.push_back(model_name);
        positions.push_back({x, y, z});
    }

    for (size_t i = 0; i < srs_names.size(); i++)
    {
        gazebo_msgs::SpawnModel req = createSRSRequest(srs_names[i], 0, positions[i][0], positions[i][1], positions[i][2],
                                                       1.5707, 0.0, 0.524461, 0.2, 0.2, 0.2);

        if (spawnClient.call(req))
        {
            ROS_INFO_STREAM(srs_names[i] << " spawned successfully!");
        }
        else
        {
            ROS_ERROR_STREAM("Failed to spawn " << srs_names[i] << "!");
            return;
        }
    }
  
  spawnSRSMainModels(num_models);

}

void spawnSRSMainModel(const std::string& mainModelName)
{
    ros::NodeHandle nh;
    ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    spawnClient.waitForExistence();
    ROS_INFO("Connected to /gazebo/spawn_sdf_model service!");

    // Define the positions for each main model (you can modify this as needed)
    std::vector<std::string> srs_names = {"srs_main", "srs_main2", "srs_main3", "srs_main4", "srs_main5"};
    double positions[5][3] = {{-0.724398, 0.264021, 1.409962},
                              {-0.303509, 0.505222, 1.409962},
                              {0.119184, 0.755238, 1.409962},
                              {0.533501, 1.005666, 1.409962},
                              {0.930640, 1.233808, 1.409962}};

    // Find the index of the chosen main model
    int chosenMainModelIndex = -1;
    for (size_t i = 0; i < srs_names.size(); i++)
    {
        if (srs_names[i] == mainModelName)
        {
            chosenMainModelIndex = i;
            break;
        }
    }

    // Check if the chosen main model name was found in the list
    if (chosenMainModelIndex == -1)
    {
        ROS_ERROR_STREAM("Main model '" << mainModelName << "' not found in the list!");
        return;
    }

    // Spawn the chosen main model
    gazebo_msgs::SpawnModel req = createORIGINRequest(srs_names[chosenMainModelIndex], 1,
                                                     positions[chosenMainModelIndex][0], positions[chosenMainModelIndex][1], positions[chosenMainModelIndex][2],
                                                     1.5707, 0.0, 0.524461, 0.2, 0.2, 0.2);

    if (spawnClient.call(req))
    {
        ROS_INFO_STREAM(mainModelName << " spawned successfully!");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to spawn " << mainModelName << "!");
        return;
    }
}

void spawnSRSModel(const std::string& modelName)
{
    ros::NodeHandle nh;
    ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    spawnClient.waitForExistence();
    ROS_INFO("Connected to /gazebo/spawn_sdf_model service!");

    // Define the positions for each model (you can modify this as needed)
    std::vector<std::string> srs_names = {"srs_1", "srs_2", "srs_3", "srs_4", "srs_5"};
    double positions[5][3] = {{-0.724398, 0.264021, 0.409962},
                              {-0.303509, 0.505222, 0.409962},
                              {0.119184, 0.755238, 0.409962},
                              {0.533501, 1.005666, 0.409962},
                              {0.930640, 1.233808, 0.409962}};

    // Find the index of the chosen model
    int chosenModelIndex = -1;
    for (size_t i = 0; i < srs_names.size(); i++)
    {
        if (srs_names[i] == modelName)
        {
            chosenModelIndex = i;
            break;
        }
    }

    // Check if the chosen model name was found in the list
    if (chosenModelIndex == -1)
    {
        ROS_ERROR_STREAM("Model '" << modelName << "' not found in the list!");
        return;
    }

    // Spawn the chosen model
    gazebo_msgs::SpawnModel req = createSRSRequest(srs_names[chosenModelIndex], 0,
                                                   positions[chosenModelIndex][0], positions[chosenModelIndex][1], positions[chosenModelIndex][2],
                                                   1.5707, 0.0, 0.524461, 0.2, 0.2, 0.2);

    if (spawnClient.call(req))
    {
        ROS_INFO_STREAM(modelName << " spawned successfully!");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to spawn " << modelName << "!");
        return;
    }
}

bool deleteModel(const std::string& modelName)
{
    ros::NodeHandle nh;

    ros::ServiceClient deleteClient = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    deleteClient.waitForExistence();

    gazebo_msgs::DeleteModel deleteReq;
    deleteReq.request.model_name = modelName;

    // ros::Duration(1.5).sleep();

    if (deleteClient.call(deleteReq))
    {
        if (deleteReq.response.success)
        {
            ROS_INFO_STREAM(modelName << " deleted successfully!");
            return true;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to delete " << modelName << ": " << deleteReq.response.status_message);
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to call /gazebo/delete_model service");
        return false;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coordinated_motion");

  ros::AsyncSpinner spinner(8);
  spinner.start();

  ros::NodeHandle nh;
  ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
  spawnClient.waitForExistence();
  ROS_INFO("Connected to /gazebo/spawn_sdf_model service!");

  // Spawn Origin
  gazebo_msgs::SpawnModel req0 = createORIGINRequest("origin", 0,
                                                  0.0, 0.0, 0.0,
                                                  0.0, 0.0, 0.0,  
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(req0))
  {
      ROS_INFO("Origin spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn Origin!");
      return 1;
  }

  std::vector<double> joint_values;
  moveit::planning_interface::MoveGroupInterface rng_group_interface("Arm1_and_Arm2");
  moveit::planning_interface::MoveGroupInterface arm1_group_interface("Arm_1");
  moveit::planning_interface::MoveGroupInterface arm2_group_interface("Arm_2");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  rng_group_interface.setMaxVelocityScalingFactor(1.0);
  rng_group_interface.setMaxAccelerationScalingFactor(1.0);
  rng_group_interface.setPlanningTime(15.0);
  rng_group_interface.setNumPlanningAttempts(20.0);
  
  moveit::core::RobotModelConstPtr kinematic_model = arm1_group_interface.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = rng_group_interface.getCurrentState();
  const moveit::core::JointModelGroup* arm1_joint_model_group = kinematic_model->getJointModelGroup("Arm_1");
  const moveit::core::JointModelGroup* arm2_joint_model_group = kinematic_model->getJointModelGroup("Arm_2");

  const std::vector<std::string>& arm1_joint_names = arm1_joint_model_group->getVariableNames();
  const std::vector<std::string>& arm2_joint_names = arm2_joint_model_group->getVariableNames();

  geometry_msgs::Pose arm1_pose;
  geometry_msgs::Pose arm2_pose;

  // multiple srs initial pose

  arm1_pose.position.x = -0.6800555811946246;  
  arm1_pose.position.y = 0.1227380846814348;
  arm1_pose.position.z = 0.42981645375823735;
  arm1_pose.orientation.x = 0.061381640088086734;
  arm1_pose.orientation.y = 0.009888047718481301;
  arm1_pose.orientation.z = -0.5230557498639642;
  arm1_pose.orientation.w = 0.8500277661974694;
  
  arm2_pose.position.x = 0.18729190146363395;
  arm2_pose.position.y = 0.6413787975830815;
  arm2_pose.position.z = 0.42099954672076123;
  arm2_pose.orientation.x = -0.00012303709918492825;
  arm2_pose.orientation.y = 0.0001309677261121113;
  arm2_pose.orientation.z = 0.5435901877342677;
  arm2_pose.orientation.w = 0.8393507464155558;

  // performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
  //                   arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);

  int num_models = 10; // Total number of SRS models to be spawned (should be even number!)
  spawnSRSModels(num_models);

  std::thread tf_broadcast_Thread(broadcastTf);
  startBroadcastingTf();

  // ==============================================================================================================================================

  offset_srs_z = 0.04;

  arm1listenAndExecute("srs_1", 
                        arm1_pose, arm2_pose, rng_group_interface, kinematic_state,
                        arm1_joint_model_group, arm2_joint_model_group,
                        arm1_joint_names, arm2_joint_names);

  // Attach SRS 1 to simplified_df_bot
  attachModels("simplified_df_bot", "link_06_1", "srs_1", "base_link");

  arm2listenAndExecute("srs_2", 
                        arm1_pose, arm2_pose, rng_group_interface, kinematic_state,
                        arm1_joint_model_group, arm2_joint_model_group,
                        arm1_joint_names, arm2_joint_names);

  // Attach SRS 3 to simplified_df_bot
  attachModels("simplified_df_bot", "link_06__2__1", "srs_2", "base_link");

  // ==============================================================================================================================================

  offset_srs_z = 0.0378;

  int a = 1, b = 3, c = 2, d = 4;

  for (int i = 0; i < ((num_models-2) / 2) ; i++)
  {    
    detachModels("simplified_df_bot", "link_06_1", "srs_" + std::to_string(a), "base_link");

    arm1listenAndExecute("srs_" + std::to_string(b), 
                          arm1_pose, arm2_pose, rng_group_interface, kinematic_state,
                          arm1_joint_model_group, arm2_joint_model_group,
                          arm1_joint_names, arm2_joint_names);

    attachModels("simplified_df_bot", "link_06_1", "srs_" + std::to_string(b), "base_link");

    detachModels("simplified_df_bot", "link_06__2__1", "srs_" + std::to_string(c), "base_link");

    arm2listenAndExecute("srs_" + std::to_string(d), 
                          arm1_pose, arm2_pose, rng_group_interface, kinematic_state,
                          arm1_joint_model_group, arm2_joint_model_group,
                          arm1_joint_names, arm2_joint_names);

    attachModels("simplified_df_bot", "link_06__2__1", "srs_" + std::to_string(d), "base_link");
    a += 2; 
    b += 2;
    c += 2;
    d += 2;
  }
 
  // ==============================================================================================================================================
 
  ros::shutdown();
}

