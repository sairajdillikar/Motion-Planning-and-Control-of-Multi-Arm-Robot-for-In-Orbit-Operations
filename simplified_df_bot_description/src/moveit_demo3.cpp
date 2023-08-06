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


#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>

#include <boost/thread/recursive_mutex.hpp>

#include <ignition/math/Pose3.hh>

// #include "fullbot_plugin/Attach.h"
// #include "fullbot_plugin/Detach.h"

#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"

#include <tf/transform_broadcaster.h>

#include <cstdlib>
#include <iostream>
#include <thread> 

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

        for (std::size_t i = 0; i < arm1_joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", arm1_joint_names[i].c_str(), arm1_joint_values[i]);
            ROS_INFO("Joint %s: %f", arm2_joint_names[i].c_str(), arm2_joint_values[i]);
        }
    }
    else
    {
        ROS_INFO("Did not find IK solution");
        return; // Return early if IK solution is not found
    }

    rng_group_interface.setJointValueTarget(arm1_joint_names, arm1_joint_values);
    rng_group_interface.setJointValueTarget(arm2_joint_names, arm2_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (rng_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
        ROS_INFO("Plan did not succeed");
        return; // Return early if planning fails
    }

    rng_group_interface.execute(my_plan);
}


// SDF with collision tag

// std::string getSDFString()
// {
//     return R"(
// <?xml version="1.0" ?>
// <sdf version="1.4">
//   <model name="MODELNAME">
//     <static>1</static>
//     <link name="base_link">
//       <inertial>
//         <mass>0.01</mass>
//         <inertia>
//           <ixx>0.01</ixx>
//           <ixy>0.0</ixy>
//           <ixz>0.0</ixz>
//           <iyy>0.01</iyy>
//           <iyz>0.0</iyz>
//           <izz>0.01</izz>
//         </inertia>
//       </inertial>
      
//       <collision name="collision">
//         <pose>0 0 0 0 0 0</pose>
//         <!-- <pose>4.0923159 0 2.95549512 0 0 0</pose> -->
//         <geometry>
//           <mesh>
//             <!-- <scale>0.001 0.001 0.001</scale> -->
//             <scale>0.0003 0.0003 0.0003</scale>

//             <!-- Add your PC Username here -->
//             <uri>/home/peraspera/test2_ws/src/srs_modules_description/meshes/base_link.stl</uri>
            
//           </mesh>
//         </geometry>
//       </collision>
        
//       <visual name="visual">
//         <pose>0 0 0 0 0 0</pose>
//         <!-- <pose>4.0923159 0 2.95549512 0 0 0</pose> -->
//         <geometry>
//           <mesh>
//             <!-- <scale>0.001 0.001 0.001</scale> -->
//             <scale>0.0003 0.0003 0.0003</scale>

//             <!-- Add your PC Username here -->
//             <uri>/home/peraspera/test2_ws/src/srs_modules_description/meshes/base_link.stl</uri>

//           </mesh>
//         </geometry>

//         <material>
//           <script>
//             <name>Gazebo/Silver</name>
//             <uri>file://media/materials/scripts/gazebo.material</uri>
//           </script>
//         </material>
//       </visual>
        
//     </link>
//   </model>
// </sdf>
// )";
// }



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
)";
}



gazebo_msgs::SpawnModel createSRSRequest(const std::string& modelname, int modelstatic, double px, double py, double pz,
                                         double rr, double rp, double ry, double sx, double sy, double sz)
{
    std::string cube = getSDFString();
    // Replace size of model
    std::string size_str = std::to_string(round(sx * 1000) / 1000) + " " +
                            std::to_string(round(sy * 1000) / 1000) + " " +
                            std::to_string(round(sz * 1000) / 1000);
    size_t sizePos = cube.find("SIZEXYZ");
    if (sizePos != std::string::npos)
        cube.replace(sizePos, 7, size_str);
    
    // Replace modelname
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










// void attachModels(const std::string& model_1, const std::string& link_1,
//                   const std::string& model_2, const std::string& link_2)
// {
//     ROS_INFO_STREAM("Attaching " << model_1 << " and " << model_2);
//     ros::NodeHandle nh;
//     ros::ServiceClient attachClient = nh.serviceClient<fullbot_plugin::Attach>("/fullbot_plugin/link/attach");
//     attachClient.waitForExistence();
//     fullbot_plugin::Attach srv;
//     srv.request.model_name_1 = model_1;
//     srv.request.link_name_1 = link_1;
//     srv.request.model_name_2 = model_2;
//     srv.request.link_name_2 = link_2;
//     if (attachClient.call(srv))
//     {
//         ROS_INFO("Models attached successfully!");
//     }
//     else
//     {
//         ROS_ERROR("Failed to attach models!");
//     }
// }

// void detachModels(const std::string& model_1, const std::string& link_1,
//                   const std::string& model_2, const std::string& link_2)
// {
//     ROS_INFO_STREAM("Detaching " << model_1 << " and " << model_2);
//     ros::NodeHandle nh;
//     ros::ServiceClient detachClient = nh.serviceClient<fullbot_plugin::Detach>("/fullbot_plugin/link/detach");
//     detachClient.waitForExistence();
//     fullbot_plugin::Detach srv;
//     srv.request.model_name_1 = model_1;
//     srv.request.link_name_1 = link_1;
//     // srv.request.model_name_2 = model_2;
//     // srv.request.link_name_2 = link_2;
//     if (detachClient.call(srv))
//     {
//         ROS_INFO("Models detached successfully!");
//     }
//     else
//     {
//         ROS_ERROR("Failed to detach models!");
//     }
// }







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












int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_demo");

  ros::AsyncSpinner spinner(8);
  spinner.start();

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
  std::vector<double> arm1_joint_values;
  std::vector<double> arm2_joint_values;


  // ROS_INFO("Robot Model Information:");
  // ROS_INFO("Model Frame: %s", kinematic_state->getRobotModel()->getModelFrame().c_str());

  // const std::vector<std::string>& joint_group_names = kinematic_state->getRobotModel()->getJointModelGroupNames();
  // for (const auto& group_name : joint_group_names) {
  //   ROS_INFO("Joint Group: %s", group_name.c_str());
  // }

  // ROS_INFO("Kinematic State Information:");
  // const std::vector<std::string>& joint_names = kinematic_state->getJointModelGroup("Arm1_and_Arm2")->getVariableNames();
  // for (const auto& joint_name : joint_names) {
  //   double joint_value = kinematic_state->getJointPositions(joint_name)[0];  // Assuming only one value for the joint
  //   ROS_INFO("Joint %s: %f", joint_name.c_str(), joint_value);
  // }


  std::random_device rd; 
  std::mt19937 gen(rd()); 
  std::uniform_int_distribution<> distr(-10, 10);
  std::uniform_int_distribution<> rad_distr(-30, 30);

  geometry_msgs::Pose arm1_pose;
  geometry_msgs::Pose arm2_pose;
 
  // w x y z
  // Eigen::Quaternionf arm1_q = Eigen::Quaternionf(0.0044319521005895665 , -0.0018064082028716572, 0.714190127940822, -0.6999353940485185);
  // Eigen::Quaternionf arm2_q = Eigen::Quaternionf(0.7171097271676862 , -0.6959453209354478, -0.029260144371181365, -0.02361341612136324);



  // Eigen::Quaternionf arm1_q = Eigen::Quaternionf(0.23738517897343436 , 0.11480451543556341, -0.41838517544784903, -0.8691501855288467);
  // Eigen::Quaternionf arm2_q = Eigen::Quaternionf(0.22514798408245928 , 0.00046480793594759116, -3.2008558286359785e-05, 0.974324467614685);

//   arm1_pose.position.x = -0.6333995832062274;  
//   arm1_pose.position.y = 0.3402833994692487;
//   arm1_pose.position.z = 0.5357946655060588;
//   arm1_pose.orientation.w = 0.23741857597625307;
//   arm1_pose.orientation.x = 0.114879217151039;
//   arm1_pose.orientation.y = -0.41839529430960104;
//   arm1_pose.orientation.z = -0.8691263216286542;

//   arm2_pose.position.x = -0.10133181541494289;
//   arm2_pose.position.y =  0.6603764020061327;
//   arm2_pose.position.z =  0.5351798281313073;
//   arm2_pose.orientation.w = 0.22498820948383588;
//   arm2_pose.orientation.x = 0.0007348607637888097;
//   arm2_pose.orientation.y = 2.0250570207587328e-05;
//   arm2_pose.orientation.z = 0.9743612087736406;





  // arm1_pose.position.x = ;  
  // arm1_pose.position.y = ;
  // arm1_pose.position.z = ;
  // arm1_pose.orientation.x = ;
  // arm1_pose.orientation.y = ;
  // arm1_pose.orientation.z = ;
  // arm1_pose.orientation.w = ;
  
  // arm2_pose.position.x = ;
  // arm2_pose.position.y = ;
  // arm2_pose.position.z = ;
  // arm2_pose.orientation.x = ;
  // arm2_pose.orientation.y = ;
  // arm2_pose.orientation.z = ;
  // arm2_pose.orientation.w = ;

  // performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
  //                   arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);






// Demo with 1 cube pose

  // arm1_pose.position.x = -0.30200364407805486;  
  // arm1_pose.position.y = 0.05386581619571012;
  // arm1_pose.position.z = 0.42042105470226093;
  // arm1_pose.orientation.x = 0.0001021346281704038;
  // arm1_pose.orientation.y = 3.867816255058851e-05;
  // arm1_pose.orientation.z = -0.9525643102516116;
  // arm1_pose.orientation.w = 0.30433735049676164;
  
  // arm2_pose.position.x = 0.07362006339415605;
  // arm2_pose.position.y =  0.33175185044468136;
  // arm2_pose.position.z =  0.4190627390092047;
  // arm2_pose.orientation.x = 6.431347388061696e-05;
  // arm2_pose.orientation.y = -2.5384487601610648e-05;
  // arm2_pose.orientation.z = 0.9746014969814294;
  // arm2_pose.orientation.w = 0.22394623752356715;

// Demo with 2 cube pose

  // arm1_pose.position.x = -0.49821158233271146;  
  // arm1_pose.position.y = -0.08576562129825285;
  // arm1_pose.position.z = 0.4203978643405997;
  // arm1_pose.orientation.x = 0.00018967830771923305;
  // arm1_pose.orientation.y = 6.631644126205693e-05;
  // arm1_pose.orientation.z = -0.9525603621318198;
  // arm1_pose.orientation.w = 0.3043496609486617;
  
  // arm2_pose.position.x = 0.2609951639102981;
  // arm2_pose.position.y = 0.45795618286334694;
  // arm2_pose.position.z =  0.4189409777514546;
  // arm2_pose.orientation.x = 1.5472834980139237e-05;
  // arm2_pose.orientation.y = -0.000588373511613903;
  // arm2_pose.orientation.z = 0.9747192928368752;
  // arm2_pose.orientation.w = 0.2234322128717884;

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

  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                    arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);

  




  ros::NodeHandle nh;
  ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
  spawnClient.waitForExistence();
  ROS_INFO("Connected to /gazebo/spawn_sdf_model service!");

  // Spawn SRS 1
  gazebo_msgs::SpawnModel req1 = createSRSRequest("srs_1", 0,
                                                  -0.724398, 0.264021, 0.409962,
                                                  1.5707, 0.0, 0.524461,  // rotation
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(req1))
  {
      ROS_INFO("SRS 1 spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn SRS 1!");
      return 1;
  }

// Spawn SRS 2
  gazebo_msgs::SpawnModel req2 = createSRSRequest("srs_2", 0,
                                                  -0.303509, 0.505222, 0.409962,
                                                  1.5707, 0.0, 0.524461,  // rotation
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(req2))
  {
      ROS_INFO("SRS 2 spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn SRS 2!");
      return 1;
  }

  // Spawn SRS 3
  gazebo_msgs::SpawnModel req3 = createSRSRequest("srs_3", 0,
                                                  0.119184, 0.755238, 0.409962,
                                                  1.5707, 0.0, 0.524461,  // rotation
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(req3))
  {
      ROS_INFO("SRS 3 spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn SRS 3!");
      return 1;
  }

  // Spawn SRS 4
  gazebo_msgs::SpawnModel req4 = createSRSRequest("srs_4", 0,
                                                  0.533501, 1.005666, 0.409962,
                                                  1.5707, 0.0, 0.524461,  // rotation
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(req4))
  {
      ROS_INFO("SRS 4 spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn SRS 4!");
      return 1;
  }

  // Spawn SRS 5
  gazebo_msgs::SpawnModel req5 = createSRSRequest("srs_5", 0,
                                                  0.930640, 1.233808, 0.409962,
                                                  1.5707, 0.0, 0.524461,  // rotation
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(req5))
  {
      ROS_INFO("SRS 5 spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn SRS 5!");
      return 1;
  }

  // Spawn SRS main
  gazebo_msgs::SpawnModel reqmain = createSRSRequest("srs_main", 1,
                                                  -0.287279, 1.160372, 100.062,
                                                  1.5707, 0.0, 0.524461,  // rotation
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(reqmain))
  {
      ROS_INFO("SRS main spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn SRS main!");
      return 1;
  }

  // Spawn SRS main2
  gazebo_msgs::SpawnModel reqmain2 = createSRSRequest("srs_main2", 1,
                                                  -1.287279, 1.160372, 100.062,
                                                  1.5707, 0.0, 0.524461,  // rotation
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(reqmain2))
  {
      ROS_INFO("SRS main2 spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn SRS main2!");
      return 1;
  }

  // Spawn SRS main3
  gazebo_msgs::SpawnModel reqmain3 = createSRSRequest("srs_main3", 1,
                                                  -2.287279, 1.160372, 100.062,
                                                  1.5707, 0.0, 0.524461,  // rotation
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(reqmain3))
  {
      ROS_INFO("SRS main3 spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn SRS main3!");
      return 1;
  }

  // Spawn SRS main4
  gazebo_msgs::SpawnModel reqmain4 = createSRSRequest("srs_main4", 1,
                                                  -1.287279, 2.160372, 100.062,
                                                  1.5707, 0.0, 0.524461,  // rotation
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(reqmain4))
  {
      ROS_INFO("SRS main4 spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn SRS main4!");
      return 1;
  }

  // Spawn SRS main5
  gazebo_msgs::SpawnModel reqmain5 = createSRSRequest("srs_main5", 1,
                                                  -1.287279, 3.160372, 100.062,
                                                  1.5707, 0.0, 0.524461,  // rotation
                                                  0.2, 0.2, 0.2);
  if (spawnClient.call(reqmain5))
  {
      ROS_INFO("SRS main5 spawned successfully!");
  }
  else
  {
      ROS_ERROR("Failed to spawn SRS main5!");
      return 1;
  }

  attachModels("srs_main", "base_link", "srs_1", "base_link");

  attachModels("srs_main2", "base_link", "srs_2", "base_link");

  attachModels("srs_main3", "base_link", "srs_3", "base_link");

  attachModels("srs_main4", "base_link", "srs_4", "base_link");

  attachModels("srs_main5", "base_link", "srs_5", "base_link");



  // Attach SRS 1 to simplified_df_bot
  attachModels("simplified_df_bot", "link_06_1", "srs_1", "base_link");

  // Attach SRS 3 to simplified_df_bot
  attachModels("simplified_df_bot", "link_06__2__1", "srs_3", "base_link");

  // // // // Detach SRS 1 from simplified_df_bot
  // detachModels("simplified_df_bot", "link_06_1", "srs_1", "base_link");

  // // Detach SRS 3 from simplified_df_bot
  // detachModels("simplified_df_bot", "link_06__2__1", "srs_3", "base_link");




  // const char* command_attach = "rosrun simplified_df_bot_description attachnow.py";

  // // // Execute the command using system()
  // int result_attach = std::system(command_attach);

  // system("rosrun simplified_df_bot_description attach_test.py");



// // push back the srs

  arm1_pose.position.x = -0.8123470181081331;  
  arm1_pose.position.y = 0.37403855039645784;
  arm1_pose.position.z = 0.4545125204166259;
  arm1_pose.orientation.x = 0.06290642299271615;
  arm1_pose.orientation.y = 0.011420050918326579;
  arm1_pose.orientation.z = -0.5234972421663993;
  arm1_pose.orientation.w = 0.8496252125657869;
  
  arm2_pose.position.x = 0.028664112728951627;
  arm2_pose.position.y = 0.8658433039124054;
  arm2_pose.position.z = 0.4225482615931009;
  arm2_pose.orientation.x = 0.0017500727106523185;
  arm2_pose.orientation.y = 0.004527432685522351;
  arm2_pose.orientation.z = 0.5414117747753332;
  arm2_pose.orientation.w = 0.8407435576520402;

  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                    arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);




// // // 1 pose push to left with both arms NOT THIS!

// //   arm1_pose.position.x = -0.8654920744857423;  
// //   arm1_pose.position.y = 0.2572661816715699;
// //   arm1_pose.position.z = 0.4640402849168767;
// //   arm1_pose.orientation.x = 0.025201810364633988;
// //   arm1_pose.orientation.y = 0.030509050029797535;
// //   arm1_pose.orientation.z = -0.45282310075196125;
// //   arm1_pose.orientation.w = 0.8907217893629882;
  
// //   arm2_pose.position.x = -0.014664755887961706;
// //   arm2_pose.position.y = 0.9015618439061583;
// //   arm2_pose.position.z = 0.42172213755003524;
// //   arm2_pose.orientation.x = 0.0002823263919096938;
// //   arm2_pose.orientation.y = 0.001084697155911538;
// //   arm2_pose.orientation.z = 0.5712144201772784;
// //   arm2_pose.orientation.w = 0.8208001156831206;

// //   performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
// //                     arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);



// // 2 pose push to left with both arms

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


  // Detach SRS 1 from simplified_df_bot
  detachModels("simplified_df_bot", "link_06_1", "srs_1", "base_link");





// // Arm1 to SRS2

  arm1_pose.position.x = -0.5533311968619872;  
  arm1_pose.position.y = 0.48578424549670635;
  arm1_pose.position.z = 0.4276538178099182;
  arm1_pose.orientation.x = 0.02814398313273;
  arm1_pose.orientation.y = 0.026665870782599885;
  arm1_pose.orientation.z = -0.45518307871677083;
  arm1_pose.orientation.w = 0.8895533780492053;

  arm2_pose.position.x = -0.16511375148312735;
  arm2_pose.position.y = 0.7895738623656268;
  arm2_pose.position.z = 0.416936620107478;
  arm2_pose.orientation.x = -0.0020125940390603404;
  arm2_pose.orientation.y = -0.0064678588946449666;
  arm2_pose.orientation.z = 0.5783232927314214;
  arm2_pose.orientation.w = 0.8157795568355705;

  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                    arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);


// Attach SRS 2 to simplified_df_bot
  attachModels("simplified_df_bot", "link_06_1", "srs_2", "base_link");

  // // Add a delay of 2 seconds (2000 milliseconds)
  // std::chrono::milliseconds delayDuration(3000);
  // std::this_thread::sleep_for(delayDuration);


  // // Detach SRS 3 from simplified_df_bot
  detachModels("simplified_df_bot", "link_06__2__1", "srs_3", "base_link");




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

  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                    arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);





  // Attach SRS 3 to simplified_df_bot
  attachModels("simplified_df_bot", "link_06__2__1", "srs_4", "base_link");

// // push back the srs

  arm1_pose.position.x = -0.8123470181081331;  
  arm1_pose.position.y = 0.37403855039645784;
  arm1_pose.position.z = 0.4545125204166259;
  arm1_pose.orientation.x = 0.06290642299271615;
  arm1_pose.orientation.y = 0.011420050918326579;
  arm1_pose.orientation.z = -0.5234972421663993;
  arm1_pose.orientation.w = 0.8496252125657869;
  
  arm2_pose.position.x = 0.028664112728951627;
  arm2_pose.position.y = 0.8658433039124054;
  arm2_pose.position.z = 0.4225482615931009;
  arm2_pose.orientation.x = 0.0017500727106523185;
  arm2_pose.orientation.y = 0.004527432685522351;
  arm2_pose.orientation.z = 0.5414117747753332;
  arm2_pose.orientation.w = 0.8407435576520402;

  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                    arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);





// // 2 pose push to left with both arms

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



  // Detach SRS 2 from simplified_df_bot
  detachModels("simplified_df_bot", "link_06_1", "srs_2", "base_link");





// // Arm1 to SRS3

  arm1_pose.position.x = -0.5533311968619872;  
  arm1_pose.position.y = 0.48578424549670635;
  arm1_pose.position.z = 0.4276538178099182;
  arm1_pose.orientation.x = 0.02814398313273;
  arm1_pose.orientation.y = 0.026665870782599885;
  arm1_pose.orientation.z = -0.45518307871677083;
  arm1_pose.orientation.w = 0.8895533780492053;

  arm2_pose.position.x = -0.16511375148312735;
  arm2_pose.position.y = 0.7895738623656268;
  arm2_pose.position.z = 0.416936620107478;
  arm2_pose.orientation.x = -0.0020125940390603404;
  arm2_pose.orientation.y = -0.0064678588946449666;
  arm2_pose.orientation.z = 0.5783232927314214;
  arm2_pose.orientation.w = 0.8157795568355705;

  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                    arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);


// Attach SRS 2 to simplified_df_bot
  attachModels("simplified_df_bot", "link_06_1", "srs_3", "base_link");

  // // Add a delay of 2 seconds (2000 milliseconds)
  // std::chrono::milliseconds delayDuration(3000);
  // std::this_thread::sleep_for(delayDuration);


  // // Detach SRS 3 from simplified_df_bot
  detachModels("simplified_df_bot", "link_06__2__1", "srs_4", "base_link");




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

  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                    arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);





  // Attach SRS 3 to simplified_df_bot
  attachModels("simplified_df_bot", "link_06__2__1", "srs_5", "base_link");

// // push back the srs

  arm1_pose.position.x = -0.8123470181081331;  
  arm1_pose.position.y = 0.37403855039645784;
  arm1_pose.position.z = 0.4545125204166259;
  arm1_pose.orientation.x = 0.06290642299271615;
  arm1_pose.orientation.y = 0.011420050918326579;
  arm1_pose.orientation.z = -0.5234972421663993;
  arm1_pose.orientation.w = 0.8496252125657869;
  
  arm2_pose.position.x = 0.028664112728951627;
  arm2_pose.position.y = 0.8658433039124054;
  arm2_pose.position.z = 0.4225482615931009;
  arm2_pose.orientation.x = 0.0017500727106523185;
  arm2_pose.orientation.y = 0.004527432685522351;
  arm2_pose.orientation.z = 0.5414117747753332;
  arm2_pose.orientation.w = 0.8407435576520402;

  performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
                    arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);

























  // const char* command_detach = "rosrun simplified_df_bot_description detachnow.py";

  // // Execute the command using system()
  // int result_detach = std::system(command_detach);


// multiple srs initial pose

  // arm1_pose.position.x = -0.6800555811946246;  
  // arm1_pose.position.y = 0.1227380846814348;
  // arm1_pose.position.z = 0.42981645375823735;
  // arm1_pose.orientation.x = 0.061381640088086734;
  // arm1_pose.orientation.y = 0.009888047718481301;
  // arm1_pose.orientation.z = -0.5230557498639642;
  // arm1_pose.orientation.w = 0.8500277661974694;
  
  // arm2_pose.position.x = 0.18729190146363395;
  // arm2_pose.position.y = 0.6413787975830815;
  // arm2_pose.position.z = 0.42099954672076123;
  // arm2_pose.orientation.x = -0.00012303709918492825;
  // arm2_pose.orientation.y = 0.0001309677261121113;
  // arm2_pose.orientation.z = 0.5435901877342677;
  // arm2_pose.orientation.w = 0.8393507464155558;

  // performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
  //                   arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);










  // moveit_msgs::CollisionObject object_to_attach;
  // // object_to_attach.header.frame_id = arm1_group_interface.getPlanningFrame();
  // object_to_attach.id = "box1";

  // // Define a box to add to the world.
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[primitive.BOX_X] = 0.1;
  // primitive.dimensions[primitive.BOX_Y] = 1.5;
  // primitive.dimensions[primitive.BOX_Z] = 0.5;

  // object_to_attach.header.frame_id = arm1_group_interface.getEndEffectorLink();
  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 3.5;
  // box_pose.position.y = 3.0;
  // box_pose.position.z = 0.25;

  // object_to_attach.primitives.push_back(primitive);
  // object_to_attach.primitive_poses.push_back(box_pose);
  // object_to_attach.operation = object_to_attach.ADD;
  // planning_scene_interface.applyCollisionObject(object_to_attach);
  
  // // arm1_group_interface.attachObject(object_to_attach.id, "link_06_1");



//   moveit_msgs::CollisionObject object_to_attach;
//   object_to_attach.id = "cylinder1";

//   shape_msgs::SolidPrimitive cylinder_primitive;
//   cylinder_primitive.type = primitive.CYLINDER;
//   cylinder_primitive.dimensions.resize(2);
//   cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
//   cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

//   // We define the frame/pose for this cylinder so that it appears in the gripper
//   object_to_attach.header.frame_id = arm1_group_interface.getEndEffectorLink();
//   geometry_msgs::Pose grab_pose;
//   grab_pose.orientation.w = 1.0;
//   grab_pose.position.z = 0.2;

//   // First, we add the object to the world (without using a vector)
//   object_to_attach.primitives.push_back(cylinder_primitive);
//   object_to_attach.primitive_poses.push_back(grab_pose);
//   object_to_attach.operation = object_to_attach.ADD;
//   planning_scene_interface.applyCollisionObject(object_to_attach);



  // for(int i; i < 30; i++){

  //   float random_x = ( ((float) distr(gen)) * 0.005);
  //   float random_y = ( ((float) distr(gen)) * 0.005);
  //   float random_z = ( ((float) distr(gen)) * 0.005);

  //   arm1_pose.position.x = -0.4123891794664267 + random_x;  
  //   arm1_pose.position.y = 0.4812721410480939 + random_y;
  //   arm1_pose.position.z = 0.5010054793908278 + random_z;
  //   arm2_pose.position.x = -0.24922596187320153 + random_x;
  //   arm2_pose.position.y = 0.5832570486483674 + random_y;
  //   arm2_pose.position.z = 0.5003571141282077 + random_z;

  //   arm1_pose.orientation.w = 0.23738517897343436;
  //   arm1_pose.orientation.x = 0.11480451543556341;
  //   arm1_pose.orientation.y = -0.41838517544784903;
  //   arm1_pose.orientation.z = -0.8691501855288467;
  //   arm2_pose.orientation.w = 0.22514798408245928;
  //   arm2_pose.orientation.x = 0.00046480793594759116;
  //   arm2_pose.orientation.y = -3.2008558286359785e-05;
  //   arm2_pose.orientation.z = 0.974324467614685;


  //   performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
  //                   arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);


  //   // double timeout = 0.1;
  //   // bool arm1_found_ik = kinematic_state->setFromIK(arm1_joint_model_group, arm1_pose, timeout);
  //   // bool arm2_found_ik = kinematic_state->setFromIK(arm2_joint_model_group, arm2_pose, timeout);

  //   // if (arm1_found_ik && arm2_found_ik)
  //   // {
  //   //   kinematic_state->copyJointGroupPositions(arm1_joint_model_group, arm1_joint_values);
  //   //   kinematic_state->copyJointGroupPositions(arm2_joint_model_group, arm2_joint_values);

  //   //   for (std::size_t i = 0; i < arm1_joint_names.size(); ++i)
  //   //   {
  //   //     ROS_INFO("Joint %s: %f", arm1_joint_names[i].c_str(), arm1_joint_values[i]);
  //   //     ROS_INFO("Joint %s: %f", arm2_joint_names[i].c_str(), arm2_joint_values[i]);
  //   //   }
  //   // }
  //   // else
  //   // {
  //   //   ROS_INFO("Did not find IK solution");
  //   // }
    
  //   // rng_group_interface.setJointValueTarget(arm1_joint_names, arm1_joint_values);
  //   // rng_group_interface.setJointValueTarget(arm2_joint_names, arm2_joint_values);

  //   // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //   // bool success = (rng_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //   // if(!success){
  //   //   ROS_INFO("Plan did not successed");
  //   // }
  //   // rng_group_interface.execute(my_plan);


  // }


  //   float random_x = ( ((float) distr(gen)) * 0.005);
  //   float random_y = ( ((float) distr(gen)) * 0.005);
  //   float random_z = ( ((float) distr(gen)) * 0.005);

  //   arm1_pose.position.x = -0.4123891794664267 + random_x;  
  //   arm1_pose.position.y = 0.4812721410480939 + random_y;
  //   arm1_pose.position.z = 0.5010054793908278 + random_z;
  //   arm2_pose.position.x = -0.24922596187320153 + random_x;
  //   arm2_pose.position.y = 0.5832570486483674 + random_y;
  //   arm2_pose.position.z = 0.5003571141282077 + random_z;

  //   arm1_pose.orientation.w = 0.23738517897343436;
  //   arm1_pose.orientation.x = 0.11480451543556341;
  //   arm1_pose.orientation.y = -0.41838517544784903;
  //   arm1_pose.orientation.z = -0.8691501855288467;
  //   arm2_pose.orientation.w = 0.22514798408245928;
  //   arm2_pose.orientation.x = 0.00046480793594759116;
  //   arm2_pose.orientation.y = -3.2008558286359785e-05;
  //   arm2_pose.orientation.z = 0.974324467614685;


  //   performIKandExecute(rng_group_interface, kinematic_state, arm1_joint_model_group, arm2_joint_model_group,
  //                   arm1_joint_names, arm2_joint_names, arm1_pose, arm2_pose, 0.1);


  //   // double timeout = 0.1;
  //   // bool arm1_found_ik = kinematic_state->setFromIK(arm1_joint_model_group, arm1_pose, timeout);
  //   // bool arm2_found_ik = kinematic_state->setFromIK(arm2_joint_model_group, arm2_pose, timeout);

  //   // if (arm1_found_ik && arm2_found_ik)
  //   // {
  //   //   kinematic_state->copyJointGroupPositions(arm1_joint_model_group, arm1_joint_values);
  //   //   kinematic_state->copyJointGroupPositions(arm2_joint_model_group, arm2_joint_values);

  //   //   for (std::size_t i = 0; i < arm1_joint_names.size(); ++i)
  //   //   {
  //   //     ROS_INFO("Joint %s: %f", arm1_joint_names[i].c_str(), arm1_joint_values[i]);
  //   //     ROS_INFO("Joint %s: %f", arm2_joint_names[i].c_str(), arm2_joint_values[i]);
  //   //   }
  //   // }
  //   // else
  //   // {
  //   //   ROS_INFO("Did not find IK solution");
  //   // }
    
  //   // rng_group_interface.setJointValueTarget(arm1_joint_names, arm1_joint_values);
  //   // rng_group_interface.setJointValueTarget(arm2_joint_names, arm2_joint_values);

  //   // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //   // bool success = (rng_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //   // if(!success){
  //   //   ROS_INFO("Plan did not successed");
  //   // }
  //   // rng_group_interface.execute(my_plan);


  // }

  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("link_00_1");
  // visual_tools.deleteAllMarkers();
  // visual_tools.loadRemoteControl();
  // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.0;
  // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();


//   geometry_msgs::PoseStamped arm1_current_pose = arm1_group_interface.getCurrentPose("link_06_1");
//   geometry_msgs::Pose arm1_pose = arm1_current_pose.pose;
  

//   // arm1_pose.position.x = -1.3805809999999998;  
//   // arm1_pose.position.y = 0.001;
//   // arm1_pose.position.z = 0.1 ;
//   // arm1_pose.orientation.w = 0.0;
//   // arm1_pose.orientation.x = 0.0;
//   // arm1_pose.orientation.y = 0.0;
//   // arm1_pose.orientation.z = 1.0;

//   std::vector<geometry_msgs::Pose> waypoints;
//   waypoints.push_back(arm1_pose);

//   geometry_msgs::Pose Circ = arm1_pose;

//   Circ.position.x = -1.0;
//   Circ.position.y = 0.0;
//   Circ.position.z = 0.5;
//   waypoints.push_back(Circ);


// //   Point Center(-0.5,0);
// //   double radius = 0.24;
// //   const double PI = 3.14159;

// //   for (double angle=0; angle<=2*PI; angle+=0.02)
// //   {
// //     geometry_msgs::Pose Circ = arm1_pose;

// //     Circ.position.x += Center.z + radius *cos(angle);
// //     Circ.position.y += radius*sin(angle);

// //     waypoints.push_back(C);
// //   }

    

//   moveit_msgs::RobotTrajectory trajectory;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;
//   double fraction = arm1_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

//   // // Visualize the plan in RViz
//   // visual_tools.deleteAllMarkers();
//   // visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
//   // visual_tools.publishTrajectoryLine(trajectory, arm1_joint_model_group);
//   // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//   // for (std::size_t i = 0; i < waypoints.size(); ++i)
//   //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
//   // visual_tools.trigger();
//   // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   arm1_group_interface.execute(trajectory);



  ros::shutdown();
}































