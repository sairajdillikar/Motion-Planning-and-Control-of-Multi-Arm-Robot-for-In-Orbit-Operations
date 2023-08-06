#include <ros/ros.h>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_demo");

  ros::AsyncSpinner spinner(8);
  spinner.start();

  std::vector<double> joint_values;
  moveit::planning_interface::MoveGroupInterface rng_group_interface("Arm1_and_Arm2");
  moveit::planning_interface::MoveGroupInterface arm1_group_interface("Arm_1");
  moveit::planning_interface::MoveGroupInterface arm2_group_interface("Arm_2");

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

  std::random_device rd; 
  std::mt19937 gen(rd()); 
  std::uniform_int_distribution<> distr(-10, 10);
  std::uniform_int_distribution<> rad_distr(-30, 30);

  geometry_msgs::Pose arm1_pose;
  geometry_msgs::Pose arm2_pose;
 
  // w x y z
  Eigen::Quaternionf arm1_q = Eigen::Quaternionf(0.23738517897343436 , 0.11480451543556341, -0.41838517544784903, -0.8691501855288467);
  Eigen::Quaternionf arm2_q = Eigen::Quaternionf(0.22514798408245928 , 0.00046480793594759116, -3.2008558286359785e-05, 0.974324467614685);

  for(int i; i < 30; i++){

    float random_x = ( ((float) distr(gen)) * 0.001);
    float random_y = ( ((float) distr(gen)) * 0.001);
    float random_z = ( ((float) distr(gen)) * 0.001);

    // arm1_pose.position.x = -0.4123891794664267 + random_x;  
    // arm1_pose.position.y = 0.4812721410480939 + random_y;
    // arm1_pose.position.z = 0.5010054793908278 + random_z;
    // arm2_pose.position.x = -0.24922596187320153 + random_x;
    // arm2_pose.position.y = 0.5832570486483674 + random_y;
    // arm2_pose.position.z = 0.5003571141282077 + random_z;

    // // float x_rotation = rad_distr(gen) * 0.01;
    // // float y_rotation = rad_distr(gen) * 0.01;
    // // float z_rotation = rad_distr(gen) * 0.01;

    // // arm1_q = Eigen::AngleAxisf(x_rotation, Eigen::Vector3f::UnitX()) *
    // //             Eigen::AngleAxisf(y_rotation, Eigen::Vector3f::UnitY()) *
    // //             Eigen::AngleAxisf(z_rotation, Eigen::Vector3f::UnitZ()) *
    // //             arm1_q;

    // // arm2_q = Eigen::AngleAxisf(x_rotation, Eigen::Vector3f::UnitX()) *
    // //             Eigen::AngleAxisf(y_rotation, Eigen::Vector3f::UnitY()) *
    // //             Eigen::AngleAxisf(z_rotation, Eigen::Vector3f::UnitZ()) *
    // //             arm2_q;

    // // arm1_pose.orientation.w = arm1_q.w();
    // // arm1_pose.orientation.x = arm1_q.x();
    // // arm1_pose.orientation.y = arm1_q.y();
    // // arm1_pose.orientation.z = arm1_q.z();
    // // arm2_pose.orientation.w = arm2_q.w();
    // // arm2_pose.orientation.x = arm2_q.x();
    // // arm2_pose.orientation.y = arm2_q.y();
    // // arm2_pose.orientation.z = arm2_q.z();

    // arm1_pose.orientation.w = 0.23738517897343436;
    // arm1_pose.orientation.x = 0.11480451543556341;
    // arm1_pose.orientation.y = -0.41838517544784903;
    // arm1_pose.orientation.z = -0.8691501855288467;
    // arm2_pose.orientation.w = 0.22514798408245928;
    // arm2_pose.orientation.x = 0.00046480793594759116;
    // arm2_pose.orientation.y = -3.2008558286359785e-05;
    // arm2_pose.orientation.z = 0.974324467614685;


  // Demo with 1 cube pose

    // arm1_pose.position.x = -0.30200364407805486 + random_x;  
    // arm1_pose.position.y = 0.05386581619571012 + random_y;
    // arm1_pose.position.z = 0.42042105470226093 + random_z;
    // arm1_pose.orientation.x = 0.0001021346281704038;
    // arm1_pose.orientation.y = 3.867816255058851e-05;
    // arm1_pose.orientation.z = -0.9525643102516116;
    // arm1_pose.orientation.w = 0.30433735049676164;
    
    // arm2_pose.position.x = 0.07362006339415605 + random_x;
    // arm2_pose.position.y =  0.33175185044468136 + random_y;
    // arm2_pose.position.z =  0.4190627390092047 + random_z;
    // arm2_pose.orientation.x = 6.431347388061696e-05;
    // arm2_pose.orientation.y = -2.5384487601610648e-05;
    // arm2_pose.orientation.z = 0.9746014969814294;
    // arm2_pose.orientation.w = 0.22394623752356715;


  // Demo with 2 cube pose

    arm1_pose.position.x = -0.49821158233271146 + random_x;  
    arm1_pose.position.y = -0.08576562129825285 + random_y;
    arm1_pose.position.z = 0.4203978643405997 + random_z;
    arm1_pose.orientation.x = 0.00018967830771923305;
    arm1_pose.orientation.y = 6.631644126205693e-05;
    arm1_pose.orientation.z = -0.9525603621318198;
    arm1_pose.orientation.w = 0.3043496609486617;
    
    arm2_pose.position.x = 0.2609951639102981 + random_x;
    arm2_pose.position.y = 0.45795618286334694 + random_y;
    arm2_pose.position.z =  0.4189409777514546 + random_z;
    arm2_pose.orientation.x = 1.5472834980139237e-05;
    arm2_pose.orientation.y = -0.000588373511613903;
    arm2_pose.orientation.z = 0.9747192928368752;
    arm2_pose.orientation.w = 0.2234322128717884;





    double timeout = 0.1;
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
    }
    
    rng_group_interface.setJointValueTarget(arm1_joint_names, arm1_joint_values);
    rng_group_interface.setJointValueTarget(arm2_joint_names, arm2_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (rng_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success){
      ROS_INFO("Plan did not successed");
    }
    rng_group_interface.execute(my_plan);



    
  }

  ros::shutdown();
}