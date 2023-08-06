// #include <ros/ros.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <gazebo_msgs/GetModelState.h>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "model_tf_broadcaster");
//   ros::NodeHandle nh;

//   // Create a TransformBroadcaster to publish the transform
//   tf2_ros::TransformBroadcaster tfBroadcaster;

//   // Set the rate at which to broadcast the transform (e.g., 10 Hz)
//   ros::Rate rate(10.0);

//   while (ros::ok())
//   {
//     // Create a service client to get the model's state from Gazebo
//     ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

//     // Create a request to get the model's state
//     gazebo_msgs::GetModelState modelStateRequest;
//     modelStateRequest.request.model_name = "srs_1"; // Replace with the actual name of your model

//     // Call the service to get the model's state
//     if (client.call(modelStateRequest))
//     {
//       // Create a transform message
//       geometry_msgs::TransformStamped transformStamped;
//       transformStamped.header.stamp = ros::Time::now();
//       transformStamped.header.frame_id = "dummy_link"; // Replace with the actual frame ID of the world
//       transformStamped.child_frame_id = "srs_1"; // Replace with the desired frame ID for your model
//       transformStamped.transform.translation.x = modelStateRequest.response.pose.position.x;
//       transformStamped.transform.translation.y = modelStateRequest.response.pose.position.y;
//       transformStamped.transform.translation.z = modelStateRequest.response.pose.position.z;
//       transformStamped.transform.rotation = modelStateRequest.response.pose.orientation;

//       // Broadcast the transform
//       tfBroadcaster.sendTransform(transformStamped);
//     }
//     else
//     {
//       ROS_WARN("Failed to get model state from Gazebo");
//     }

//     // Sleep to maintain the desired rate
//     rate.sleep();
//   }

//   return 0;
// }




#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_tf_broadcaster");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster tfBroadcaster;

  ros::Rate rate(10.0);

  while (ros::ok())
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

  return 0;
}









// #include <ros/ros.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <gazebo_msgs/GetModelState.h>
// #include <gazebo_msgs/GetWorldProperties.h>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "model_tf_broadcaster");
//   ros::NodeHandle nh;

//   tf2_ros::TransformBroadcaster tfBroadcaster;

//   ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

//   gazebo_msgs::GetWorldProperties worldPropertiesRequest;

//   if (client.call(worldPropertiesRequest))
//   {
//     std::vector<std::string>& modelNames = worldPropertiesRequest.response.model_names;

//     int loopCount = 0; // Counter variable

//     while (loopCount < 10)
//     {
//       for (const std::string& modelName : modelNames)
//       {
//         ros::ServiceClient getStateClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

//         gazebo_msgs::GetModelState modelStateRequest;
//         modelStateRequest.request.model_name = modelName;
//         modelStateRequest.request.relative_entity_name = "dummy_link";

//         if (getStateClient.call(modelStateRequest))
//         {
//           geometry_msgs::TransformStamped transformStamped;
//           transformStamped.header.stamp = ros::Time::now();
//           transformStamped.header.frame_id = "dummy_link"; 
//           transformStamped.child_frame_id = modelName; 
//           transformStamped.transform.translation.x = modelStateRequest.response.pose.position.x;
//           transformStamped.transform.translation.y = modelStateRequest.response.pose.position.y;
//           transformStamped.transform.translation.z = modelStateRequest.response.pose.position.z;
//           transformStamped.transform.rotation = modelStateRequest.response.pose.orientation;

//           tfBroadcaster.sendTransform(transformStamped);
//         }
//         else
//         {
//           ROS_WARN_STREAM("Failed to get model state for " << modelName << " from Gazebo");
//         }
//       }

//       loopCount++;
//     }
//   }
//   else
//   {
//     ROS_WARN("Failed to get list of model properties from Gazebo");
//   }

//   return 0;
// }

