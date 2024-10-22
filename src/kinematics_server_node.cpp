#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <memory>

#include "gramps_kd/kinematics_server.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinematics_node");

  ros::NodeHandle nh;

  std::string robot_desc_topic;
  if (!nh.getParam("robot_description", robot_desc_topic)) {
    ROS_ERROR("No robot description parameter!");
    return 1;
  }

  auto urdf_model = std::make_shared<urdf::Model>();
  if (!urdf_model->initParamWithNodeHandle(robot_desc_topic, nh)) {
    ROS_ERROR("Failed to parse URDF model");
    return 1;
  }

  gramps_kd::KinematicsServer kinematics_server(urdf_model);

  ros::ServiceServer service =
      nh.advertiseService("forward_kinematics", &gramps_kd::KinematicsServer::forward_kinematics, &kinematics_server);
  ros::spin();

  return 0;
}
