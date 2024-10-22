#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <memory>

#include "gramps_kd/kinematics_server.h"

using namespace gramps_kd;

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinematics_node");

  ros::NodeHandle nh;

  std::string robot_description;
  if (!nh.getParam("robot_description", robot_description)) {
    ROS_ERROR("No robot description found");
    return 1;
  }

  auto urdf_model = std::make_shared<urdf::Model>();
  if (!urdf_model->initString(robot_description)) {
    ROS_ERROR("Failed to parse URDF model");
    return 1;
  }

  IKParams ik_params{};
  ik_params.eps = nh.param("eps", 1e-4);
  ik_params.it_max = nh.param("it_max", 1000);
  ik_params.dt = nh.param("dt", 1e-1);
  ik_params.damping = nh.param("damping", 1e-6);

  KinematicsServer kinematics_server(urdf_model, ik_params);
  auto fk = nh.advertiseService("forward_kinematics", &KinematicsServer::forward_kinematics, &kinematics_server);
  auto ik = nh.advertiseService("inverse_kinematics", &KinematicsServer::inverse_kinematics, &kinematics_server);

  ros::spin();

  return 0;
}
