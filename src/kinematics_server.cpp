#include "gramps_kd/kinematics_server.h"

// clang-format off
// pinocchio includes have to go before urdf for some reason
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <urdf/model.h>
// clang-format on

namespace pin = pinocchio;

namespace gramps_kd {

KinematicsServer::KinematicsServer(std::shared_ptr<urdf::Model> urdf_model)
    : urdf_model_(urdf_model), model_(std::make_unique<pin::Model>()) {
  pin::urdf::buildModel(urdf_model_, *model_);
  data_ = std::make_unique<pin::Data>(*model_);
}

KinematicsServer::~KinematicsServer() = default;

bool KinematicsServer::forward_kinematics(gramps_kd::ForwardKinematics::Request &req,
                                          gramps_kd::ForwardKinematics::Response &res) {
  // Forward kinematics code here
  Eigen::Map<Eigen::VectorXd> q(req.q.data.data(), req.q.data.size());
  pin::forwardKinematics(*model_, *data_, q);
  auto ee = data_->oMi.back();
  res.pose.position.x = ee.translation()(0);
  res.pose.position.y = ee.translation()(1);
  res.pose.position.z = ee.translation()(2);

  Eigen::Quaterniond quat(ee.rotation());
  res.pose.orientation.x = quat.x();
  res.pose.orientation.y = quat.y();
  res.pose.orientation.z = quat.z();
  res.pose.orientation.w = quat.w();
  return true;
}

}  // namespace gramps_kd
