#include "gramps_kd/kinematics_server.h"

// clang-format off
// pinocchio includes have to go before urdf for some reason
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <urdf/model.h>
// clang-format on

namespace pin = pinocchio;

namespace gramps_kd {

KinematicsServer::KinematicsServer(std::shared_ptr<urdf::Model> urdf_model, const IKParams &ik_params)
    : urdf_model_(urdf_model), model_(std::make_unique<pin::Model>()), ik_params_(ik_params) {
  pin::urdf::buildModel(urdf_model_, *model_);
  data_ = std::make_unique<pin::Data>(*model_);
}

KinematicsServer::~KinematicsServer() = default;

bool KinematicsServer::forward_kinematics(gramps_kd::ForwardKinematics::Request &req,
                                          gramps_kd::ForwardKinematics::Response &res) {
  if (req.q.data.size() != model_->nq) {
    ROS_ERROR("Incorrect number of joint angles! Should be %d, got %ld.", model_->nq, req.q.data.size());
    return false;
  }
  if (!model_->existFrame(req.link_name)) {
    ROS_ERROR("No frame found with name: %s", req.link_name.c_str());
    return false;
  }

  Eigen::Map<Eigen::VectorXd> q(req.q.data.data(), req.q.data.size());
  pin::forwardKinematics(*model_, *data_, q);

  auto frame_id = model_->getFrameId(req.link_name);
  pin::updateFramePlacement(*model_, *data_, frame_id);

  res.frame_id = urdf_model_->getRoot()->name;

  auto link = data_->oMf[frame_id];
  res.pose.position.x = link.translation()(0);
  res.pose.position.y = link.translation()(1);
  res.pose.position.z = link.translation()(2);

  Eigen::Quaterniond quat(link.rotation());
  res.pose.orientation.x = quat.x();
  res.pose.orientation.y = quat.y();
  res.pose.orientation.z = quat.z();
  res.pose.orientation.w = quat.w();

  return true;
}

bool KinematicsServer::inverse_kinematics(gramps_kd::InverseKinematics::Request &req,
                                          gramps_kd::InverseKinematics::Response &res) {
  if (req.q.data.size() != model_->nq) {
    ROS_ERROR("Incorrect number of joint angles! Should be %d, got %ld.", model_->nq, req.q.data.size());
    return false;
  }
  if (!model_->existFrame(req.link_name)) {
    ROS_ERROR("No frame found with name: %s", req.link_name.c_str());
    return false;
  }

  auto frame_id = model_->getFrameId(req.link_name);
  pin::SE3 pose(Eigen::Quaterniond(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y,
                                   req.pose.orientation.z),
                Eigen::Vector3d(req.pose.position.x, req.pose.position.y, req.pose.position.z));
  // note that the result fields are modified in-place
  res.q = req.q;
  Eigen::Map<Eigen::VectorXd> q(res.q.data.data(), res.q.data.size());
  res.err.data.resize(6);

  // CLIK algorithm

  pin::Data::Matrix6x J(6, model_->nv);
  J.setZero();

  Eigen::Map<Eigen::VectorXd> err(res.err.data.data(), res.err.data.size());
  Eigen::VectorXd v(model_->nv);
  for (int i = 0;; i++) {
    pin::forwardKinematics(*model_, *data_, q);
    pin::updateFramePlacements(*model_, *data_);

    const pinocchio::SE3 iMd = data_->oMf[frame_id].actInv(pose);
    err = pinocchio::log6(iMd).toVector();  // in joint frame
    if (err.norm() < ik_params_.eps) {
      return true;
    }
    if (i >= ik_params_.it_max) {
      return false;
    }
    pin::computeFrameJacobian(*model_, *data_, q, frame_id, J);
    pin::Data::Matrix6 Jlog;
    pin::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;
    pin::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += ik_params_.damping;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pin::integrate(*model_, q, v * ik_params_.dt);
  }
}

}  // namespace gramps_kd
