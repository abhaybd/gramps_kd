#include "gramps_kd/kd_server.h"

// clang-format off
// pinocchio includes have to go before urdf for some reason
#include "pinocchio/algorithm/aba.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/parsers/urdf.hpp>

#include <urdf/model.h>
// clang-format on

namespace pin = pinocchio;

namespace gramps_kd {

KDServer::KDServer(std::shared_ptr<urdf::Model> urdf_model, const IKParams &ik_params)
    : urdf_model_(urdf_model), model_(std::make_unique<pin::Model>()), ik_params_(ik_params) {
  pin::urdf::buildModel(urdf_model_, *model_);
  data_ = std::make_unique<pin::Data>(*model_);
}

KDServer::~KDServer() = default;

int KDServer::gripper_to_fingers(std::vector<double> &arr) const {
  if (arr.size() == model_->nq) {
    return 1;
  }

  // We assume that multi-fingered grippers are controlled by a single joint position.
  // We also assume that the gripper joint is the last joint in the model, so we can
  // distribute the gripper joint position evenly among the fingers, if there are multiple.
  int n_fingers = model_->nq - arr.size() + 1;
  double gripper_total = arr.back();
  double finger_pos = gripper_total / n_fingers;

  arr.reserve(model_->nq);
  arr.back() = finger_pos;
  while (arr.size() < model_->nq) {
    arr.push_back(finger_pos);
  }

  return n_fingers;
}

void KDServer::fingers_to_gripper(std::vector<double> &arr, int n_fingers) const {
  if (n_fingers == 1) {
    return;
  }

  // If we previously split the gripper joint position among multiple fingers,
  // we need to sum the finger positions to get the total gripper joint position.
  double gripper_total = 0.0;
  for (int i = 0; i < n_fingers; i++) {
    gripper_total += arr.back();
    arr.pop_back();
  }
  arr.push_back(gripper_total);
}

bool KDServer::forward_kinematics(gramps_kd::ForwardKinematics::Request &req,
                                  gramps_kd::ForwardKinematics::Response &res) {
  if (req.q.data.size() > model_->nq) {
    ROS_ERROR("Too many joint angles! Got %ld, should be at most %d", req.q.data.size(), model_->nq);
    return false;
  }
  if (!model_->existFrame(req.link_name)) {
    ROS_ERROR("No frame found with name: %s", req.link_name.c_str());
    return false;
  }

  gripper_to_fingers(req.q.data);

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

bool KDServer::inverse_kinematics(gramps_kd::InverseKinematics::Request &req,
                                  gramps_kd::InverseKinematics::Response &res) {
  if (req.q.data.size() > model_->nq) {
    ROS_ERROR("Too many joint angles! Got %ld, should be at most %d", req.q.data.size(), model_->nq);
    return false;
  }
  if (!model_->existFrame(req.link_name)) {
    ROS_ERROR("No frame found with name: %s", req.link_name.c_str());
    return false;
  }

  int n_fingers = gripper_to_fingers(req.q.data);

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
  bool success = false;
  for (int i = 0;; i++) {
    pin::forwardKinematics(*model_, *data_, q);
    pin::updateFramePlacements(*model_, *data_);

    const pinocchio::SE3 iMd = data_->oMf[frame_id].actInv(pose);
    err = pinocchio::log6(iMd).toVector();  // in joint frame
    if (err.norm() < ik_params_.eps) {
      success = true;
      break;
    }
    if (i >= ik_params_.it_max) {
      success = false;
      break;
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

  fingers_to_gripper(res.q.data, n_fingers);
  return success;
}

bool KDServer::forward_dynamics(gramps_kd::ForwardDynamics::Request &req, gramps_kd::ForwardDynamics::Response &res) {
  if (req.q.data.size() > model_->nq) {
    ROS_ERROR("Too many joint angles! Got %ld, should be at most %d", req.q.data.size(), model_->nq);
    return false;
  }
  if (req.v.data.size() > model_->nv) {
    ROS_ERROR("Too many joint velocities! Got %ld, should be at most %d", req.v.data.size(), model_->nv);
    return false;
  }
  if (req.tau.data.size() > model_->nv) {
    ROS_ERROR("Too many joint torques! Got %ld, should be at most %d", req.tau.data.size(), model_->nv);
    return false;
  }

  int n_fingers = gripper_to_fingers(req.q.data);
  gripper_to_fingers(req.v.data);
  gripper_to_fingers(req.tau.data);

  Eigen::Map<Eigen::VectorXd> q(req.q.data.data(), req.q.data.size());
  Eigen::Map<Eigen::VectorXd> qd(req.v.data.data(), req.v.data.size());
  Eigen::Map<Eigen::VectorXd> tau(req.tau.data.data(), req.tau.data.size());

  pin::aba(*model_, *data_, q, qd, tau);

  res.a.data.resize(model_->nv);
  Eigen::Map<Eigen::VectorXd> qdd(res.a.data.data(), res.a.data.size());
  qdd = data_->ddq;
  fingers_to_gripper(res.a.data, n_fingers);

  return true;
}

bool KDServer::inverse_dynamics(gramps_kd::InverseDynamics::Request &req, gramps_kd::InverseDynamics::Response &res) {
  if (req.q.data.size() > model_->nq) {
    ROS_ERROR("Too many joint angles! Got %ld, should be at most %d", req.q.data.size(), model_->nq);
    return false;
  }
  if (req.v.data.size() > model_->nv) {
    ROS_ERROR("Too many joint velocities! Got %ld, should be at most %d", req.v.data.size(), model_->nv);
    return false;
  }
  if (req.a.data.size() > model_->nv) {
    ROS_ERROR("Too many joint accelerations! Got %ld, should be at most %d", req.a.data.size(), model_->nv);
    return false;
  }

  int n_fingers = gripper_to_fingers(req.q.data);
  gripper_to_fingers(req.v.data);
  gripper_to_fingers(req.a.data);

  Eigen::Map<Eigen::VectorXd> q(req.q.data.data(), req.q.data.size());
  Eigen::Map<Eigen::VectorXd> qd(req.v.data.data(), req.v.data.size());
  Eigen::Map<Eigen::VectorXd> qdd(req.a.data.data(), req.a.data.size());

  pin::rnea(*model_, *data_, q, qd, qdd);

  res.tau.data.resize(model_->nv);
  Eigen::Map<Eigen::VectorXd> tau(res.tau.data.data(), res.tau.data.size());
  tau = data_->tau;
  fingers_to_gripper(res.tau.data, n_fingers);

  return true;
}

}  // namespace gramps_kd
