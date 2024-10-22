#include <memory>
#include <pinocchio/multibody/fwd.hpp>

#include "gramps_kd/ForwardKinematics.h"

namespace urdf {
class Model;
}

namespace gramps_kd {

class KinematicsServer {
public:
  explicit KinematicsServer(std::shared_ptr<urdf::Model> urdf_model);
  ~KinematicsServer();

  bool forward_kinematics(gramps_kd::ForwardKinematics::Request &req, gramps_kd::ForwardKinematics::Response &res);

private:
  std::shared_ptr<urdf::Model> urdf_model_;
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
};

}  // namespace gramps_kd
