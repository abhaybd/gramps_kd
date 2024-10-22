#include <memory>
#include <pinocchio/multibody/fwd.hpp>

#include "gramps_kd/ForwardKinematics.h"
#include "gramps_kd/InverseKinematics.h"

namespace urdf {
class Model;
}

namespace gramps_kd {

struct IKParams {
  double eps;
  int it_max;
  double dt;
  double damping;
};

class KinematicsServer {
public:
  explicit KinematicsServer(std::shared_ptr<urdf::Model> urdf_model, const IKParams &ik_params);

  ~KinematicsServer();

  bool forward_kinematics(gramps_kd::ForwardKinematics::Request &req, gramps_kd::ForwardKinematics::Response &res);

  bool inverse_kinematics(gramps_kd::InverseKinematics::Request &req, gramps_kd::InverseKinematics::Response &res);

private:
  std::shared_ptr<urdf::Model> urdf_model_;
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
  const IKParams ik_params_;
};

}  // namespace gramps_kd
