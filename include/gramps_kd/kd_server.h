#include <memory>
#include <pinocchio/multibody/fwd.hpp>

#include "gramps_kd/ForwardDynamics.h"
#include "gramps_kd/ForwardKinematics.h"
#include "gramps_kd/InverseDynamics.h"
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

class KDServer {
public:
  explicit KDServer(std::shared_ptr<urdf::Model> urdf_model, const IKParams &ik_params);

  ~KDServer();

  bool forward_kinematics(gramps_kd::ForwardKinematics::Request &req, gramps_kd::ForwardKinematics::Response &res);

  bool inverse_kinematics(gramps_kd::InverseKinematics::Request &req, gramps_kd::InverseKinematics::Response &res);

  bool forward_dynamics(gramps_kd::ForwardDynamics::Request &req, gramps_kd::ForwardDynamics::Response &res);

  bool inverse_dynamics(gramps_kd::InverseDynamics::Request &req, gramps_kd::InverseDynamics::Response &res);

private:
  std::shared_ptr<urdf::Model> urdf_model_;
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
  const IKParams ik_params_;

  int gripper_to_fingers(std::vector<double> &arr) const;

  void fingers_to_gripper(std::vector<double> &arr, int n_fingers) const;
};

}  // namespace gramps_kd
