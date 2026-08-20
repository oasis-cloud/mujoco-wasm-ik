#pragma once

#include "minc/limits/limit.h"
#include <optional>
#include <string>

namespace minc {
namespace limits {

class FreeJointVelocityLimit : public Limit {
 public:
  FreeJointVelocityLimit(const mjModel* model,
                         const std::optional<Eigen::Vector3d>& max_linear,
                         const std::optional<Eigen::Vector3d>& max_angular,
                         const std::string& joint_name = "");

  Constraint compute_qp_inequalities(const Configuration& configuration,
                                     double dt) const override;

 private:
  static int resolve_free_joint(const mjModel* model,
                                const std::string& joint_name);

  int base_body_id_;
  int nv_;
  Eigen::Vector3i lin_dofs_;
  std::optional<Eigen::Vector3d> linear_max_;
  std::optional<Eigen::Vector3d> angular_max_;
  std::optional<Eigen::MatrixXd> G_ang_;
};

}  // namespace limits
}  // namespace minc
