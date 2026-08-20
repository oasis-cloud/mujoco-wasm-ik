#pragma once

#include "minc/tasks/task.h"
#include <string>

namespace minc {
namespace tasks {

class LookAtTask : public Task {
 public:
  LookAtTask(const std::string& frame_name, const std::string& frame_type,
             const Eigen::Vector3d& axis, double cost, double gain = 1.0,
             double lm_damping = 0.0);

  void set_target(const Eigen::Vector3d& target_pos);
  void set_target_from_configuration(const Configuration& configuration);

  Eigen::VectorXd compute_error(const Configuration& configuration) override;
  Eigen::MatrixXd compute_jacobian(const Configuration& configuration) override;
  int get_dimension() const override { return 3; }

 private:
  void error_and_jacobian(const Configuration& configuration,
                          Eigen::VectorXd& error,
                          Eigen::MatrixXd& jacobian) const;

  std::string frame_name_;
  std::string frame_type_;
  Eigen::Vector3d axis_;
  Eigen::Vector3d target_pos_;
  bool target_set_ = false;
};

}  // namespace tasks
}  // namespace minc
