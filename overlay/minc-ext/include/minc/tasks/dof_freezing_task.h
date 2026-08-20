#pragma once

#include "minc/tasks/task.h"
#include <vector>

namespace minc {
namespace tasks {

class DofFreezingTask : public Task {
 public:
  DofFreezingTask(mjModel* model, const std::vector<int>& dof_indices,
                  double gain = 1.0);

  Eigen::VectorXd compute_error(const Configuration& configuration) override;
  Eigen::MatrixXd compute_jacobian(const Configuration& configuration) override;
  int get_dimension() const override;

 private:
  std::vector<int> dof_indices_;
  int nv_;
  Eigen::VectorXd error_;
  Eigen::MatrixXd jacobian_;
};

}  // namespace tasks
}  // namespace minc
