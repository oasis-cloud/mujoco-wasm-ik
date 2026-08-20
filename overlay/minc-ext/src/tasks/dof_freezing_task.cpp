#include "minc/tasks/dof_freezing_task.h"
#include "minc/exceptions.h"
#include <algorithm>
#include <set>

namespace minc {
namespace tasks {

DofFreezingTask::DofFreezingTask(mjModel* model,
                                 const std::vector<int>& dof_indices,
                                 double gain)
    : Task(Eigen::VectorXd::Ones(static_cast<int>(dof_indices.size())), gain,
           0.0),
      nv_(model->nv) {
  if (dof_indices.empty()) {
    throw TaskDefinitionError("DofFreezingTask requires at least one DOF index.");
  }
  std::set<int> seen;
  for (int dof_idx : dof_indices) {
    if (dof_idx < 0 || dof_idx >= model->nv) {
      throw TaskDefinitionError("DOF index " + std::to_string(dof_idx) +
                                " is out of range [0, " +
                                std::to_string(model->nv) + ").");
    }
    if (!seen.insert(dof_idx).second) {
      throw TaskDefinitionError("Duplicate DOF indices found.");
    }
  }
  dof_indices_ = dof_indices;
  std::sort(dof_indices_.begin(), dof_indices_.end());

  const int k = static_cast<int>(dof_indices_.size());
  error_ = Eigen::VectorXd::Zero(k);
  jacobian_ = Eigen::MatrixXd::Zero(k, nv_);
  for (int i = 0; i < k; ++i) {
    jacobian_(i, dof_indices_[i]) = 1.0;
  }
}

Eigen::VectorXd DofFreezingTask::compute_error(
    const Configuration& /*configuration*/) {
  return error_;
}

Eigen::MatrixXd DofFreezingTask::compute_jacobian(
    const Configuration& /*configuration*/) {
  return jacobian_;
}

int DofFreezingTask::get_dimension() const {
  return static_cast<int>(dof_indices_.size());
}

}  // namespace tasks
}  // namespace minc
