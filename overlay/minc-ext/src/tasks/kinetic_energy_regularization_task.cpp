#include "minc/tasks/kinetic_energy_regularization_task.h"
#include "minc/exceptions.h"

namespace minc {
namespace tasks {

KineticEnergyRegularizationTask::KineticEnergyRegularizationTask(double cost)
    : cost_(cost) {
  if (cost_ < 0.0) {
    throw TaskDefinitionError(
        "KineticEnergyRegularizationTask cost should be >= 0");
  }
}

void KineticEnergyRegularizationTask::set_dt(double dt) {
  inv_dt_sq_ = 1.0 / (dt * dt);
}

Objective KineticEnergyRegularizationTask::compute_qp_objective(
    const Configuration& configuration) {
  if (!inv_dt_sq_.has_value()) {
    throw IntegrationTimestepNotSet("KineticEnergyRegularizationTask");
  }
  Eigen::MatrixXd H =
      cost_ * inv_dt_sq_.value() * configuration.get_inertia_matrix();
  Eigen::VectorXd c = Eigen::VectorXd::Zero(configuration.nv());
  return Objective{H, c};
}

}  // namespace tasks
}  // namespace minc
