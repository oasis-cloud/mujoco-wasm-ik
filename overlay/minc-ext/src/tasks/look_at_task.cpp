#include "minc/tasks/look_at_task.h"
#include "minc/exceptions.h"
#include "minc/lie/utils.h"
#include <algorithm>

namespace minc {
namespace tasks {

LookAtTask::LookAtTask(const std::string& frame_name,
                       const std::string& frame_type,
                       const Eigen::Vector3d& axis, double cost, double gain,
                       double lm_damping)
    : Task(Eigen::VectorXd::Constant(3, cost), gain, lm_damping),
      frame_name_(frame_name),
      frame_type_(frame_type) {
  if (cost < 0.0) {
    throw TaskDefinitionError("LookAtTask cost must be >= 0");
  }
  const double norm = axis.norm();
  if (norm < lie::get_epsilon<double>()) {
    throw TaskDefinitionError("LookAtTask axis must be non-zero");
  }
  axis_ = axis / norm;
}

void LookAtTask::set_target(const Eigen::Vector3d& target_pos) {
  target_pos_ = target_pos;
  target_set_ = true;
}

void LookAtTask::set_target_from_configuration(
    const Configuration& configuration) {
  const lie::SE3 transform =
      configuration.get_transform_frame_to_world(frame_name_, frame_type_);
  set_target(transform.rotation().as_matrix() * axis_ + transform.translation());
}

void LookAtTask::error_and_jacobian(const Configuration& configuration,
                                    Eigen::VectorXd& error,
                                    Eigen::MatrixXd& jacobian) const {
  if (!target_set_) {
    throw TargetNotSet("LookAtTask");
  }

  const lie::SE3 transform =
      configuration.get_transform_frame_to_world(frame_name_, frame_type_);
  const Eigen::Matrix3d rotation = transform.rotation().as_matrix();
  const Eigen::Vector3d position = transform.translation();
  const Eigen::MatrixXd jac =
      configuration.get_frame_jacobian(frame_name_, frame_type_);
  const Eigen::MatrixXd jac_v = jac.topRows(3);
  const Eigen::MatrixXd jac_w = jac.bottomRows(3);

  Eigen::Vector3d r = rotation.transpose() * (target_pos_ - position);
  double n = r.norm();
  n = std::max(n, lie::get_epsilon<double>());
  const Eigen::Vector3d d_hat = r / n;

  error = axis_ - d_hat;
  const Eigen::Matrix3d projector =
      Eigen::Matrix3d::Identity() - d_hat * d_hat.transpose();
  jacobian = (projector * (jac_v - lie::skew(r) * jac_w)) / n;
}

Eigen::VectorXd LookAtTask::compute_error(
    const Configuration& configuration) {
  Eigen::VectorXd error;
  Eigen::MatrixXd jacobian;
  error_and_jacobian(configuration, error, jacobian);
  return error;
}

Eigen::MatrixXd LookAtTask::compute_jacobian(
    const Configuration& configuration) {
  Eigen::VectorXd error;
  Eigen::MatrixXd jacobian;
  error_and_jacobian(configuration, error, jacobian);
  return jacobian;
}

}  // namespace tasks
}  // namespace minc
