#include "minc/tasks/axis_align_task.h"
#include "minc/exceptions.h"
#include "minc/lie/utils.h"

namespace minc {
namespace tasks {

AxisAlignTask::AxisAlignTask(const std::string& frame_name,
                             const std::string& frame_type,
                             const Eigen::Vector3d& axis, double cost,
                             double gain, double lm_damping)
    : Task(Eigen::VectorXd::Constant(3, cost), gain, lm_damping),
      frame_name_(frame_name),
      frame_type_(frame_type) {
  if (cost < 0.0) {
    throw TaskDefinitionError("AxisAlignTask cost must be >= 0");
  }
  const double norm = axis.norm();
  if (norm < lie::get_epsilon<double>()) {
    throw TaskDefinitionError("AxisAlignTask axis must be non-zero");
  }
  axis_ = axis / norm;
}

void AxisAlignTask::set_target(const Eigen::Vector3d& target_dir) {
  const double norm = target_dir.norm();
  if (norm < lie::get_epsilon<double>()) {
    throw InvalidTarget("AxisAlignTask target direction must be non-zero");
  }
  target_dir_ = target_dir / norm;
  target_set_ = true;
}

void AxisAlignTask::set_target_from_configuration(
    const Configuration& configuration) {
  const lie::SE3 transform =
      configuration.get_transform_frame_to_world(frame_name_, frame_type_);
  set_target(transform.rotation().as_matrix() * axis_);
}

void AxisAlignTask::error_and_jacobian(const Configuration& configuration,
                                       Eigen::VectorXd& error,
                                       Eigen::MatrixXd& jacobian) const {
  if (!target_set_) {
    throw TargetNotSet("AxisAlignTask");
  }

  const lie::SE3 transform =
      configuration.get_transform_frame_to_world(frame_name_, frame_type_);
  const Eigen::Matrix3d rotation = transform.rotation().as_matrix();
  const Eigen::MatrixXd jac_w =
      configuration.get_frame_jacobian(frame_name_, frame_type_).bottomRows(3);

  const Eigen::Vector3d d = rotation.transpose() * target_dir_;
  error = axis_ - d;
  jacobian = -lie::skew(d) * jac_w;
}

Eigen::VectorXd AxisAlignTask::compute_error(
    const Configuration& configuration) {
  Eigen::VectorXd error;
  Eigen::MatrixXd jacobian;
  error_and_jacobian(configuration, error, jacobian);
  return error;
}

Eigen::MatrixXd AxisAlignTask::compute_jacobian(
    const Configuration& configuration) {
  Eigen::VectorXd error;
  Eigen::MatrixXd jacobian;
  error_and_jacobian(configuration, error, jacobian);
  return jacobian;
}

}  // namespace tasks
}  // namespace minc
