#include "minc/limits/free_joint_velocity_limit.h"
#include "minc/exceptions.h"
#include <mujoco/mujoco.h>
#include <vector>

namespace minc {
namespace limits {

int FreeJointVelocityLimit::resolve_free_joint(const mjModel* model,
                                               const std::string& joint_name) {
  if (!joint_name.empty()) {
    const int jid = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    if (jid < 0) {
      throw LimitDefinitionError("Joint '" + joint_name + "' not found");
    }
    if (model->jnt_type[jid] != mjJNT_FREE) {
      throw LimitDefinitionError("Joint " + joint_name + " is not a free joint");
    }
    return jid;
  }

  std::vector<int> free_jids;
  for (int i = 0; i < model->njnt; ++i) {
    if (model->jnt_type[i] == mjJNT_FREE) {
      free_jids.push_back(i);
    }
  }
  if (free_jids.empty()) {
    throw LimitDefinitionError("Model has no free joint");
  }
  if (free_jids.size() > 1) {
    throw LimitDefinitionError(
        "Model has multiple free joints; pass joint_name");
  }
  return free_jids[0];
}

FreeJointVelocityLimit::FreeJointVelocityLimit(
    const mjModel* model, const std::optional<Eigen::Vector3d>& max_linear,
    const std::optional<Eigen::Vector3d>& max_angular,
    const std::string& joint_name)
    : linear_max_(max_linear), angular_max_(max_angular), nv_(model->nv) {
  if (!linear_max_.has_value() && !angular_max_.has_value()) {
    throw LimitDefinitionError(
        "At least one of max_linear_velocity or max_angular_velocity must be "
        "set");
  }

  const int jid = resolve_free_joint(model, joint_name);
  base_body_id_ = model->jnt_bodyid[jid];
  const int dofadr = model->jnt_dofadr[jid];
  lin_dofs_ = Eigen::Vector3i(dofadr, dofadr + 1, dofadr + 2);

  if (angular_max_.has_value()) {
    Eigen::MatrixXd G_ang = Eigen::MatrixXd::Zero(3, nv_);
    for (int i = 0; i < 3; ++i) {
      G_ang(i, dofadr + 3 + i) = 1.0;
    }
    G_ang_ = G_ang;
  }
}

Constraint FreeJointVelocityLimit::compute_qp_inequalities(
    const Configuration& configuration, double dt) const {
  std::vector<Eigen::MatrixXd> blocks;
  std::vector<Eigen::VectorXd> bounds;

  if (linear_max_.has_value()) {
    const mjData* data = configuration.data();
    Eigen::Matrix3d R;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R(i, j) = data->xmat[9 * base_body_id_ + 3 * i + j];
      }
    }
    Eigen::MatrixXd G_lin = Eigen::MatrixXd::Zero(3, nv_);
    G_lin.block<3, 3>(0, lin_dofs_[0]) = R.transpose();
    blocks.push_back(G_lin);
    bounds.push_back(linear_max_.value());
  }
  if (G_ang_.has_value()) {
    blocks.push_back(G_ang_.value());
    bounds.push_back(angular_max_.value());
  }

  int rows = 0;
  for (const auto& block : blocks) {
    rows += block.rows();
  }
  Eigen::MatrixXd G_active(rows, nv_);
  Eigen::VectorXd h_active(rows);
  int offset = 0;
  for (size_t i = 0; i < blocks.size(); ++i) {
    const int r = blocks[i].rows();
    G_active.block(offset, 0, r, nv_) = blocks[i];
    h_active.segment(offset, r) = dt * bounds[i];
    offset += r;
  }

  Eigen::MatrixXd G(2 * rows, nv_);
  Eigen::VectorXd h(2 * rows);
  G.topRows(rows) = G_active;
  G.bottomRows(rows) = -G_active;
  h.head(rows) = h_active;
  h.tail(rows) = h_active;
  return Constraint{G, h};
}

}  // namespace limits
}  // namespace minc
