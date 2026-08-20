#pragma once

#include <memory>
#include <string>
#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include "minc/lie/se3.h"

namespace minc {

/**
 * @brief Encapsulates a model and data for convenient access to kinematic quantities.
 *
 * Supports either owning a fresh mjData, or borrowing an existing mjData so WASM
 * bindings can share the simulation state with IkConfiguration.
 */
class Configuration {
 public:
  /**
   * @brief Own a new mjData, optionally initialized from q.
   */
  Configuration(const mjModel* model, const double* q = nullptr);

  /**
   * @brief Borrow an existing mjData (does not take ownership).
   */
  Configuration(const mjModel* model, mjData* data);

  ~Configuration();

  Configuration(const Configuration&) = delete;
  Configuration& operator=(const Configuration&) = delete;
  Configuration(Configuration&&) = default;
  Configuration& operator=(Configuration&&) = default;

  void update(const double* q = nullptr);
  void update_from_keyframe(const std::string& key_name);
  void check_limits(double tol = 1e-6, bool safety_break = true) const;

  Eigen::MatrixXd get_frame_jacobian(const std::string& frame_name,
                                     const std::string& frame_type) const;
  lie::SE3 get_transform_frame_to_world(const std::string& frame_name,
                                        const std::string& frame_type) const;
  lie::SE3 get_transform(const std::string& source_name,
                         const std::string& source_type,
                         const std::string& dest_name,
                         const std::string& dest_type) const;

  Eigen::VectorXd integrate(const Eigen::VectorXd& velocity, double dt) const;
  void integrate_inplace(const Eigen::VectorXd& velocity, double dt);
  Eigen::MatrixXd get_inertia_matrix() const;

  Eigen::VectorXd q() const;
  Eigen::VectorXd get_q() const { return q(); }

  void differentiate_pos(const Eigen::VectorXd& target_q,
                         const Eigen::VectorXd& current_q,
                         Eigen::VectorXd& result) const;

  int nv() const { return model_->nv; }
  int nq() const { return model_->nq; }
  const mjModel* model() const { return model_; }
  const mjModel* get_model() const { return model_; }
  int get_nq() const { return model_->nq; }
  int get_nv() const { return model_->nv; }
  mjData* data() const { return data_; }

 private:
  const mjModel* model_;
  mjData* data_;
  std::unique_ptr<mjData, void (*)(mjData*)> owned_data_;
};

}  // namespace minc
