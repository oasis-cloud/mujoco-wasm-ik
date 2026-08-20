#pragma once

#include "minc/tasks/task.h"
#include <optional>

namespace minc {
namespace tasks {

class KineticEnergyRegularizationTask : public BaseTask {
 public:
  explicit KineticEnergyRegularizationTask(double cost);

  void set_dt(double dt);

  Objective compute_qp_objective(const Configuration& configuration) override;

 private:
  double cost_;
  std::optional<double> inv_dt_sq_;
};

}  // namespace tasks
}  // namespace minc
