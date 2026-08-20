#pragma once

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "minc/configuration.h"
#include "minc/limits/limit.h"
#include "minc/tasks/task.h"

namespace minc {

Eigen::VectorXd solve_ik(
    Configuration& configuration,
    const std::vector<std::shared_ptr<tasks::BaseTask>>& tasks, double dt,
    const std::string& solver = "daqp", double damping = 1e-12,
    bool safety_break = false,
    const std::vector<std::shared_ptr<limits::Limit>>& limits = {},
    const std::vector<std::shared_ptr<tasks::Task>>& constraints = {});

}  // namespace minc
