#include "minc/solve_ik.h"
#include "minc/exceptions.h"
#include "minc/limits/configuration_limit.h"
#include "minc/tasks/task.h"
#include <Eigen/Dense>
#include <daqp.h>
#include <api.h>
#include <types.h>
#include <limits>
#include <vector>

using namespace std;

namespace {

void eigen_to_row_major(const Eigen::MatrixXd& matrix, vector<c_float>& data) {
  const int rows = matrix.rows();
  const int cols = matrix.cols();
  data.resize(rows * cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      data[i * cols + j] = matrix(i, j);
    }
  }
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> compute_qp_objective(
    const minc::Configuration& configuration,
    const std::vector<std::shared_ptr<minc::tasks::BaseTask>>& tasks,
    double damping) {
  const int nv = configuration.nv();
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(nv, nv) * damping;
  Eigen::VectorXd c = Eigen::VectorXd::Zero(nv);
  for (const auto& task : tasks) {
    const auto objective = task->compute_qp_objective(configuration);
    H += objective.H;
    c += objective.c;
  }
  return {H, c};
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> compute_qp_inequalities(
    const minc::Configuration& configuration,
    const std::vector<std::shared_ptr<minc::limits::Limit>>& limits,
    double dt) {
  std::vector<Eigen::MatrixXd> G_list;
  std::vector<Eigen::VectorXd> h_list;
  for (const auto& limit : limits) {
    const auto constraint = limit->compute_qp_inequalities(configuration, dt);
    if (!constraint.inactive()) {
      G_list.push_back(constraint.G.value());
      h_list.push_back(constraint.h.value());
    }
  }
  if (G_list.empty()) {
    return {Eigen::MatrixXd::Zero(0, configuration.nv()),
            Eigen::VectorXd::Zero(0)};
  }

  int total_rows = 0;
  for (const auto& G : G_list) {
    total_rows += G.rows();
  }
  Eigen::MatrixXd G_combined(total_rows, configuration.nv());
  Eigen::VectorXd h_combined(total_rows);
  int current_row = 0;
  for (size_t i = 0; i < G_list.size(); ++i) {
    const int rows = G_list[i].rows();
    G_combined.block(current_row, 0, rows, configuration.nv()) = G_list[i];
    h_combined.segment(current_row, rows) = h_list[i];
    current_row += rows;
  }
  return {G_combined, h_combined};
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> compute_qp_equalities(
    const minc::Configuration& configuration,
    const std::vector<std::shared_ptr<minc::tasks::Task>>& constraints) {
  if (constraints.empty()) {
    return {Eigen::MatrixXd::Zero(0, configuration.nv()),
            Eigen::VectorXd::Zero(0)};
  }
  std::vector<Eigen::MatrixXd> A_list;
  std::vector<Eigen::VectorXd> b_list;
  for (const auto& task : constraints) {
    A_list.push_back(task->compute_jacobian(configuration));
    b_list.push_back(-task->get_gain() * task->compute_error(configuration));
  }
  int total_rows = 0;
  for (const auto& A : A_list) {
    total_rows += A.rows();
  }
  Eigen::MatrixXd A_combined(total_rows, configuration.nv());
  Eigen::VectorXd b_combined(total_rows);
  int current_row = 0;
  for (size_t i = 0; i < A_list.size(); ++i) {
    const int rows = A_list[i].rows();
    A_combined.block(current_row, 0, rows, configuration.nv()) = A_list[i];
    b_combined.segment(current_row, rows) = b_list[i];
    current_row += rows;
  }
  return {A_combined, b_combined};
}

}  // namespace

namespace minc {

Eigen::VectorXd solve_ik(
    Configuration& configuration,
    const std::vector<std::shared_ptr<tasks::BaseTask>>& tasks, double dt,
    const std::string& solver, double damping, bool safety_break,
    const std::vector<std::shared_ptr<limits::Limit>>& limits,
    const std::vector<std::shared_ptr<tasks::Task>>& constraints) {
  (void)solver;

  if (safety_break) {
    configuration.check_limits(1e-6, true);
  }

  // Mink semantics: empty limits disables inequalities; no default fill-in.
  const std::vector<std::shared_ptr<limits::Limit>>& active_limits = limits;

  const int nv = configuration.nv();
  if (nv == 0) {
    return Eigen::VectorXd::Zero(0);
  }

  auto [H, c] = compute_qp_objective(configuration, tasks, damping);
  auto [G, h] = compute_qp_inequalities(configuration, active_limits, dt);
  auto [Aeq, beq] = compute_qp_equalities(configuration, constraints);

  const int m_ineq = G.rows();
  const int m_eq = Aeq.rows();
  const int m = m_ineq + m_eq;

  vector<c_float> H_data, f_data, A_data, lbA_data, ubA_data;
  eigen_to_row_major(H, H_data);
  f_data.resize(nv);
  for (int i = 0; i < nv; ++i) {
    f_data[i] = c[i];
  }

  if (m > 0) {
    Eigen::MatrixXd A_all(m, nv);
    if (m_ineq > 0) {
      A_all.topRows(m_ineq) = G;
    }
    if (m_eq > 0) {
      A_all.bottomRows(m_eq) = Aeq;
    }
    eigen_to_row_major(A_all, A_data);
    lbA_data.resize(m);
    ubA_data.resize(m);
    for (int i = 0; i < m_ineq; ++i) {
      lbA_data[i] = -std::numeric_limits<c_float>::infinity();
      ubA_data[i] = h[i];
    }
    for (int i = 0; i < m_eq; ++i) {
      lbA_data[m_ineq + i] = beq[i];
      ubA_data[m_ineq + i] = beq[i];
    }
  }

  DAQPProblem qp;
  qp.n = nv;
  qp.m = m;
  qp.ms = 0;
  qp.H = H_data.data();
  qp.f = f_data.data();
  qp.A = m > 0 ? A_data.data() : nullptr;
  qp.bupper = m > 0 ? ubA_data.data() : nullptr;
  qp.blower = m > 0 ? lbA_data.data() : nullptr;
  qp.sense = nullptr;
  qp.nh = 1;
  qp.break_points = nullptr;

  DAQPSettings settings;
  daqp_default_settings(&settings);
  settings.primal_tol = 1e-4;
  settings.dual_tol = 1e-4;
  settings.iter_limit = 1000;

  vector<c_float> x_solution(nv);
  vector<c_float> lam_solution(m);
  DAQPResult result_daqp;
  result_daqp.x = x_solution.data();
  result_daqp.lam = m > 0 ? lam_solution.data() : nullptr;
  result_daqp.fval = 0.0;
  result_daqp.soft_slack = 0.0;
  result_daqp.exitflag = 0;
  result_daqp.iter = 0;
  result_daqp.nodes = 0;
  result_daqp.solve_time = 0.0;
  result_daqp.setup_time = 0.0;

  daqp_quadprog(&result_daqp, &qp, &settings);

  Eigen::VectorXd result(nv);
  if (result_daqp.exitflag == 1) {
    result = Eigen::Map<Eigen::VectorXd>(result_daqp.x, nv);
    if (dt > 1e-9) {
      result /= dt;
    }
  } else {
    throw NoSolutionFound("DAQP solver failed to find a solution. Exit flag: " +
                          std::to_string(result_daqp.exitflag));
  }
  return result;
}

}  // namespace minc
