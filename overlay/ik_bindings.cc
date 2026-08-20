// P3 IK bindings: mink 1.3 tasks/limits + solveIK constraints.
// Shares MjModel / MjData with the official MuJoCo WASM module.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <array>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "minc/configuration.h"
#include "minc/exceptions.h"
#include "minc/lie/se3.h"
#include "minc/lie/so3.h"
#include "minc/limits/collision_avoidance_limit.h"
#include "minc/limits/configuration_limit.h"
#include "minc/limits/free_joint_velocity_limit.h"
#include "minc/limits/velocity_limit.h"
#include "minc/solve_ik.h"
#include "minc/tasks/axis_align_task.h"
#include "minc/tasks/com_task.h"
#include "minc/tasks/damping_task.h"
#include "minc/tasks/dof_freezing_task.h"
#include "minc/tasks/equality_constraint_task.h"
#include "minc/tasks/frame_task.h"
#include "minc/tasks/kinetic_energy_regularization_task.h"
#include "minc/tasks/look_at_task.h"
#include "minc/tasks/posture_task.h"
#include "minc/tasks/relative_frame_task.h"
#include "wasm/codegen/generated/bindings.h"

namespace mujoco::wasm {
namespace {

using emscripten::val;

void ThrowJs(const std::string& message) {
  val(val::global("Error").new_(val(message))).throw_();
}

Eigen::VectorXd ValToVector(const val& v, int expected = -1) {
  if (v.isNull() || v.isUndefined()) {
    throw minc::MinkError("Expected a numeric array");
  }
  const int n = v["length"].as<int>();
  if (expected >= 0 && n != expected) {
    throw minc::MinkError("Expected array of length " + std::to_string(expected) +
                          " but got " + std::to_string(n));
  }
  Eigen::VectorXd out(n);
  for (int i = 0; i < n; ++i) {
    out[i] = v[i].as<double>();
  }
  return out;
}

val VectorToVal(const Eigen::VectorXd& x) {
  val arr = val::array();
  for (int i = 0; i < x.size(); ++i) {
    arr.call<void>("push", x[i]);
  }
  return arr;
}

minc::lie::SE3 Se3FromPosQuat(const val& pos, const val& quat) {
  const Eigen::VectorXd p = ValToVector(pos, 3);
  const Eigen::VectorXd q = ValToVector(quat, 4);
  const std::array<double, 4> wxyz = {q[0], q[1], q[2], q[3]};
  return minc::lie::SE3(minc::lie::SO3(wxyz), Eigen::Vector3d(p[0], p[1], p[2]));
}

std::vector<int> ValToIntVector(const val& v) {
  std::vector<int> out;
  if (v.isNull() || v.isUndefined()) {
    return out;
  }
  const int n = v["length"].as<int>();
  out.reserve(n);
  for (int i = 0; i < n; ++i) {
    out.push_back(v[i].as<int>());
  }
  return out;
}

std::unordered_map<std::string, double> ValToStringDoubleMap(const val& obj) {
  std::unordered_map<std::string, double> out;
  if (obj.isNull() || obj.isUndefined()) {
    return out;
  }
  const val keys = val::global("Object").call<val>("keys", obj);
  const int n = keys["length"].as<int>();
  for (int i = 0; i < n; ++i) {
    const std::string key = keys[i].as<std::string>();
    out[key] = obj[key].as<double>();
  }
  return out;
}

minc::limits::GeomSequence ParseGeomRef(const val& v) {
  minc::limits::GeomSequence seq;
  const std::string ty = v.typeOf().as<std::string>();
  if (ty == "string") {
    seq.emplace_back(v.as<std::string>());
    return seq;
  }
  if (ty == "number") {
    seq.emplace_back(v.as<int>());
    return seq;
  }
  const int n = v["length"].as<int>();
  seq.reserve(n);
  for (int i = 0; i < n; ++i) {
    const val item = v[i];
    if (item.typeOf().as<std::string>() == "number") {
      seq.emplace_back(item.as<int>());
    } else {
      seq.emplace_back(item.as<std::string>());
    }
  }
  return seq;
}

minc::limits::CollisionPairs ParseCollisionPairs(const val& pairs) {
  minc::limits::CollisionPairs out;
  if (pairs.isNull() || pairs.isUndefined()) {
    return out;
  }
  const int n = pairs["length"].as<int>();
  out.reserve(n);
  for (int i = 0; i < n; ++i) {
    const val pair = pairs[i];
    if (pair["length"].as<int>() != 2) {
      throw minc::MinkError(
          "Each collision pair must be [geomA, geomB] or [groupA, groupB]");
    }
    out.emplace_back(ParseGeomRef(pair[0]), ParseGeomRef(pair[1]));
  }
  return out;
}

Eigen::Vector3d ValToVec3(const val& v) {
  const Eigen::VectorXd p = ValToVector(v, 3);
  return Eigen::Vector3d(p[0], p[1], p[2]);
}

std::optional<Eigen::Vector3d> ParseOptionalVec3(const val& v) {
  if (v.isNull() || v.isUndefined()) {
    return std::nullopt;
  }
  if (v.typeOf().as<std::string>() == "number") {
    return Eigen::Vector3d::Constant(v.as<double>());
  }
  return ValToVec3(v);
}

}  // namespace

class IkConfiguration {
 public:
  IkConfiguration(MjModel& model, MjData& data)
      : config_(model.get(), data.get()) {}

  minc::Configuration& config() { return config_; }
  const minc::Configuration& config() const { return config_; }

  void update() { config_.update(); }

  void updateFromKeyframe(const std::string& key_name) {
    config_.update_from_keyframe(key_name);
  }

  int nq() const { return config_.nq(); }
  int nv() const { return config_.nv(); }

  val q() const { return VectorToVal(config_.q()); }

  void integrateInplace(const val& velocity, double dt) {
    const Eigen::VectorXd vel = ValToVector(velocity, config_.nv());
    config_.integrate_inplace(vel, dt);
  }

  val getFramePose(const std::string& frame_name, const std::string& frame_type) const {
    const minc::lie::SE3 pose =
        config_.get_transform_frame_to_world(frame_name, frame_type);
    const Eigen::Vector3d t = pose.translation();
    const auto& q = pose.rotation().wxyz();
    val pos = val::array();
    pos.call<void>("push", t[0]);
    pos.call<void>("push", t[1]);
    pos.call<void>("push", t[2]);
    val quat = val::array();
    quat.call<void>("push", q[0]);
    quat.call<void>("push", q[1]);
    quat.call<void>("push", q[2]);
    quat.call<void>("push", q[3]);
    val out = val::object();
    out.set("pos", pos);
    out.set("quat", quat);
    return out;
  }

  // mink ComTask uses data.subtree_com[1] (first non-world body subtree).
  val getCom() const {
    const mjData* data = config_.data();
    val arr = val::array();
    arr.call<void>("push", data->subtree_com[3]);
    arr.call<void>("push", data->subtree_com[4]);
    arr.call<void>("push", data->subtree_com[5]);
    return arr;
  }

 private:
  minc::Configuration config_;
};

class IkTask {
 public:
  virtual ~IkTask() = default;
  std::shared_ptr<minc::tasks::BaseTask> impl;
};

class FrameTask : public IkTask {
 public:
  FrameTask(const std::string& frame_name, const std::string& frame_type,
            double position_cost, double orientation_cost, double gain,
            double lm_damping) {
    impl = std::make_shared<minc::tasks::FrameTask>(
        frame_name, frame_type, Eigen::Vector3d::Constant(position_cost),
        Eigen::Vector3d::Constant(orientation_cost), gain, lm_damping);
  }

  void setTargetPosQuat(const val& pos, const val& quat) {
    auto* task = static_cast<minc::tasks::FrameTask*>(impl.get());
    task->set_target(Se3FromPosQuat(pos, quat));
  }

  void setTargetFromConfiguration(IkConfiguration& configuration) {
    auto* task = static_cast<minc::tasks::FrameTask*>(impl.get());
    task->set_target_from_configuration(configuration.config());
  }
};

class PostureTask : public IkTask {
 public:
  PostureTask(MjModel& model, double cost, double gain, double lm_damping) {
    impl = std::make_shared<minc::tasks::PostureTask>(
        model.get(), Eigen::VectorXd::Constant(1, cost), gain, lm_damping);
  }

  void setTargetFromConfiguration(IkConfiguration& configuration) {
    auto* task = static_cast<minc::tasks::PostureTask*>(impl.get());
    task->set_target_from_configuration(configuration.config());
  }
};

class RelativeFrameTask : public IkTask {
 public:
  RelativeFrameTask(const std::string& frame_name, const std::string& frame_type,
                    const std::string& root_name, const std::string& root_type,
                    double position_cost, double orientation_cost, double gain,
                    double lm_damping) {
    impl = std::make_shared<minc::tasks::RelativeFrameTask>(
        frame_name, frame_type, root_name, root_type,
        Eigen::Vector3d::Constant(position_cost),
        Eigen::Vector3d::Constant(orientation_cost), gain, lm_damping);
  }

  void setTargetPosQuat(const val& pos, const val& quat) {
    auto* task = static_cast<minc::tasks::RelativeFrameTask*>(impl.get());
    task->set_target(Se3FromPosQuat(pos, quat));
  }

  void setTargetFromConfiguration(IkConfiguration& configuration) {
    auto* task = static_cast<minc::tasks::RelativeFrameTask*>(impl.get());
    task->set_target_from_configuration(configuration.config());
  }
};

class ComTask : public IkTask {
 public:
  ComTask(double cost, double gain, double lm_damping) {
    impl = std::make_shared<minc::tasks::ComTask>(Eigen::Vector3d::Constant(cost),
                                                  gain, lm_damping);
  }

  void setTarget(const val& pos) {
    const Eigen::VectorXd p = ValToVector(pos, 3);
    auto* task = static_cast<minc::tasks::ComTask*>(impl.get());
    task->set_target(Eigen::Vector3d(p[0], p[1], p[2]));
  }

  void setTargetFromConfiguration(IkConfiguration& configuration) {
    setTarget(configuration.getCom());
  }
};

class DampingTask : public IkTask {
 public:
  explicit DampingTask(MjModel& model, double cost) {
    impl = std::make_shared<minc::tasks::DampingTask>(
        model.get(), Eigen::VectorXd::Constant(1, cost));
  }
};

class EqualityConstraintTask : public IkTask {
 public:
  EqualityConstraintTask(MjModel& model, double cost)
      : EqualityConstraintTask(model, cost, val::array(), 1.0, 0.0) {}

  EqualityConstraintTask(MjModel& model, double cost, val equality_ids, double gain,
                         double lm_damping) {
    try {
      impl = std::make_shared<minc::tasks::EqualityConstraintTask>(
          model.get(), Eigen::VectorXd::Constant(1, cost),
          ValToIntVector(equality_ids), gain, lm_damping);
    } catch (const std::exception& e) {
      ThrowJs(std::string("IK Error: ") + e.what());
    }
  }
};

class LookAtTask : public IkTask {
 public:
  LookAtTask(const std::string& frame_name, const std::string& frame_type, val axis,
             double cost, double gain, double lm_damping) {
    try {
      impl = std::make_shared<minc::tasks::LookAtTask>(
          frame_name, frame_type, ValToVec3(axis), cost, gain, lm_damping);
    } catch (const std::exception& e) {
      ThrowJs(std::string("IK Error: ") + e.what());
    }
  }

  void setTarget(const val& pos) {
    static_cast<minc::tasks::LookAtTask*>(impl.get())->set_target(ValToVec3(pos));
  }

  void setTargetFromConfiguration(IkConfiguration& configuration) {
    static_cast<minc::tasks::LookAtTask*>(impl.get())
        ->set_target_from_configuration(configuration.config());
  }
};

class AxisAlignTask : public IkTask {
 public:
  AxisAlignTask(const std::string& frame_name, const std::string& frame_type, val axis,
                double cost, double gain, double lm_damping) {
    try {
      impl = std::make_shared<minc::tasks::AxisAlignTask>(
          frame_name, frame_type, ValToVec3(axis), cost, gain, lm_damping);
    } catch (const std::exception& e) {
      ThrowJs(std::string("IK Error: ") + e.what());
    }
  }

  void setTarget(const val& dir) {
    static_cast<minc::tasks::AxisAlignTask*>(impl.get())->set_target(ValToVec3(dir));
  }

  void setTargetFromConfiguration(IkConfiguration& configuration) {
    static_cast<minc::tasks::AxisAlignTask*>(impl.get())
        ->set_target_from_configuration(configuration.config());
  }
};

class DofFreezingTask : public IkTask {
 public:
  DofFreezingTask(MjModel& model, val dof_indices)
      : DofFreezingTask(model, dof_indices, 1.0) {}

  DofFreezingTask(MjModel& model, val dof_indices, double gain) {
    try {
      impl = std::make_shared<minc::tasks::DofFreezingTask>(
          model.get(), ValToIntVector(dof_indices), gain);
    } catch (const std::exception& e) {
      ThrowJs(std::string("IK Error: ") + e.what());
    }
  }
};

class KineticEnergyRegularizationTask : public IkTask {
 public:
  explicit KineticEnergyRegularizationTask(double cost) {
    try {
      impl = std::make_shared<minc::tasks::KineticEnergyRegularizationTask>(cost);
    } catch (const std::exception& e) {
      ThrowJs(std::string("IK Error: ") + e.what());
    }
  }

  void setDt(double dt) {
    static_cast<minc::tasks::KineticEnergyRegularizationTask*>(impl.get())->set_dt(dt);
  }
};

class IkLimit {
 public:
  virtual ~IkLimit() = default;
  std::shared_ptr<minc::limits::Limit> impl;
};

class ConfigurationLimit : public IkLimit {
 public:
  explicit ConfigurationLimit(MjModel& model, double gain = 0.95) {
    impl = std::make_shared<minc::limits::ConfigurationLimit>(model.get(), gain);
  }
};

class VelocityLimit : public IkLimit {
 public:
  VelocityLimit(MjModel& model, val velocities) {
    impl = std::make_shared<minc::limits::VelocityLimit>(
        model.get(), ValToStringDoubleMap(velocities));
  }
};

class CollisionAvoidanceLimit : public IkLimit {
 public:
  CollisionAvoidanceLimit(MjModel& model, val geom_pairs)
      : CollisionAvoidanceLimit(model, geom_pairs, 0.85, 0.005, 0.01, 0.0) {}

  CollisionAvoidanceLimit(MjModel& model, val geom_pairs, double gain,
                          double min_dist, double detect_dist, double relax) {
    impl = std::make_shared<minc::limits::CollisionAvoidanceLimit>(
        model.get(), ParseCollisionPairs(geom_pairs), gain, min_dist, detect_dist,
        relax);
  }
};

class FreeJointVelocityLimit : public IkLimit {
 public:
  FreeJointVelocityLimit(MjModel& model, val max_linear, val max_angular)
      : FreeJointVelocityLimit(model, max_linear, max_angular, std::string()) {}

  FreeJointVelocityLimit(MjModel& model, val max_linear, val max_angular,
                         const std::string& joint_name) {
    try {
      impl = std::make_shared<minc::limits::FreeJointVelocityLimit>(
          model.get(), ParseOptionalVec3(max_linear), ParseOptionalVec3(max_angular),
          joint_name);
    } catch (const std::exception& e) {
      ThrowJs(std::string("IK Error: ") + e.what());
    }
  }
};

val solveIK(IkConfiguration& configuration, const val& tasks, const val& limits,
            double dt, double damping, val constraints) {
  try {
    std::vector<std::shared_ptr<minc::tasks::BaseTask>> task_list;
    const int n_tasks = tasks["length"].as<int>();
    task_list.reserve(n_tasks);
    for (int i = 0; i < n_tasks; ++i) {
      IkTask& task = tasks[i].as<IkTask&>();
      if (!task.impl) {
        throw minc::MinkError("Invalid IK task at index " + std::to_string(i));
      }
      task_list.push_back(task.impl);
    }

    std::vector<std::shared_ptr<minc::limits::Limit>> limit_list;
    if (!limits.isNull() && !limits.isUndefined()) {
      const int n_limits = limits["length"].as<int>();
      limit_list.reserve(n_limits);
      for (int i = 0; i < n_limits; ++i) {
        IkLimit& limit = limits[i].as<IkLimit&>();
        if (!limit.impl) {
          throw minc::MinkError("Invalid IK limit at index " + std::to_string(i));
        }
        limit_list.push_back(limit.impl);
      }
    }

    std::vector<std::shared_ptr<minc::tasks::Task>> constraint_list;
    if (!constraints.isNull() && !constraints.isUndefined()) {
      const int n_constraints = constraints["length"].as<int>();
      constraint_list.reserve(n_constraints);
      for (int i = 0; i < n_constraints; ++i) {
        IkTask& task = constraints[i].as<IkTask&>();
        auto kinematic = std::dynamic_pointer_cast<minc::tasks::Task>(task.impl);
        if (!kinematic) {
          throw minc::MinkError(
              "constraints must be kinematic Task instances (not "
              "KineticEnergyRegularizationTask)");
        }
        constraint_list.push_back(kinematic);
      }
    }

    const Eigen::VectorXd vel =
        minc::solve_ik(configuration.config(), task_list, dt, "daqp", damping, false,
                       limit_list, constraint_list);
    return VectorToVal(vel);
  } catch (const std::exception& e) {
    ThrowJs(std::string("IK Error: ") + e.what());
    return val::undefined();
  }
}

val solveIK(IkConfiguration& configuration, const val& tasks, const val& limits,
            double dt, double damping) {
  return solveIK(configuration, tasks, limits, dt, damping, val::undefined());
}

EMSCRIPTEN_BINDINGS(mujoco_ik) {
  emscripten::class_<IkConfiguration>("IkConfiguration")
      .constructor<MjModel&, MjData&>()
      .function("update", &IkConfiguration::update)
      .function("updateFromKeyframe", &IkConfiguration::updateFromKeyframe)
      .function("nq", &IkConfiguration::nq)
      .function("nv", &IkConfiguration::nv)
      .function("q", &IkConfiguration::q)
      .function("integrateInplace", &IkConfiguration::integrateInplace)
      .function("getFramePose", &IkConfiguration::getFramePose)
      .function("getCom", &IkConfiguration::getCom);

  emscripten::class_<IkTask>("IkTask");

  emscripten::class_<FrameTask, emscripten::base<IkTask>>("FrameTask")
      .constructor<std::string, std::string, double, double, double, double>()
      .function("setTargetPosQuat", &FrameTask::setTargetPosQuat)
      .function("setTargetFromConfiguration",
                &FrameTask::setTargetFromConfiguration);

  emscripten::class_<PostureTask, emscripten::base<IkTask>>("PostureTask")
      .constructor<MjModel&, double, double, double>()
      .function("setTargetFromConfiguration",
                &PostureTask::setTargetFromConfiguration);

  emscripten::class_<RelativeFrameTask, emscripten::base<IkTask>>(
      "RelativeFrameTask")
      .constructor<std::string, std::string, std::string, std::string, double,
                   double, double, double>()
      .function("setTargetPosQuat", &RelativeFrameTask::setTargetPosQuat)
      .function("setTargetFromConfiguration",
                &RelativeFrameTask::setTargetFromConfiguration);

  emscripten::class_<ComTask, emscripten::base<IkTask>>("ComTask")
      .constructor<double, double, double>()
      .function("setTarget", &ComTask::setTarget)
      .function("setTargetFromConfiguration",
                &ComTask::setTargetFromConfiguration);

  emscripten::class_<DampingTask, emscripten::base<IkTask>>("DampingTask")
      .constructor<MjModel&, double>();

  emscripten::class_<EqualityConstraintTask, emscripten::base<IkTask>>(
      "EqualityConstraintTask")
      .constructor<MjModel&, double>()
      .constructor<MjModel&, double, val, double, double>();

  emscripten::class_<LookAtTask, emscripten::base<IkTask>>("LookAtTask")
      .constructor<std::string, std::string, val, double, double, double>()
      .function("setTarget", &LookAtTask::setTarget)
      .function("setTargetFromConfiguration",
                &LookAtTask::setTargetFromConfiguration);

  emscripten::class_<AxisAlignTask, emscripten::base<IkTask>>("AxisAlignTask")
      .constructor<std::string, std::string, val, double, double, double>()
      .function("setTarget", &AxisAlignTask::setTarget)
      .function("setTargetFromConfiguration",
                &AxisAlignTask::setTargetFromConfiguration);

  emscripten::class_<DofFreezingTask, emscripten::base<IkTask>>("DofFreezingTask")
      .constructor<MjModel&, val>()
      .constructor<MjModel&, val, double>();

  emscripten::class_<KineticEnergyRegularizationTask, emscripten::base<IkTask>>(
      "KineticEnergyRegularizationTask")
      .constructor<double>()
      .function("setDt", &KineticEnergyRegularizationTask::setDt);

  emscripten::class_<IkLimit>("IkLimit");

  emscripten::class_<ConfigurationLimit, emscripten::base<IkLimit>>(
      "ConfigurationLimit")
      .constructor<MjModel&>()
      .constructor<MjModel&, double>();

  emscripten::class_<VelocityLimit, emscripten::base<IkLimit>>("VelocityLimit")
      .constructor<MjModel&, val>();

  emscripten::class_<CollisionAvoidanceLimit, emscripten::base<IkLimit>>(
      "CollisionAvoidanceLimit")
      .constructor<MjModel&, val>()
      .constructor<MjModel&, val, double, double, double, double>();

  emscripten::class_<FreeJointVelocityLimit, emscripten::base<IkLimit>>(
      "FreeJointVelocityLimit")
      .constructor<MjModel&, val, val>()
      .constructor<MjModel&, val, val, std::string>();

  emscripten::function(
      "solveIK",
      emscripten::select_overload<val(IkConfiguration&, const val&, const val&,
                                      double, double)>(&solveIK));
  emscripten::function(
      "solveIK",
      emscripten::select_overload<val(IkConfiguration&, const val&, const val&,
                                      double, double, val)>(&solveIK));
}

}  // namespace mujoco::wasm
