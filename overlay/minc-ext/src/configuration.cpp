#include "minc/configuration.h"
#include "Eigen/src/Core/util/Constants.h"
#include "minc/constants.h"
#include "minc/exceptions.h"
#include <algorithm>
#include <cstring>
#include <iostream>

namespace minc {

Configuration::Configuration(const mjModel* model, const double* q)
    : model_(model),
      data_(nullptr),
      owned_data_(mj_makeData(model), mj_deleteData) {
    data_ = owned_data_.get();
    update(q);
}

Configuration::Configuration(const mjModel* model, mjData* data)
    : model_(model), data_(data), owned_data_(nullptr, mj_deleteData) {
    if (!data_) {
        throw MinkError("Configuration requires a non-null mjData");
    }
    update();
}

Configuration::~Configuration() = default;

void Configuration::update(const double* q) {
    if (q != nullptr)
        mju_copy(data_->qpos, q, model_->nq);
    
    // The minimal function call required to get updated frame transforms is
    // mj_kinematics. An extra call to mj_comPos is required for updated Jacobians.
    mj_kinematics(model_, data_);
    mj_comPos(model_, data_);
    
    if (model_->neq > 0)
        mj_makeConstraint(model_, data_);
}

void Configuration::update_from_keyframe(const std::string& key_name) {
    int key_id = mj_name2id(model_, mjOBJ_KEY, key_name.c_str());
    if (key_id == -1) {
        throw InvalidKeyframe(key_name, model_);
    }
    update(&model_->key_qpos[key_id * model_->nq]);
}

void Configuration::check_limits(double tol, bool safety_break) const {
    for (int jnt = 0; jnt < model_->njnt; ++jnt) {
        mjtJoint jnt_type = static_cast<mjtJoint>(model_->jnt_type[jnt]);
        if (jnt_type == mjJNT_FREE || !model_->jnt_limited[jnt]) {
            continue;
        }
        
        int padr = model_->jnt_qposadr[jnt];
        double qval = data_->qpos[padr];
        double qmin = model_->jnt_range[2 * jnt];
        double qmax = model_->jnt_range[2 * jnt + 1];
        
        if (qval < qmin - tol || qval > qmax + tol) {
            if (safety_break) {
                throw NotWithinConfigurationLimits(jnt, qval, qmin, qmax, model_);
            } else {
                const char* joint_name = mj_id2name(model_, mjOBJ_JOINT, jnt);
                std::cout << "Warning: Joint " << jnt << " (" 
                         << (joint_name ? joint_name : "unnamed") << ") "
                         << "violates configuration limits " << qmin 
                         << " <= " << qval << " <= " << qmax << std::endl;
            }
        }
    }
}

Eigen::MatrixXd Configuration::get_frame_jacobian(const std::string& frame_name, const std::string& frame_type) const {
    // Check if frame type is supported
    if (std::find(SUPPORTED_FRAMES.begin(), SUPPORTED_FRAMES.end(), frame_type) == SUPPORTED_FRAMES.end()) {
        throw UnsupportedFrame(frame_type, SUPPORTED_FRAMES);
    }
    
    // Get frame ID
    auto enum_it = FRAME_TO_ENUM.find(frame_type);
    if (enum_it == FRAME_TO_ENUM.end()) {
        throw UnsupportedFrame(frame_type, SUPPORTED_FRAMES);
    }
    
    int frame_id = mj_name2id(model_, enum_it->second, frame_name.c_str());
    if (frame_id == -1) {
        throw InvalidFrame(frame_name, frame_type, model_);
    }
    
    // Compute Jacobian - MuJoCo functions fill matrices in row-major order
    // Create row-major matrices to match MuJoCo's output format
    Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> jac_pos(3, model_->nv);
    Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> jac_rot(3, model_->nv);
    
    // call_jacobian_function(frame_type, model_, data_, jac_pos.data(), jac_rot.data(), frame_id);
    
    // Stack position and rotation Jacobians
    Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor> jac(6, model_->nv);
    if (frame_type == "site")
        mj_jacSite(model_, data_, jac.row(0).data(), jac.row(3).data(), frame_id);
    else if (frame_type == "body")
        mj_jacBody(model_, data_, jac.row(0).data(), jac.row(3).data(), frame_id);
    else if (frame_type == "geom")
        mj_jacGeom(model_, data_, jac.row(0).data(), jac.row(3).data(), frame_id);
    else
        std::cout << "critical error";
    
    
    // MuJoCo jacobians have a frame of reference centered at the local frame but
    // aligned with the world frame. To obtain a jacobian expressed in the local
    // frame, aka body jacobian, we need to left-multiply by A[T_fw].
    const double* xmat = get_frame_xmat(data_, frame_type, frame_id);
    if (!xmat) {
        throw UnsupportedFrame(frame_type, SUPPORTED_FRAMES);
    }
    
    // Convert rotation matrix from MuJoCo format
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation_matrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation_matrix(i, j) = xmat[3 * i + j];
        }
    }
    
    lie::SO3 R_wf = lie::SO3::from_matrix(rotation_matrix);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_fw = lie::SE3::from_rotation(R_wf.inverse()).adjoint();
    jac = A_fw * jac;
    return jac;
}

lie::SE3 Configuration::get_transform_frame_to_world(const std::string& frame_name, const std::string& frame_type) const {
    // Check if frame type is supported
    if (std::find(SUPPORTED_FRAMES.begin(), SUPPORTED_FRAMES.end(), frame_type) == SUPPORTED_FRAMES.end()) {
        throw UnsupportedFrame(frame_type, SUPPORTED_FRAMES);
    }
    
    // Get frame ID
    auto enum_it = FRAME_TO_ENUM.find(frame_type);
    if (enum_it == FRAME_TO_ENUM.end()) {
        throw UnsupportedFrame(frame_type, SUPPORTED_FRAMES);
    }
    
    int frame_id = mj_name2id(model_, enum_it->second, frame_name.c_str());
    if (frame_id == -1) {
        throw InvalidFrame(frame_name, frame_type, model_);
    }
    
    // Get position and orientation
    const double* xpos = get_frame_pos(data_, frame_type, frame_id);
    const double* xmat = get_frame_xmat(data_, frame_type, frame_id);
    
    if (!xpos || !xmat) {
        throw UnsupportedFrame(frame_type, SUPPORTED_FRAMES);
    }
    
    // Convert to Eigen types
    Eigen::Vector3d position(xpos[0], xpos[1], xpos[2]);
    Eigen::Matrix3d rotation;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation(i, j) = xmat[3 * i + j];
        }
    }
    
    return lie::SE3(lie::SO3::from_matrix(rotation), position);
}

lie::SE3 Configuration::get_transform(const std::string& source_name, const std::string& source_type,
                                     const std::string& dest_name, const std::string& dest_type) const {
    lie::SE3 transform_source_to_world = get_transform_frame_to_world(source_name, source_type);
    lie::SE3 transform_dest_to_world = get_transform_frame_to_world(dest_name, dest_type);
    return transform_dest_to_world.inverse() * transform_source_to_world;
}

Eigen::VectorXd Configuration::integrate(const Eigen::VectorXd& velocity, double dt) const {
    if (velocity.size() != model_->nv) {
        throw std::invalid_argument("Velocity vector size must match model nv");
    }
    
    Eigen::VectorXd q_new(model_->nq);
    std::memcpy(q_new.data(), data_->qpos, model_->nq * sizeof(double));
    
    mj_integratePos(model_, q_new.data(), velocity.data(), dt);
    return q_new;
}

void Configuration::integrate_inplace(const Eigen::VectorXd& velocity, double dt) {
    if (velocity.size() != model_->nv) {
        throw std::invalid_argument("Velocity vector size must match model nv");
    }
    
    mj_integratePos(model_, data_->qpos, velocity.data(), dt);
    update();
}

Eigen::MatrixXd Configuration::get_inertia_matrix() const {
    // MuJoCo >= 3.3.4 / 3.11: mj_makeM fills sparse inertia; mj_fullM densifies.
    // Older releases used mj_crb + data->qM.
#if mjVERSION_HEADER >= 3000000
    mj_makeM(model_, data_);
    Eigen::MatrixXd M(model_->nv, model_->nv);
    mj_fullM(model_, data_, M.data());
    return M;
#else
    if (mj_version() >= 334) {
        mj_makeM(model_, data_);
    } else {
        mj_crb(model_, data_);
    }
    Eigen::MatrixXd M(model_->nv, model_->nv);
    mj_fullM(model_, M.data(), data_->qM);
    return M;
#endif
}

Eigen::VectorXd Configuration::q() const {
    Eigen::VectorXd q_copy(model_->nq);
    std::memcpy(q_copy.data(), data_->qpos, model_->nq * sizeof(double));
    return q_copy;
}

void Configuration::differentiate_pos(const Eigen::VectorXd& target_q, const Eigen::VectorXd& current_q, Eigen::VectorXd& result) const {
    if (target_q.size() != model_->nq || current_q.size() != model_->nq) {
        throw MinkError("Configuration dimension mismatch in differentiate_pos");
    }
    
    if (result.size() != model_->nv) {
        result.resize(model_->nv);
    }
    
    // Use MuJoCo's mj_differentiatePos to properly handle different joint types
    mj_differentiatePos(model_, result.data(), 1.0, target_q.data(), current_q.data());
}

} // namespace minc
