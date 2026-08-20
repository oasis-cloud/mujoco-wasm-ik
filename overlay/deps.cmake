# Third-party IK deps live next to the mujoco checkout:
#   vendor/{mujoco,minc,daqp,eigen}

set(MINK_DEPS_DIR "${PROJECT_SOURCE_DIR}/..")
set(MINC_DIR "${MINK_DEPS_DIR}/minc")
set(DAQP_DIR "${MINK_DEPS_DIR}/daqp")
set(EIGEN_DIR "${MINK_DEPS_DIR}/eigen")

if(NOT EXISTS "${MINC_DIR}/include/minc/configuration.h")
  message(FATAL_ERROR "Minc not found at ${MINC_DIR}")
endif()
if(NOT EXISTS "${DAQP_DIR}/include/daqp.h")
  message(FATAL_ERROR "DAQP not found at ${DAQP_DIR}")
endif()
if(NOT EXISTS "${EIGEN_DIR}/Eigen/Dense")
  message(FATAL_ERROR "Eigen not found at ${EIGEN_DIR}")
endif()

add_library(daqp_wasm STATIC
  "${DAQP_DIR}/src/api.c"
  "${DAQP_DIR}/src/auxiliary.c"
  "${DAQP_DIR}/src/bnb.c"
  "${DAQP_DIR}/src/daqp.c"
  "${DAQP_DIR}/src/daqp_prox.c"
  "${DAQP_DIR}/src/factorization.c"
  "${DAQP_DIR}/src/hierarchical.c"
  "${DAQP_DIR}/src/utils.c"
  "${DAQP_DIR}/codegen/codegen.c"
)
target_include_directories(daqp_wasm PUBLIC
  "${DAQP_DIR}/include"
  "${DAQP_DIR}/codegen"
)
set_target_properties(daqp_wasm PROPERTIES
  C_STANDARD 11
  POSITION_INDEPENDENT_CODE ON
)

add_library(minc_ik STATIC
  "${MINC_DIR}/src/configuration.cpp"
  "${MINC_DIR}/src/constants.cpp"
  "${MINC_DIR}/src/exceptions.cpp"
  "${MINC_DIR}/src/utils.cpp"
  "${MINC_DIR}/src/lie/se3.cpp"
  "${MINC_DIR}/src/lie/so3.cpp"
  "${MINC_DIR}/src/lie/utils.cpp"
  "${MINC_DIR}/src/tasks/task.cpp"
  "${MINC_DIR}/src/tasks/frame_task.cpp"
  "${MINC_DIR}/src/tasks/posture_task.cpp"
  "${MINC_DIR}/src/tasks/relative_frame_task.cpp"
  "${MINC_DIR}/src/tasks/com_task.cpp"
  "${MINC_DIR}/src/tasks/damping_task.cpp"
  "${MINC_DIR}/src/tasks/equality_constraint_task.cpp"
  "${MINC_DIR}/src/tasks/look_at_task.cpp"
  "${MINC_DIR}/src/tasks/axis_align_task.cpp"
  "${MINC_DIR}/src/tasks/dof_freezing_task.cpp"
  "${MINC_DIR}/src/tasks/kinetic_energy_regularization_task.cpp"
  "${MINC_DIR}/src/limits/limit.cpp"
  "${MINC_DIR}/src/limits/configuration_limit.cpp"
  "${MINC_DIR}/src/limits/velocity_limit.cpp"
  "${MINC_DIR}/src/limits/collision_avoidance_limit.cpp"
  "${MINC_DIR}/src/limits/free_joint_velocity_limit.cpp"
  "${MINC_DIR}/src/solve_ik.cpp"
)
target_include_directories(minc_ik PUBLIC
  "${MINC_DIR}/include"
  "${EIGEN_DIR}"
)
target_link_libraries(minc_ik PUBLIC daqp_wasm)
target_link_libraries(minc_ik PRIVATE mujoco)
target_compile_features(minc_ik PUBLIC cxx_std_20)
target_compile_options(minc_ik PRIVATE -fexceptions)
