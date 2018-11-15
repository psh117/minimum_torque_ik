#pragma once

#include <vector>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/test_vars_constr_cost.h>

#include "ik_force/robot_model/robot_model.h"
#include "ik_force/optimization/ik_optimization_var_constr_cost.h"


struct TargetTransformWrench
{
  Eigen::Affine3d transform_; ///< Target transform w.r.t. world frame
  Eigen::Vector3d force_; ///< Target force w.r.t. world frame
  Eigen::Vector3d torque_; ///< Target torque w.r.t. world frame
};

/**
 * @brief The IKForce class
 * @details
 * The Inverse Kinematics algorithm minimizing torque \n
 * It optimizes robot configuration q that minimizes \n
 * the torque when an arbitrary wrench is given.
 */
class IKForce
{
public:
  IKForce(RobotModelPtr model);

  void setBaseFrameTransform(const Eigen::Affine3d& transform);
  void setTargets(const std::vector<TargetTransformWrench>& targets);
  void addTarget(const TargetTransformWrench& target);
  void clearTarget();
  std::vector<Eigen::VectorXd> getSolutions();

  void solve();

private:
  RobotModelPtr model_;

  Eigen::Affine3d base_frame_transform_; ///< World to Base of manipulator Trnasform

  std::vector<TargetTransformWrench> target_; ///< Target transform and wrench
  std::vector<Eigen::VectorXd> solution_;
  Eigen::VectorXd initial_configuration_; ///< $q_0$
};
