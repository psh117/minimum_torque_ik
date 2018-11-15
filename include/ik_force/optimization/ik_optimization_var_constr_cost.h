
#include <iostream>

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include "ik_force/robot_model/robot_model.h"

namespace ifopt {

class IKJointVariables : public VariableSet {
public:
  IKJointVariables(RobotModelPtr& model);
  IKJointVariables(RobotModelPtr& model, const std::string& name);

  void SetVariables(const VectorXd& x) override;
  VectorXd GetValues() const override;
  VecBound GetBounds() const override;

  void setInitValue(VectorXd q);

private:
  Eigen::VectorXd q_;
  RobotModelPtr model_;
};

class IkConstraintPosition : public ConstraintSet {
public:
  IkConstraintPosition(RobotModelPtr& model);
  IkConstraintPosition(RobotModelPtr& model, const std::string& name);

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

  void setTargetTranslation(const Eigen::Vector3d & translation);

private:
  RobotModelPtr model_;
  Eigen::Vector3d target_translation_;
  double epsilon_ {0.0001};  ///< IK solution precision
};

class IkConstraintOrientation : public ConstraintSet {
public:
  IkConstraintOrientation(RobotModelPtr& model);
  IkConstraintOrientation(RobotModelPtr& model, const std::string& name);

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

  void setTargetRotation(const Eigen::Matrix3d & rotation);

private:
  RobotModelPtr model_;
  Eigen::Matrix3d target_orientation_;
  double epsilon_ {0.01};  ///< IK solution precision
};

class IkCost: public CostTerm {
public:
  IkCost(RobotModelPtr& model);
  IkCost(RobotModelPtr& model, const std::string& name);

  double GetCost() const override;
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

  void setTargetWrench(const Eigen::Matrix<double, 6, 1> & wrench);
  void setInitialConfiguration(const Eigen::VectorXd & q0);
  void disableInitialConfiguration();

private:
  RobotModelPtr model_;
  Eigen::Matrix<double, 6, 1> required_wrench_;
  Eigen::VectorXd q0_;
  double lambda_{20.0};
  bool use_initial_condition_term_ {false};
};

} // namespace opt

