
#include "ik_force/optimization/ik_optimization_var_constr_cost.h"

namespace ifopt {

IKJointVariables::IKJointVariables(RobotModelPtr& model) : IKJointVariables(model, "ik_joint_var_set") {}
IKJointVariables::IKJointVariables(RobotModelPtr& model, const std::string& name) :
  VariableSet(model->getDof(), name),
  model_(model)
{
  // Set the initial values
  q_.setZero(model->getDof());
  q_(0) = -1.5; // RIGHT
  // q_(0) = 1.5 // LEFT
  q_(3) = -1.5;
  q_(5) = 1.5;
}

void IKJointVariables::SetVariables(const Eigen::VectorXd& x)
{
  q_ = x;
}

Eigen::VectorXd IKJointVariables::GetValues() const
{
  return q_;
}

Component::VecBound IKJointVariables::GetBounds() const
{
  // TOOD: This is only for Franka-Emika Panda
  Component::VecBound b(GetRows());
  Eigen::MatrixXd joint_limits = model_->getJointLimit();
  for(int i=0; i<GetRows(); i++)
  {
    b[i] = Bounds(joint_limits(i,0), joint_limits(i,1));
  }
  return b;
}

void IKJointVariables::setInitValue(Eigen::VectorXd q) { q_ = q; }


// IkConstraintPosition
IkConstraintPosition::IkConstraintPosition(RobotModelPtr& model) : IkConstraintPosition(model, "ik_constraint_position") {}
IkConstraintPosition::IkConstraintPosition(RobotModelPtr& model, const std::string& name) :
  ConstraintSet(3, name),
  model_(model) {}

Eigen::VectorXd IkConstraintPosition::GetValues() const
{
  Eigen::VectorXd q;
  q = GetVariables()->GetComponent("ik_joint_var_set")->GetValues();
  Eigen::Vector3d g = model_->getTranslation(q);
//  std::cout << "getTranslation" << std::endl;
//  std::cout << g.transpose() << std::endl;
  return g;
}

Component::VecBound IkConstraintPosition::GetBounds() const
{
  Component::VecBound b(GetRows());
  for(int i=0; i<GetRows(); i++)
  {
    b[i] = Bounds(target_translation_(i) - epsilon_,
                  target_translation_(i) + epsilon_);
  }
  return b;
}

void IkConstraintPosition::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  if (var_set == "ik_joint_var_set") {
    Eigen::VectorXd q = GetVariables()->GetComponent("ik_joint_var_set")->GetValues();
    auto jv = model_->getJacobianMatrix(q).block(0, 0, 3, model_->getDof());
    for(int i=0; i<3; i++)
    {
      for(int j=0; j<model_->getDof(); j++)
      {
        jac_block.coeffRef(i,j) = jv(i,j);
      }
    }
  }
}
void IkConstraintPosition::setTargetTranslation(const Eigen::Vector3d & translation)
{
  target_translation_ = translation;
}

// IkConstraintOrientation
IkConstraintOrientation::IkConstraintOrientation(RobotModelPtr& model) : IkConstraintOrientation(model, "ik_constraint_ori") {}
IkConstraintOrientation::IkConstraintOrientation(RobotModelPtr& model, const std::string& name) :
  ConstraintSet(1, name),
  model_(model) {}

Eigen::VectorXd IkConstraintOrientation::GetValues() const
{
  Eigen::VectorXd q;
  q = GetVariables()->GetComponent("ik_joint_var_set")->GetValues();


  auto cur_rot =  model_->getRotation(q);
  Eigen::Matrix3d rot_diff = target_orientation_.transpose() * cur_rot;
  Eigen::VectorXd g(1);
  g(0) = (rot_diff-Eigen::Matrix3d::Identity()).row(2).norm();
  Eigen::Vector3d delphi = target_orientation_ * model_->getPhi(target_orientation_, cur_rot);
//  std::cout << "target_orientation_" << std::endl;
//  std::cout << target_orientation_ << std::endl;
//  std::cout << "cur_rot" << std::endl;
//  std::cout << cur_rot << std::endl;
//  std::cout << "rot_diff" << std::endl;
//  std::cout << rot_diff << std::endl;
  return g;
}

Component::VecBound IkConstraintOrientation::GetBounds() const
{
  Component::VecBound b(GetRows());
  for(int i=0; i<GetRows(); i++)
  {
    b[i] = Bounds(- epsilon_,
                  + epsilon_);
  }
  return b;
}

void IkConstraintOrientation::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  if (var_set == "ik_joint_var_set") {
    double h = 0.01;
    Eigen::VectorXd q = GetVariables()->GetComponent("ik_joint_var_set")->GetValues();
    for(int i=0; i<model_->getDof(); i++)
    {
      Eigen::VectorXd q_tmp;
      q_tmp = q;
      q_tmp(i) = q(i) + h;
      auto  cur_rot1 =  model_->getRotation(q_tmp);
      Eigen::Matrix3d rot_diff1 = target_orientation_.transpose() * cur_rot1;
      double g1 = (rot_diff1-Eigen::Matrix3d::Identity()).row(2).norm();
      q_tmp(i) = q(i) - h;
      auto  cur_rot2 =  model_->getRotation(q_tmp);
      Eigen::Matrix3d rot_diff2 = target_orientation_.transpose() * cur_rot2;
      double g2 = (rot_diff2-Eigen::Matrix3d::Identity()).row(2).norm();
      jac_block.coeffRef(0,i) = (g1-g2) / (2*h);
    }
  }
}
void IkConstraintOrientation::setTargetRotation(const Eigen::Matrix3d & rotation)
{
  target_orientation_ = rotation;
}

// IkCost
IkCost::IkCost(RobotModelPtr& model) : IkCost(model, "ik_cost") {}
IkCost::IkCost(RobotModelPtr& model, const std::string& name) :
  CostTerm(name),
  model_(model) {}

double IkCost::GetCost() const
{
  Eigen::VectorXd q = GetVariables()->GetComponent("ik_joint_var_set")->GetValues();
  Eigen::MatrixXd j = model_->getJacobianMatrix(q);
  auto & F = required_wrench_;

  double cost = (F.transpose() * j * j.transpose() * F);
  if(use_initial_condition_term_)
  {
    cost += lambda_ * (q-q0_).norm();
  }
  //std::cout << "cost" << std::endl << cost << std::endl;
  return cost;
}

void IkCost::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  if (var_set == "ik_joint_var_set") {
    auto & F = required_wrench_;
    double h = 0.01;
    Eigen::VectorXd q = GetVariables()->GetComponent("ik_joint_var_set")->GetValues();
    for(int i=0; i<model_->getDof(); i++)
    {
      Eigen::VectorXd q_tmp;
      q_tmp = q;
      q_tmp(i) = q(i) + h;
      Eigen::MatrixXd j1 = model_->getJacobianMatrix(q_tmp);
      q_tmp(i) = q(i) - h;
      Eigen::MatrixXd j2 = model_->getJacobianMatrix(q_tmp);
      double g1 = F.transpose() * j1 * j1.transpose() * F;
      double g2 = F.transpose() * j2 * j2.transpose() * F;
      jac_block.coeffRef(0,i) = (g1-g2) / (2*h);
      if(use_initial_condition_term_)
      {
        jac_block.coeffRef(0,i) += 2*lambda_ * q(i);
      }
    }
  }
}

void IkCost::setTargetWrench(const Eigen::Matrix<double, 6, 1> & wrench)
{
  required_wrench_ = wrench;
}

void IkCost::setInitialConfiguration(const Eigen::VectorXd & q0)
{ q0_ = q0; use_initial_condition_term_ = true; }
void IkCost::disableInitialConfiguration() { use_initial_condition_term_ = false; }

} // namespace opt

