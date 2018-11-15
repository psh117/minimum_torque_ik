
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include <rbdl/rbdl.h>

#include "robot_model.h"

namespace ifopt {

class IKJointVariables : public VariableSet {
public:
  IKJointVariables(RobotModelPtr& model) : IKJointVariables(model, "ik_joint_var_set") {}
  IKJointVariables(RobotModelPtr& model, const std::string& name) :
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

  void SetVariables(const VectorXd& x) override
  {
    q_ = x;
  }

  VectorXd GetValues() const override
  {
    return q_;
  }

  VecBound GetBounds() const override
  {
    // TOOD: This is only for Franka-Emika Panda
    VecBound b(GetRows());
    Eigen::MatrixXd joint_limits = model_->getJointLimit();
    for(int i=0; i<GetRows(); i++)
    {
      b[i] = Bounds(joint_limits(i,0), joint_limits(i,1));
    }
    return b;
  }

  void setInitValue(VectorXd q) { q_ = q; }

private:
  Eigen::VectorXd q_;
  RobotModelPtr model_;
};

class IkConstraint1 : public ConstraintSet {
public:
  IkConstraint1(RobotModelPtr& model) : IkConstraint1(model, "ik_constraint1") {}
  IkConstraint1(RobotModelPtr& model, const std::string& name) :
    ConstraintSet(model_->getDof(), name),
    model_(model) {}

  VectorXd GetValues() const override
  {
    VectorXd g;
    g = GetVariables()->GetComponent("ik_joint_var_set")->GetValues();
    return g;
  }

  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    Eigen::MatrixXd joint_limits = model_->getJointLimit();
    for(int i=0; i<GetRows(); i++)
    {
      b[i] = Bounds(joint_limits(i,0), joint_limits(i,1));
    }
    return b;
  }

  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override
  {
    if (var_set == "ik_joint_var_set") {
      jac_block.setIdentity();
    }
  }

private:
  RobotModelPtr model_;
};

class IkConstraintPosition : public ConstraintSet {
public:
  IkConstraintPosition(RobotModelPtr& model) : IkConstraintPosition(model, "ik_constraint_position") {}
  IkConstraintPosition(RobotModelPtr& model, const std::string& name) :
    ConstraintSet(3, name),
    model_(model) {}

  VectorXd GetValues() const override
  {
    VectorXd q;
    q = GetVariables()->GetComponent("ik_joint_var_set")->GetValues();
    Eigen::Vector3d g = model_->getTranslation(q);
    std::cout << "getTranslation" << std::endl;
    std::cout << g.transpose() << std::endl;
    return g;
  }

  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    for(int i=0; i<GetRows(); i++)
    {
      b[i] = Bounds(target_translation_(i) - epsilon_,
                    target_translation_(i) + epsilon_);
    }
    return b;
  }

  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override
  {
    if (var_set == "ik_joint_var_set") {
      VectorXd q = GetVariables()->GetComponent("ik_joint_var_set")->GetValues();
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
  void setTargetTranslation(const Eigen::Vector3d & translation)
  {
    target_translation_ = translation;
  }

private:
  RobotModelPtr model_;
  Eigen::Vector3d target_translation_;
  double epsilon_ {0.0001};  ///< IK solution precision
};

class IkConstraintOrientation : public ConstraintSet {
public:
  IkConstraintOrientation(RobotModelPtr& model) : IkConstraintOrientation(model, "ik_constraint_ori") {}
  IkConstraintOrientation(RobotModelPtr& model, const std::string& name) :
    ConstraintSet(1, name),
    model_(model) {}

  VectorXd GetValues() const override
  {
    VectorXd q;
    q = GetVariables()->GetComponent("ik_joint_var_set")->GetValues();


    auto cur_rot =  model_->getRotation(q);
    Eigen::Matrix3d rot_diff = target_orientation_.transpose() * cur_rot;
    Eigen::VectorXd g(1);
    g(0) = (rot_diff-Eigen::Matrix3d::Identity()).row(2).norm();
    Eigen::Vector3d delphi = target_orientation_ * model_->getPhi(target_orientation_, cur_rot);
    std::cout << "target_orientation_" << std::endl;
    std::cout << target_orientation_ << std::endl;
    std::cout << "cur_rot" << std::endl;
    std::cout << cur_rot << std::endl;
    std::cout << "rot_diff" << std::endl;
    std::cout << rot_diff << std::endl;
    return g;
  }

  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    for(int i=0; i<GetRows(); i++)
    {
      b[i] = Bounds(- epsilon_,
                    + epsilon_);
    }
     return b;
  }

  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override
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
  void setTargetRotation(const Eigen::Matrix3d & rotation)
  {
    target_orientation_ = rotation;
  }

private:
  RobotModelPtr model_;
  Eigen::Matrix3d target_orientation_;
  double epsilon_ {0.01};  ///< IK solution precision
};

class IkCost: public CostTerm {
public:
  IkCost(RobotModelPtr& model) : IkCost(model, "ik_cost") {}
  IkCost(RobotModelPtr& model, const std::string& name) :
    CostTerm(name),
    model_(model) {}

  double GetCost() const override
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

  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override
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

  void setTargetWrench(const Eigen::Matrix<double, 6, 1> & wrench)
  {
    required_wrench_ = wrench;
  }

  void setInitialConfiguration(const Eigen::VectorXd & q0)
  { q0_ = q0; use_initial_condition_term_ = true; }
  void disableInitialConfiguration() { use_initial_condition_term_ = false; }

private:
  RobotModelPtr model_;
  Eigen::Matrix<double, 6, 1> required_wrench_;
  Eigen::VectorXd q0_;
  double lambda_{20.0};
  bool use_initial_condition_term_ {false};
};

} // namespace opt

