#include "ik_force/ik_force.h"

IKForce::IKForce(RobotModelPtr model)
{
  model_ = model;
}

void IKForce::setBaseFrameTransform(const Eigen::Affine3d& transform)
{
  base_frame_transform_ = transform;
}

void IKForce::setIntialConfiguration(const Eigen::VectorXd& q0)
{
  initial_configuration_ = q0;
}

void IKForce::setTargets(const std::vector<TargetTransformWrench>& targets)
{
  target_ = targets;
}

void IKForce::addTarget(const TargetTransformWrench& target)
{
  target_.push_back(target);
}

void IKForce::clearTarget()
{
  target_.clear();
}

std::vector<Eigen::VectorXd> IKForce::getSolutions()
{
  return solution_;
}

void IKForce::solve()
{
  Eigen::VectorXd q0 = initial_configuration_;
  solution_.clear();

  for(auto & target: target_)
  {
    TargetTransformWrench local;
    local.transform_ = base_frame_transform_.inverse() *
        target.transform_;
    local.force_ = base_frame_transform_.linear().transpose() *
        target.force_;
    local.torque_ = base_frame_transform_.linear().transpose() *
        target.torque_;

    Eigen::Vector3d target_point;
    Eigen::Matrix<double, 6, 1> target_wrench;
    Eigen::Matrix3d target_orientation;

    target_point = local.transform_.translation();
    target_wrench << local.force_, local.torque_;
    target_orientation = local.transform_.linear();

    std::shared_ptr<ifopt::IKJointVariables> var = std::make_shared<ifopt::IKJointVariables>(model_);
    std::shared_ptr<ifopt::IkConstraintPosition> constraint_pos = std::make_shared<ifopt::IkConstraintPosition>(model_);
    std::shared_ptr<ifopt::IkConstraintOrientation> constraint_ori = std::make_shared<ifopt::IkConstraintOrientation>(model_);
    std::shared_ptr<ifopt::IkCost> cost = std::make_shared<ifopt::IkCost>(model_);

    cost->setTargetWrench(target_wrench);
    constraint_pos->setTargetTranslation(target_point);
    constraint_ori->setTargetRotation(target_orientation);

    cost->setInitialConfiguration(q0);


    ifopt::Problem nlp;

    nlp.AddVariableSet  (var);
    nlp.AddConstraintSet(constraint_pos);
    nlp.AddConstraintSet(constraint_ori);
    nlp.AddCostSet      (cost);

    nlp.PrintCurrent();

    ifopt::IpoptSolver ipopt;
    ipopt.SetOption("linear_solver", "mumps");
    ipopt.SetOption("jacobian_approximation", "finite-difference-values");

    ipopt.Solve(nlp);

    Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
    solution_.push_back(x);

    q0 = x;
  }
}
