#include <ros/ros.h>
// #include "suhan_opt_ik_test/test_const.h"

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/test_vars_constr_cost.h>

#include "franka_panda_model/franka_panda_model.h"
#include "suhan_opt_ik_test/ik_optimization_var_constr_cost.h"

#include "test_var.h"

using namespace ifopt;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ik_test");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");

  Eigen::Affine3d target_global_transform, target_local_transform;
  Eigen::Vector3d target_global_force, target_global_torque;
  Eigen::Vector3d target_local_force, target_local_torque;

  // LEFT
//  target_global_transform.matrix() <<
//                             1,           0,           0,         0.6,
//                             0, 6.12323e-17,          -1,         0.2,
//                             0,           1, 6.12323e-17,        0.85,
//                             0,           0,           0,           1;
  // RIGHT
  target_global_transform.matrix() <<
                             1,           0,           0,         0.6,
                             0, 6.12323e-17,           1,        -0.2,
                             0,          -1, 6.12323e-17,        0.85,
                             0,           0,           0,           1;

  // LEFT
//  target_global_force <<          0,        -1.75,        0.35 ;
//  target_global_torque << 4.33681e-18,           0,           0;

  // RIGHT
  target_global_force <<          0,        1.75,        0.35 ;
  target_global_torque << 4.33681e-18,           0,           0;


  RobotModelPtr model = std::make_shared<FrankaPandaModel>();

  Eigen::Affine3d right_arm_tf;
  Eigen::Affine3d left_arm_tf;
  right_arm_tf.translation() << 0.1481, -0.1559, 0.5168;
  left_arm_tf.translation() << 0.1481, 0.1559, 0.5168;
  left_arm_tf.linear() = Eigen::AngleAxisd( -M_PI_4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  right_arm_tf.linear() = Eigen::AngleAxisd( M_PI_4, Eigen::Vector3d::UnitX()).toRotationMatrix();


  target_local_transform = right_arm_tf.inverse() * target_global_transform;
  target_local_force = right_arm_tf.linear().transpose() * target_global_force;
  target_local_torque = right_arm_tf.linear().transpose() * target_global_torque;

  Eigen::Vector3d target_point;
  //target_point << 0.65, -0.15, 0.4;
  Eigen::Matrix<double, 6, 1> target_wrench;
  //target_wrench << 0.0, 9.8, 1.2, 0., 0., 0.;
  Eigen::Matrix3d target_orientation;
//  target_orientation
//      << 1, 0 ,0,
//      0, 0, 1,
//      0, -1, 0;

  target_point = target_local_transform.translation();
  target_wrench << target_local_force, target_local_torque;
  target_orientation = target_local_transform.linear();

  auto var = std::make_shared<IKJointVariables>(model);
  auto constraint = std::make_shared<IkConstraint2>(model);
  auto constraint_ori = std::make_shared<IkConstraintOrientation>(model);
  auto cost = std::make_shared<IkCost>(model);
  constraint->setTargetTranslation(target_point);
  cost->setTargetWrench(target_wrench);
  constraint_ori->setTargetRotation(target_orientation);

  Eigen::VectorXd q0;
  q0.setZero(model->getDof());
  q0(0) = -1.5; // RIGHT
  // q_(0) = 1.5 // LEFT
  q0(3) = -1.5;
  q0(5) = 1.5;
  cost->setInitialConfiguration(q0);
  // 1. define the problem
  Problem nlp;
  nlp.AddVariableSet  (var);
  nlp.AddConstraintSet(constraint);
  nlp.AddConstraintSet(constraint_ori);
  nlp.AddCostSet      (cost);

  //nlp.AddVariableSet  (std::make_shared<ExVariables>());
  //nlp.AddConstraintSet(std::make_shared<ExConstraint2>());
  //nlp.AddCostSet      (std::make_shared<ExCost2>());
  nlp.PrintCurrent();

  // 2. choose solver and options
  IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("jacobian_approximation", "finite-difference-values");

  // 3 . solve
  ipopt.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << (x * 180./M_PI).transpose() << std::endl;

  // 4. test if solution correct
  //double eps = 1e-5; //double precision
  //assert(1.0-eps < x(0) && x(0) < 1.0+eps);
  //assert(0.0-eps < x(1) && x(1) < 0.0+eps);

}
