// Bring in my package's API, which is what I'm testing
#include "ik_force/ik_force.h"
#include "ik_force/robot_model/franka_panda_model.h"

#include <ros/ros.h>
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
using namespace std;

TEST(IKSolverTest, testCase1)
{
  TargetTransformWrench target;

  Eigen::Affine3d right_arm_tf;
  right_arm_tf.translation() << 0.1481, -0.1559, 0.5168;
  right_arm_tf.linear() = Eigen::AngleAxisd( M_PI_4, Eigen::Vector3d::UnitX()).toRotationMatrix();

  target.transform_.matrix() <<
                             1,           0,           0,         0.6,
                             0, 6.12323e-17,           1,        -0.2,
                             0,          -1, 6.12323e-17,        0.85,
                             0,           0,           0,           1;

  target.force_ <<          0,        1.75,        0.35 ;
  target.torque_ << 4.33681e-18,           0,           0;

  RobotModelPtr model = std::make_shared<FrankaPandaModel>();

  IKForce ik_force(model);
  ik_force.setBaseFrameTransform(right_arm_tf);
  ik_force.addTarget(target);
  ik_force.solve();
  auto solutions = ik_force.getSolutions();

  auto x = model->getTranslation(solutions[0]);

  Eigen::Vector4d x_1;
  x_1 << x, 1;

  auto x_global = right_arm_tf * x_1;

  EXPECT_NEAR(x_global(0), 0.6, 0.001);
  EXPECT_NEAR(x_global(1), -0.2, 0.001);
  EXPECT_NEAR(x_global(2), 0.85, 0.001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ik_force_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
