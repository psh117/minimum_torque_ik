#pragma once

#include <memory>
#include <Eigen/Dense>

class RobotModel
{
public:
  RobotModel() {}

  virtual Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd &q) = 0;
  virtual Eigen::Vector3d getTranslation(const Eigen::VectorXd &q) = 0;
  virtual Eigen::Matrix3d getRotation(const Eigen::VectorXd &q) = 0;
  virtual Eigen::Affine3d getTransform(const Eigen::VectorXd &q) = 0;
  virtual Eigen::MatrixXd getJointLimit() = 0;
  virtual Eigen::VectorXd getInitialConfiguration() = 0;

  virtual int getDof() = 0;

  Eigen::Vector3d getPhi(const Eigen::Matrix3d a, const Eigen::Matrix3d b)
  {
    Eigen::Vector3d phi;
    Eigen::Vector3d s[3], v[3], w[3];

    for (int i = 0; i < 3; i++) {
      v[i] = a.block<3, 1>(0, i);
      w[i] = b.block<3, 1>(0, i);
      s[i] = v[i].cross(w[i]);
    }
    phi = s[0] + s[1] + s[2];
    phi = -0.5* phi;

    return phi;
  }
};

typedef std::shared_ptr<RobotModel> RobotModelPtr;
