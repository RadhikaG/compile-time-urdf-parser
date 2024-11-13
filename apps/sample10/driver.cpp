// $ apps/sample10/driver deps/pinocchio/models/baxter_simple.urdf
//
// ignore unused header warning in IDE, this is needed
#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "Eigen/Dense"
#include "fk_gen.h"
#include <iostream>

using pinocchio::Model;

void set_X_T(Eigen::Matrix<double, 6, 6> X_T[], const Model &model) {
  typedef typename Model::JointIndex JointIndex;
  JointIndex i;

  int r;
  int c;

  for (i = 1; i < (JointIndex)model.njoints; i = i+1) {
    Eigen::Matrix<double, 6, 6> pin_X_T = model.jointPlacements[i];

    // setting E
    for (r = 0; r < 6; r = r + 1) {
      for (c = 0; c < 6; c = c + 1) {
        double entry = pin_X_T.coeffRef(c, r);
        if (std::abs(entry) < 1e-5)
          X_T[i].coeffRef(r, c) = 0;
        else
          X_T[i].coeffRef(r, c) = entry;
      }
    }
  }
}

int main(int argc, char* argv[]) {
  const std::string urdf_filename = argv[1];
  std::cout << urdf_filename << "\n";

  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  Eigen::Matrix<double, 6, 6> X_T[model.njoints];

  Eigen::Matrix<double, 6, 6> X1, X2;
  X1.setZero(); X2.setZero();

  set_X_T(X_T, model);

  Eigen::VectorXd q(2);
  q.coeffRef(0) = 0.2;
  q.coeffRef(1) = 0.3;

  Eigen::Matrix3d E, rcross, minus_E_rcross;
  E.setZero(); rcross.setZero(), minus_E_rcross.setZero();

  double cosq = cos(q(1));
  double sinq = sin(q(1));

  E.coeffRef(2, 2) = 1;
  E.coeffRef(0, 0) = cosq;
  E.coeffRef(0, 1) = sinq;
  E.coeffRef(1, 0) = -sinq;
  E.coeffRef(1, 1) = cosq;

  double x = 0;
  double y = 0;
  double z = 0;
  //double x = 0.06;
  //double y = 0.00;
  //double z = 0.686;

  rcross.coeffRef(0, 0) = 0;
  rcross.coeffRef(1, 1) = 0;
  rcross.coeffRef(2, 2) = 0;
  rcross.coeffRef(1, 0) = z;
  rcross.coeffRef(0, 1) = -z;
  rcross.coeffRef(2, 0) = -y;
  rcross.coeffRef(0, 2) = y;
  rcross.coeffRef(2, 1) = x;
  rcross.coeffRef(1, 2) = -x;

  minus_E_rcross = -E * rcross;

  X1.block<3, 3>(0, 0) = E;
  X1.block<3, 3>(3, 3) = E;
  X1.block<3, 3>(3, 0) = minus_E_rcross;

  X2 = X1 * X_T[1];

  std::cout << "Eigen X1:\n" << X1 << "\n";
  std::cout << "Eigen X_T[1]:\n" << X_T[1] << "\n";
  std::cout << "Eigen X2:\n" << X2 << "\n";

  ctup_gen::fk(q);
}
