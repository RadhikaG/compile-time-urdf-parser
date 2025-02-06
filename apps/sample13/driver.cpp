#include "Eigen/Dense"
#include "fk_gen.h"
#include <iostream>

const size_t N_X_T = 3;

// clang-format off
const double data[N_X_T][36] = {
  {1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0.686,0,1,0,0,-0.686,0,0.06,0,1,0,0,-0.06,0,0,0,1},
  {0.707105,0.707108,0,0,0,0,-0.707108,0.707105,0,0,0,0,0,0,1,0,0,0,-0.0916596,0.0916593,-0.137886,0.707105,0.707108,0,-0.0916593,-0.0916596,0.228434,-0.707108,0.707105,0,0.259027,-0.0640272,0,0,0,1},
  {1,0,0,0,0,0,0,4.89664e-12,-1,0,0,0,0,1,4.89664e-12,0,0,0,0,0.27035,0,1,0,0,-1.32381e-12,0.069,3.37868e-13,0,4.89664e-12,-1,-0.27035,-3.37868e-13,0.069,0,1,4.89664e-12}
};
// clang-format on

static void set_X_T(Eigen::Matrix<double, 6, 6> X_T[]) {
  size_t r, c;

  for (size_t i = 1; i < N_X_T; i++) {
    Eigen::Matrix<double, 6, 6> pin_X_T(data[i]);

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

int main(int argc, char *argv[]) {
  Eigen::Matrix<double, 6, 6> X_T[N_X_T];

  const size_t i = 2;

  Eigen::Matrix<double, 6, 6> X_J, X_pi, X_0;
  X_J.setZero();
  X_pi.setZero();
  X_0.setZero();

  set_X_T(X_T);

  Eigen::VectorXd q(2);
  q.coeffRef(0) = 0.2;
  q.coeffRef(1) = 0.3;

  Eigen::Matrix3d E, rcross, minus_E_rcross;
  E.setZero();
  rcross.setZero(), minus_E_rcross.setZero();

  double cosq = cos(q(i-1));
  double sinq = sin(q(i-1));

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

  X_J.block<3, 3>(0, 0) = E;
  X_J.block<3, 3>(3, 3) = E;
  X_J.block<3, 3>(3, 0) = minus_E_rcross;

  X_pi = X_J * X_T[i];

  X_0 = X_pi;

  std::cout << "Eigen X_J:\n" << X_J << "\n";
  std::cout << "Eigen X_T:\n" << X_T[i] << "\n";
  std::cout << "Eigen X_pi:\n" << X_pi << "\n";
  std::cout << "Eigen X_0:\n" << X_0 << "\n";
  std::cout << "--------\n";
  X_0 = X_J;
  std::cout << "Eigen X_0 X_J:\n" << X_0 << "\n";
  X_0 = X_T[i];
  std::cout << "Eigen X_0 X_T:\n" << X_0 << "\n";

  Eigen::Matrix<double, 6, 6> us_X_0 = ctup_gen::fk(q);

  if (us_X_0.isApprox(us_X_0, 1e-5))
    return 0;
  else
    return 1;
}

