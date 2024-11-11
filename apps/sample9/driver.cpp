#include "fk_gen.h"
#include <iostream>

int main() {
  
  Eigen::VectorXd q(2);
  q.coeffRef(0) = 0.2;
  q.coeffRef(1) = 0.3;

  Eigen::Matrix<double, 6, 6> X1, X2;

  double x = 0.06;
  double y = 0.00;
  double z = 0.686;

  X1.setIdentity();
  X2.setIdentity();

  // setting translation, negative(r-cross)
  X1.coeffRef(0, 0+3) = 0;
  X1.coeffRef(1, 1+3) = 0;
  X1.coeffRef(2, 2+3) = 0;
  X1.coeffRef(1, 0+3) = z;
  X1.coeffRef(0, 1+3) = -z;
  X1.coeffRef(2, 0+3) = -y;
  X1.coeffRef(0, 2+3) = y;
  X1.coeffRef(2, 1+3) = x;
  X1.coeffRef(1, 2+3) = -x;

  double cosq = cos(q(1));
  double sinq = sin(q(1));

  X2.coeffRef(2, 2) = 1;
  X2.coeffRef(0, 0) = cosq;
  X2.coeffRef(0, 1) = -sinq;
  X2.coeffRef(1, 0) = sinq;
  X2.coeffRef(1, 1) = cosq;

  X2.coeffRef(2+3, 2+3) = 1;
  X2.coeffRef(0+3, 0+3) = cosq;
  X2.coeffRef(0+3, 1+3) = -sinq;
  X2.coeffRef(1+3, 0+3) = sinq;
  X2.coeffRef(1+3, 1+3) = cosq;

  std::cout << "Eigen X1:\n" << X1 << "\n";
  std::cout << "Eigen X2:\n" << X2 << "\n";
  std::cout << "Eigen X3:\n" << X1 * X2 << "\n";

  std::cout << "Us:\n" << ctup_gen::fk(q) << "\n";

  Eigen::Matrix<double, 4, 4> a, b;
  a.setZero(); b.setZero();

  a.coeffRef(0, 0) = 0;
  a.coeffRef(0, 1) = 1;
  a.coeffRef(1, 0) = 2;
  a.coeffRef(1, 1) = 3;

  b(0, 0) = 0;
  b(0, 1) = 1;
  b(1, 0) = 2;
  b(1, 1) = 3;

  std::cout << "a:\n" << a << "\n";
  std::cout << "b:\n" << b << "\n";
}
