#include "blaze/Math.h"
#include "fk_gen.h"
#include <iostream>

const size_t N_X_T = 3;
const size_t SIMD_WIDTH = 8UL;

typedef blaze::StaticVector<double, SIMD_WIDTH> blazeSIMDVecd;

// clang-format off
const double data[N_X_T][36] = {
  {1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0.686,0,1,0,0,-0.686,0,0.06,0,1,0,0,-0.06,0,0,0,1},
  {0.707105,0.707108,0,0,0,0,-0.707108,0.707105,0,0,0,0,0,0,1,0,0,0,-0.0916596,0.0916593,-0.137886,0.707105,0.707108,0,-0.0916593,-0.0916596,0.228434,-0.707108,0.707105,0,0.259027,-0.0640272,0,0,0,1},
  {1,0,0,0,0,0,0,4.89664e-12,-1,0,0,0,0,1,4.89664e-12,0,0,0,0,0.27035,0,1,0,0,-1.32381e-12,0.069,3.37868e-13,0,4.89664e-12,-1,-0.27035,-3.37868e-13,0.069,0,1,4.89664e-12}
};
// clang-format on

static void set_X_T(blaze::StaticMatrix<double, 6, 6> X_T[]) {
  size_t r, c;

  for (size_t i = 1; i < N_X_T; i++) {
    Eigen::Matrix<double, 6, 6> pin_X_T(data[i]);

    for (r = 0; r < 6; r = r + 1) {
      for (c = 0; c < 6; c = c + 1) {
        double entry = pin_X_T(c, r);
        if (std::abs(entry) < 1e-5)
          X_T[i](r, c) = 0;
        else
          X_T[i](r, c) = entry;
      }
    }
  }
}

int main(int argc, char *argv[]) {
  blaze::StaticMatrix<double, 6, 6> X_T[N_X_T];

  blaze::StaticMatrix<blazeSIMDVecd, 6, 6> X1, X2;
  X1.reset();
  X2.reset();

  set_X_T(X_T);

  blaze::StaticVector<blazeSIMDVecd, 3> q;
  q[0] = 0.2;
  q[1] = 0.3;

  blaze::StaticMatrix<blazeSIMDVecd, 3, 3> E, rcross, minus_E_rcross;
  E.reset();
  rcross.reset(), minus_E_rcross.reset();

  blazeSIMDVecd cosq = cos(q[1]);
  blazeSIMDVecd sinq = sin(q[1]);

  E(2, 2) = 1;
  E(0, 0) = cosq;
  E(0, 1) = sinq;
  E(1, 0) = -sinq;
  E(1, 1) = cosq;

  blazeSIMDVecd x, y, z;
  x = 0;
  y = 0;
  z = 0;
  //double x = 0.06;
  //double y = 0.00;
  //double z = 0.686;

  rcross(0, 0) = 0;
  rcross(1, 1) = 0;
  rcross(2, 2) = 0;
  rcross(1, 0) = z;
  rcross(0, 1) = -z;
  rcross(2, 0) = -y;
  rcross(0, 2) = y;
  rcross(2, 1) = x;
  rcross(1, 2) = -x;

  minus_E_rcross = -E * rcross;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      X1(i, j) = E(i, j);
      X1(i+3, j+3) = E(i, j);
      X1(i+3, j) = minus_E_rcross(i, j);
    }
  }

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      X2(i, j) = 0;
      for (int k = 0; k < 6; k++) {
        X2(i, j) += X1(i, k) * X_T[1](k, j);
      }
    }
  }

  std::cout << "Blaze X1:\n";
  ctup_gen::print_matrix(X1);
  std::cout << "Blaze X_T[1]:\n";
  ctup_gen::print_matrix(X_T[1]);
  std::cout << "Blaze X2:\n";
  ctup_gen::print_matrix(X2);

  blaze::StaticMatrix<blazeSIMDVecd, 6, 6> us_X2 = ctup_gen::fk(q);

  blaze::StaticMatrix<blazeSIMDVecd, 6, 6> diff, abs_diff;
  diff = us_X2 - X2;
  abs_diff = abs(diff);

  std::cout << "errors:\n";
  ctup_gen::print_matrix(abs_diff);

  //if ()
  //  return 0;
  //else
  //  return 1;
}

