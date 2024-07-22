#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "spatial_algebra.h"

using builder::dyn_var;
using builder::static_var;

void eep() {
  Eigen::MatrixXcsd a(2,2);
  a << 
      1, 0,
      0, 1;

  Eigen::MatrixXcsd b(2,2);
  b << 
      0, 6,
      7, 8;

  Eigen::MatrixXcsd c(2,2);

  c = (a + b).eval();
}

int main(int argc, char* argv[]) {
  // parse URDF file into link and joint tree format
  // do this at runtime because compile-time XML parsing
  // not required.
  SpatialAlgebra::generate_spatial_algebra_program(eep, "eep", std::cout);
  return 0;
}
