#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "spatial_algebra.h"

using builder::dyn_var;
using builder::static_var;

void eep() {
  SpatialAlgebra::cmatrix<double> a({1,2}), b({1,2}), c({1,2});

  std::cout << "all\n";

  c = a + b;
}

int main(int argc, char* argv[]) {
  // parse URDF file into link and joint tree format
  // do this at runtime because compile-time XML parsing
  // not required.
  SpatialAlgebra::generate_spatial_algebra_program(eep, "eep", std::cout);
  return 0;
}

