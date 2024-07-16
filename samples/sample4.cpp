#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "builder/static_var.h"
#include "cscalar.h"
#include "cscalar_impl.h"

using builder::dyn_var;
using builder::static_var;

void blah() {
  SpatialAlgebra::cscalard a(1);
  SpatialAlgebra::cscalard b(3);
  SpatialAlgebra::cscalard x;

  x = a + b;
}

int main(int argc, char* argv[]) {
  // parse URDF file into link and joint tree format
  // do this at runtime because compile-time XML parsing
  // not required.
  SpatialAlgebra::generate_spatial_algebra_program(blah, "blah", std::cout);
  return 0;
}

