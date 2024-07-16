#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "builder/static_var.h"
#include "cscalar.h"
#include "cscalar_impl.h"

using builder::dyn_var;
using builder::static_var;

void meh() {
  SpatialAlgebra::cscalard a, b;
  a = 2;
  b = 3;
  SpatialAlgebra::cscalard sum;
  sum = 0;
  sum += a; 
  sum += b; 
}

int main(int argc, char* argv[]) {
  SpatialAlgebra::generate_spatial_algebra_program(meh, "meh", std::cout);
  return 0;
}

