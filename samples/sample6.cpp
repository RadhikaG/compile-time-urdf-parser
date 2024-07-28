#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "builder/static_var.h"
#include "spatial_algebra.h"

using builder::dyn_var;
using builder::static_var;

void shonan(dyn_var<int**> a, dyn_var<int*> v, dyn_var<int*> res) {

  static_var<int> n = 3;

  static_var<int> sparsity_pattern[] = {
    1, 0, 1,
    0, 1, 1,
    1, 0, 0
  };

  static_var<int> i = 0;
  static_var<int> j;

  for (; i < n; i = i+1) {
    for (j = 0; j < n; j = j+1) {
      if (sparsity_pattern[i*n + j] == 1) {
        res[i] += a[i][j] * v[j]; 
      }
    }
  }
}

int main(int argc, char* argv[]) {
  SpatialAlgebra::generate_spatial_algebra_program(shonan, "shonan", std::cout);
  return 0;
}
