#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "builder/static_var.h"
#include "cscalar.h"

using builder::dyn_var;
using builder::static_var;

void ree() {
  std::vector<dyn_var<double>*> a, b, c;

  for (static_var<int> i = 0; i < 4; i = i+1) {
    dyn_var<double>& l = *new dyn_var<double>(i);
    dyn_var<double>& p = *new dyn_var<double>(i+2);
    dyn_var<double>& q = *new dyn_var<double>;
    a.push_back(l.addr());
    b.push_back(p.addr());
    c.push_back(q.addr());
  }

  for (static_var<int> i = 0; i < 4; i = i+1) {
    *c[i] = *a[i] + *b[i];
  }
}

int main(int argc, char* argv[]) {
  SpatialAlgebra::generate_spatial_algebra_program(ree, "ree", std::cout);
  return 0;
}

