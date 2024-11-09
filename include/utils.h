#ifndef UTILS_H
#define UTILS_H
#include "builder/builder_base.h"
#include "builder/static_var.h"
#include "builder/dyn_var.h"
#include <unordered_map>
#include <vector>
#include "backend.h"
#include "xform_impl.h"

#include <sstream>
#include <cassert>
// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

using builder::dyn_var;
using builder::static_var;

namespace ctup {

template<typename Scalar>
void toEigen(dyn_var<builder::eigen_Xmat_t> &mat, Xform<Scalar> &xform) {
  static_var<int> r,c;

  for (c = 0; c < 6; c = c + 1) {
    for (r = 0; r < 6; r = r + 1) {
      mat.coeffRef(c, r) = Xform_expr_leaf<Scalar>(xform).get_value_at(c, r);
    }
  }
}

}

#endif
