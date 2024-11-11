#ifndef UTILS_H
#define UTILS_H
#include "builder/builder_base.h"
#include "builder/forward_declarations.h"
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

// the actual function implementation is in sample9.cpp
builder::dyn_var<void (builder::eigen_Xmat_t &)> print_matrix = builder::as_global("print_matrix");
builder::dyn_var<void (char *)> print_string = builder::as_global("print_string");

template<typename Scalar>
void toEigen(dyn_var<builder::eigen_Xmat_t> &mat, Xform<Scalar> &xform) {
  static_var<int> r,c;

  for (c = 0; c < 6; c = c + 1) {
    for (r = 0; r < 6; r = r + 1) {
      mat.coeffRef(c, r) = Xform_expr_leaf<Scalar>(xform).get_value_at(c, r);
    }
  }
}


template<typename Scalar>
void print_Xmat(Xform<Scalar> &xform) {
  dyn_var<builder::eigen_Xmat_t> tmp;
  toEigen(tmp, xform);
  print_matrix(tmp);
}

template<typename Scalar>
void print_Xmat(std::string prefix, Xform<Scalar> &xform) {
  dyn_var<builder::eigen_Xmat_t> tmp;
  toEigen(tmp, xform);

  print_string(prefix.c_str());
  print_matrix(tmp);
}

}

#endif
