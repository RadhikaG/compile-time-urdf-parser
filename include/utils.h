#ifndef UTILS_H
#define UTILS_H
#include "backend.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "builder/static_var.h"
#include "xform_impl.h"
#include <cassert>
// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

using builder::dyn_var;
using builder::static_var;

namespace ctup {

// the actual function implementation is in sample9.cpp
builder::dyn_var<void(EigenMatrix<double> &)> print_matrix = builder::as_global("print_matrix");
builder::dyn_var<void(char *)> print_string = builder::as_global("print_string");

template <typename Scalar>
void toPinEigen(dyn_var<EigenMatrix<Scalar>> &mat, Xform<Scalar> &xform) {
  // pinocchio outputs transpose of actual matrix
  static_var<int> r, c;

  for (c = 0; c < 6; c = c + 1) {
    for (r = 0; r < 6; r = r + 1) {
      mat.coeffRef(c, r) = Xform_expr_leaf<Scalar>(xform).get_value_at(r, c);
    }
  }
}

template <typename Scalar>
void toEigen(dyn_var<EigenMatrix<Scalar>> &mat, Xform<Scalar> &xform) {
  // pinocchio outputs transpose of actual matrix
  static_var<int> r, c;

  for (c = 0; c < 6; c = c + 1) {
    for (r = 0; r < 6; r = r + 1) {
      mat.coeffRef(r, c) = Xform_expr_leaf<Scalar>(xform).get_value_at(r, c);
    }
  }
}

template <typename Scalar>
void print_Xmat(Xform<Scalar> &xform) {
  print_matrix(Xform_expr_leaf<Scalar>(xform).get_value());
}

template <typename Scalar>
void print_Xmat(std::string prefix, Xform<Scalar> &xform) {
  print_string(prefix.c_str());
  print_matrix(Xform_expr_leaf<Scalar>(xform).get_value());
}

template <typename Scalar>
void print_Xmat_pin_order(Xform<Scalar> &xform) {
  print_matrix(Xform_expr_leaf<Scalar>(xform).get_value().transpose());
}

template <typename Scalar>
void print_Xmat_pin_order(std::string prefix, Xform<Scalar> &xform) {
  print_string(prefix.c_str());
  print_matrix(Xform_expr_leaf<Scalar>(xform).get_value().transpose());
}

} // namespace ctup

#endif
