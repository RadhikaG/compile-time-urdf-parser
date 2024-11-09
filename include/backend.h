#ifndef BACKEND_H
#define BACKEND_H
#include "blocks/block_visitor.h"
#include "blocks/c_code_generator.h"
#include "builder/builder_base.h"
#include "builder/builder_context.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "builder/static_var.h"
#include "builder/lib/utils.h"
#include "Eigen/Dense"

namespace ctup {

namespace backend {
  builder::dyn_var<double (double)> sin = builder::as_global("sin");
  builder::dyn_var<double (double)> cos = builder::as_global("cos");
}
}

namespace builder {

static const char eigen_Xmat_t_name[] = "Eigen::Matrix<double, 6, 6>";
static const char eigen_vec_t_name[] = "Eigen::VectorXd";

using eigen_Xmat_t = name<eigen_Xmat_t_name>;
using eigen_vectorXd_t = name<eigen_vec_t_name>;

template <>
class dyn_var<eigen_vectorXd_t> : public dyn_var_impl<eigen_vectorXd_t> {
public:
  typedef dyn_var_impl<eigen_vectorXd_t> super;
  using super::super;
  using super::operator=;
  builder operator=(const dyn_var<eigen_vectorXd_t> &t) {
    return (*this) = (builder)t;
  }
  dyn_var(const dyn_var &t) : dyn_var_impl((builder)t) {}
  dyn_var() : dyn_var_impl<eigen_vectorXd_t>() {} 

  // so indexing into vector types returns a dyn_var<double>
  dyn_var<double> operator[](const builder &bt) {
    return (dyn_var<double>)(cast)this->dyn_var_impl<eigen_vectorXd_t>::operator[](bt);
  }
};

template <>
class dyn_var<eigen_Xmat_t> : public dyn_var_impl<eigen_Xmat_t> {
public:
  typedef dyn_var_impl<eigen_Xmat_t> super;
  using super::super;
  using super::operator=;
  builder operator=(const dyn_var<eigen_Xmat_t> &t) {
    return (*this) = (builder)t;
  }
  dyn_var(const dyn_var &t) : dyn_var_impl((builder)t) {}
  dyn_var() : dyn_var_impl<eigen_Xmat_t>() {} 

  //dyn_var<double& (Eigen::MatrixXd::*)(Eigen::Index, Eigen::Index)> coeffRef = as_member(this, "coeffRef");
  dyn_var<double& (Eigen::Index, Eigen::Index)> coeffRef = as_member(this, "coeffRef");
  dyn_var<double& (Eigen::Index, Eigen::Index)> block = as_member(this, "block<3,3>"); // remove template params later
  dyn_var<void (void)> setZero = as_member(this, "setZero");
};

template <>
class dyn_var<eigen_Xmat_t[]> : public dyn_var_impl<eigen_Xmat_t[]> {
public:
  typedef dyn_var_impl<eigen_Xmat_t[]> super;
  using super::super;
  using super::operator=;
  builder operator=(const dyn_var<eigen_Xmat_t> &t) {
    return (*this) = (builder)t;
  }
  dyn_var(const dyn_var &t) : dyn_var_impl((builder)t) {}
  dyn_var() : dyn_var_impl<eigen_Xmat_t[]>() {} 

  // for indexing into MatrixXd[] arrays and using MatrixXd class methods
  dyn_var_mimic<eigen_Xmat_t> operator[](const builder &bt) {
    return (dyn_var_mimic<eigen_Xmat_t>)(cast)this->dyn_var_impl<eigen_Xmat_t[]>::operator[](bt);
  }
};
}

#endif
