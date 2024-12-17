#ifndef BACKEND_H
#define BACKEND_H
#include "Eigen/Dense"
#include "blocks/block_visitor.h"
#include "blocks/c_code_generator.h"
#include "blocks/var.h"
#include "builder/builder_base.h"
#include "builder/builder_context.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "builder/static_var.h"

using builder::dyn_var;
using builder::static_var;

namespace ctup {

namespace backend {
builder::dyn_var<double(double)> sin = builder::as_global("sin");
builder::dyn_var<double(double)> cos = builder::as_global("cos");
} // namespace backend

static const char eigen_matrix_t_name[] = "Eigen::Matrix";
template <typename Scalar>
using EigenMatrix = builder::name<eigen_matrix_t_name, Scalar>;
} // namespace ctup

namespace builder {

static const char eigen_vec_t_name[] = "Eigen::VectorXd";
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

  dyn_var<eigen_vectorXd_t(Eigen::Index, Eigen::Index)> setZero = as_member(this, "setZero");
  dyn_var<eigen_vectorXd_t(Eigen::Index, Eigen::Index)> setOnes = as_member(this, "setOnes");
  dyn_var<eigen_vectorXd_t(void)> cos = as_member(this, "cos");
  dyn_var<eigen_vectorXd_t(void)> array = as_member(this, "array");
  dyn_var<eigen_vectorXd_t(double)> setConstant = as_member(this, "setConstant");
  dyn_var<eigen_vectorXd_t(void)> sin = as_member(this, "sin");
};

template <typename Scalar>
class dyn_var<ctup::EigenMatrix<Scalar>> : public dyn_var_impl<ctup::EigenMatrix<Scalar>> {
public:
  typedef dyn_var_impl<ctup::EigenMatrix<Scalar>> super;
  using super::super;
  using super::operator=;
  builder operator=(const dyn_var<ctup::EigenMatrix<Scalar>> &t) {
    return (*this) = (builder)t;
  }

  dyn_var() : dyn_var_impl<ctup::EigenMatrix<Scalar>>() {}

  dyn_var(size_t _n_rows, size_t _n_cols) : dyn_var_impl<ctup::EigenMatrix<Scalar>>() {
    auto type = block::to<block::named_type>(this->block_var->var_type);
    auto d1 = std::make_shared<block::named_type>();
    auto d2 = std::make_shared<block::named_type>();
    if (_n_rows == 0)
      d1->type_name = "Eigen::Dynamic";
    else
      d1->type_name = std::to_string(_n_rows);
    if (_n_cols == 0)
      d2->type_name = "Eigen::Dynamic";
    else
      d2->type_name = std::to_string(_n_cols);

    type->template_args.push_back(d1);
    type->template_args.push_back(d2);
  }

  dyn_var(const dyn_var<ctup::EigenMatrix<Scalar>> &t)
      : dyn_var_impl<ctup::EigenMatrix<Scalar>>((builder)t) {
    this->block_var->var_type = block::clone(t.block_var->var_type);
  }

  // so indexing into matrix types returns a dyn_var<Scalar>
  dyn_var<Scalar> operator[](const builder &bt) {
    return (dyn_var<Scalar>)(cast)this->dyn_var_impl<ctup::EigenMatrix<Scalar>>::operator[](bt);
  }

  dyn_var<double &(Eigen::Index, Eigen::Index)> coeffRef = as_member(this, "coeffRef");
  dyn_var<double &(Eigen::Index, Eigen::Index)> block =
      as_member(this, "block<3,3>"); // remove template params later
  dyn_var<void(void)> setZero = as_member(this, "setZero");
  dyn_var<ctup::EigenMatrix<Scalar> &(double)> setConstant = as_member(this, "setConstant");
  dyn_var<ctup::EigenMatrix<Scalar> &(void)> transpose = as_member(this, "transpose");
  dyn_var<eigen_vectorXd_t &(Eigen::Index)> col = as_member(this, "col");
};

} // namespace builder

#endif
