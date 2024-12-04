#ifndef BACKEND_H
#define BACKEND_H
#include "blocks/block_visitor.h"
#include "blocks/c_code_generator.h"
#include "blocks/var.h"
#include "builder/builder_base.h"
#include "builder/builder_context.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "builder/static_var.h"
#include "Eigen/Dense"

using builder::dyn_var;
using builder::static_var;

namespace ctup {

namespace backend {
  builder::dyn_var<double (double)> sin = builder::as_global("sin");
  builder::dyn_var<double (double)> cos = builder::as_global("cos");
}

template <typename Scalar>
struct EigenMatrix : builder::custom_type<Scalar> {
  static constexpr const char* type_name = "Eigen::Matrix";

  dyn_var<double& (Eigen::Index, Eigen::Index)> coeffRef = builder::as_member("coeffRef");
  dyn_var<double& (Eigen::Index, Eigen::Index)> block = builder::as_member("block<3,3>"); // remove template params later
  dyn_var<void (void)> setZero = builder::as_member("setZero");
  dyn_var<EigenMatrix<Scalar> (void)> transpose = builder::as_member("transpose");
};

template <typename Scalar>
void setEigenMatrixTemplateDims(dyn_var<EigenMatrix<Scalar>> &mat, size_t num_rows, size_t num_cols) {
  auto type = block::to<block::named_type>(mat.block_var->var_type);
  auto d1 = std::make_shared<block::named_type>();
  auto d2 = std::make_shared<block::named_type>();
  if (num_rows == 0)
    d1->type_name = "Eigen::Dynamic";
  else
    d1->type_name = std::to_string(num_rows);
  if (num_cols == 0)
    d2->type_name = "Eigen::Dynamic";
  else
    d2->type_name = std::to_string(num_cols);

  type->template_args.push_back(d1);
  type->template_args.push_back(d2);
}

}

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
};

//template <typename Scalar, size_t Rows, size_t Cols>
//class dyn_var<ctup::EigenMatrix<Scalar, Rows, Cols>> : public dyn_var_impl<ctup::EigenMatrix<Scalar, Rows, Cols>> {
//public:
//  using EigenMatrix = ctup::EigenMatrix<Scalar, Rows, Cols>;
//
//  typedef dyn_var_impl<EigenMatrix> super;
//  using super::super;
//  using super::operator=;
//  builder operator=(const dyn_var<EigenMatrix> &t) {
//    return (*this) = (builder)t;
//  }
//  dyn_var(const dyn_var &t) : dyn_var_impl((builder)t) {}
//  dyn_var() : dyn_var_impl<EigenMatrix>() {} 
//
//};
//
//template <typename Scalar, size_t Rows, size_t Cols>
//class dyn_var<ctup::EigenMatrix<Scalar, Rows, Cols>[]> : public dyn_var_impl<ctup::EigenMatrix<Scalar, Rows, Cols>[]> {
//public:
//  using EigenMatrix = ctup::EigenMatrix<Scalar, Rows, Cols>;
//
//  typedef dyn_var_impl<EigenMatrix[]> super;
//  using super::super;
//  using super::operator=;
//  builder operator=(const dyn_var<EigenMatrix> &t) {
//    return (*this) = (builder)t;
//  }
//  dyn_var(const dyn_var &t) : dyn_var_impl((builder)t) {}
//  dyn_var() : dyn_var_impl<EigenMatrix[]>() {} 
//
//  // for indexing into MatrixXd[] arrays and using MatrixXd class methods
//  dyn_var_mimic<EigenMatrix> operator[](const builder &bt) {
//    return (dyn_var_mimic<EigenMatrix>)(cast)this->dyn_var_impl<EigenMatrix[]>::operator[](bt);
//  }
//};
}

#endif
