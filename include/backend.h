#ifndef BACKEND_H
#define BACKEND_H
#include "Eigen/Dense"
#include "Eigen/src/Core/util/Constants.h"
#include "blocks/block_visitor.h"
#include "blocks/c_code_generator.h"
#include "blocks/var.h"
#include "builder/builder_base.h"
#include "builder/builder_context.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "builder/static_var.h"
// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

using builder::dyn_var;
using builder::static_var;

namespace ctup {

//static const char eigen_matrix_t_name[] = "Eigen::Matrix";
//template <typename Scalar>
//using _EigenMatrix = builder::name<eigen_matrix_t_name, Scalar>;

template <typename Scalar, int Rows_=Eigen::Dynamic, int Cols_=Eigen::Dynamic>
struct EigenMatrix {
  static constexpr const char* type_name = "Eigen::Matrix";

  static auto get_template_arg_types() {
    std::vector<block::type::Ptr> types;

    auto d1 = std::make_shared<block::named_type>();
    auto d2 = std::make_shared<block::named_type>();
    if (Rows_ == Eigen::Dynamic)
      d1->type_name = "Eigen::Dynamic";
    else
      d1->type_name = std::to_string(Rows_);
    if (Cols_ == Eigen::Dynamic)
      d2->type_name = "Eigen::Dynamic";
    else
      d2->type_name = std::to_string(Cols_);

    types.push_back(dyn_var<Scalar>::create_block_type());
    types.push_back(d1);
    types.push_back(d2);

    return types;
  }
};

template <typename Scalar, int Rows_=-1, int Cols_=-1>
struct BlazeStaticMatrix {
  static constexpr const char* type_name = "blaze::StaticMatrix";

  static auto get_template_arg_types() {
    std::vector<block::type::Ptr> types;

    auto d1 = std::make_shared<block::named_type>();
    auto d2 = std::make_shared<block::named_type>();
    d1->type_name = std::to_string(Rows_);
    d2->type_name = std::to_string(Cols_);

    types.push_back(dyn_var<Scalar>::create_block_type());
    types.push_back(d1);
    types.push_back(d2);

    return types;
  }
};

template <typename Scalar, int Dim=-1>
struct BlazeStaticVector {
  static constexpr const char* type_name = "blaze::StaticVector";

  static auto get_template_arg_types() {
    std::vector<block::type::Ptr> types;

    auto d1 = std::make_shared<block::named_type>();
    d1->type_name = std::to_string(Dim);

    types.push_back(dyn_var<Scalar>::create_block_type());
    types.push_back(d1);

    return types;
  }
};

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

// same convention as Eigen::Matrix
// https://eigen.tuxfamily.org/dox-devel/classEigen_1_1Matrix.html
template <typename Scalar, int Rows_, int Cols_>
class dyn_var<ctup::EigenMatrix<Scalar, Rows_, Cols_>> : public dyn_var_impl<ctup::EigenMatrix<Scalar, Rows_, Cols_>> {
public:
  typedef dyn_var_impl<ctup::EigenMatrix<Scalar, Rows_, Cols_>> super;
  using super::super;
  using super::operator=;
  builder operator=(const dyn_var<ctup::EigenMatrix<Scalar, Rows_, Cols_>> &t) {
    return (*this) = (builder)t;
  }

  // convenience getter variables
  size_t n_rows;
  size_t n_cols;

  void set_matrix_fixed_size(size_t _n_rows, size_t _n_cols) {
    auto type = block::to<block::named_type>(this->block_var->var_type);
    auto d1 = block::to<block::named_type>(type->template_args[1]);
    auto d2 = block::to<block::named_type>(type->template_args[2]);
    d1->type_name = std::to_string(_n_rows);
    d2->type_name = std::to_string(_n_cols);

    // setting convenience getter variables
    n_rows = _n_rows;
    n_cols = _n_cols;

    //type->template_args.push_back(d1);
    //type->template_args.push_back(d2);
  }

  dyn_var() : dyn_var_impl<ctup::EigenMatrix<Scalar, Rows_, Cols_>>() {}

  dyn_var(size_t _n_rows, size_t _n_cols) : dyn_var_impl<ctup::EigenMatrix<Scalar>>() {
    assertm(Rows_ == Eigen::Dynamic && Cols_ == Eigen::Dynamic, "can't have fixed and dynamic size param at same time");
    set_matrix_fixed_size(_n_rows, _n_cols);
  }

  dyn_var(const dyn_var<ctup::EigenMatrix<Scalar, Rows_, Cols_>> &t)
      : dyn_var_impl<ctup::EigenMatrix<Scalar, Rows_, Cols_>>((builder)t) {
    this->block_var->var_type = block::clone(t.block_var->var_type);
  }

  // so indexing into matrix types returns a dyn_var<Scalar>
  dyn_var<Scalar> operator[](const builder &bt) {
    return (dyn_var<Scalar>)(cast)this->dyn_var_impl<ctup::EigenMatrix<Scalar, Rows_, Cols_>>::operator[](bt);
  }

  dyn_var<double &(Eigen::Index, Eigen::Index)> coeffRef = as_member(this, "coeffRef");
  dyn_var<double &(Eigen::Index, Eigen::Index)> block =
      as_member(this, "block<3,3>"); // remove template params later
  dyn_var<void(void)> setZero = as_member(this, "setZero");
  dyn_var<ctup::EigenMatrix<Scalar, Rows_, Cols_> &(double)> setConstant = as_member(this, "setConstant");
  //dyn_var<ctup::EigenMatrix<Scalar, Rows_, Cols_> &(void)> transpose = as_member(this, "transpose");
  dyn_var<ctup::EigenMatrix<Scalar> &(void)> transpose = as_member(this, "transpose");
  dyn_var<eigen_vectorXd_t &(Eigen::Index)> col = as_member(this, "col");
};

template <typename Scalar, int Rows_, int Cols_>
class dyn_var<ctup::BlazeStaticMatrix<Scalar, Rows_, Cols_>> : public dyn_var_impl<ctup::BlazeStaticMatrix<Scalar, Rows_, Cols_>> {
public:
  typedef dyn_var_impl<ctup::BlazeStaticMatrix<Scalar, Rows_, Cols_>> super;
  using super::super;
  using super::operator=;
  builder operator=(const dyn_var<ctup::BlazeStaticMatrix<Scalar, Rows_, Cols_>> &t) {
    return (*this) = (builder)t;
  }

  void set_matrix_fixed_size(int _n_rows, int _n_cols) {
    auto type = block::to<block::named_type>(this->block_var->var_type);
    auto d1 = block::to<block::named_type>(type->template_args[1]);
    auto d2 = block::to<block::named_type>(type->template_args[2]);
    d1->type_name = std::to_string(_n_rows);
    d2->type_name = std::to_string(_n_cols);
  }

  dyn_var() : dyn_var_impl<ctup::BlazeStaticMatrix<Scalar, Rows_, Cols_>>() {}

  dyn_var(size_t _n_rows, size_t _n_cols) : dyn_var_impl<ctup::BlazeStaticMatrix<Scalar, Rows_, Cols_>>() {
    set_matrix_fixed_size(_n_rows, _n_cols);
  }

  dyn_var(const dyn_var<ctup::BlazeStaticMatrix<Scalar, Rows_, Cols_>> &t)
      : dyn_var_impl<ctup::BlazeStaticMatrix<Scalar, Rows_, Cols_>>((builder)t) {
    this->block_var->var_type = block::clone(t.block_var->var_type);
  }

  // so indexing into matrix types returns a dyn_var<Scalar>
  dyn_var<Scalar> operator[](const builder &bt) {
    return (dyn_var<Scalar>)(cast)this->dyn_var_impl<ctup::BlazeStaticMatrix<Scalar, Rows_, Cols_>>::operator[](bt);
  }
};

template <typename Scalar, int Dim>
class dyn_var<ctup::BlazeStaticVector<Scalar, Dim>> : public dyn_var_impl<ctup::BlazeStaticVector<Scalar, Dim>> {
public:
  typedef dyn_var_impl<ctup::BlazeStaticVector<Scalar, Dim>> super;
  using super::super;
  using super::operator=;
  builder operator=(const dyn_var<ctup::BlazeStaticVector<Scalar, Dim>> &t) {
    return (*this) = (builder)t;
  }

  void set_vector_fixed_size(int dim) {
    auto type = block::to<block::named_type>(this->block_var->var_type);
    auto d1 = block::to<block::named_type>(type->template_args[1]);
    d1->type_name = std::to_string(dim);
  }

  dyn_var() : dyn_var_impl<ctup::BlazeStaticVector<Scalar, Dim>>() {}

  dyn_var(size_t dim) : dyn_var_impl<ctup::BlazeStaticVector<Scalar, Dim>>() {
    set_vector_fixed_size(dim);
  }

  dyn_var(const dyn_var<ctup::BlazeStaticVector<Scalar, Dim>> &t)
      : dyn_var_impl<ctup::BlazeStaticVector<Scalar, Dim>>((builder)t) {
    this->block_var->var_type = block::clone(t.block_var->var_type);
  }

  // so indexing into Vector types returns a dyn_var<Scalar>
  dyn_var<Scalar> operator[](const builder &bt) {
    return (dyn_var<Scalar>)(cast)this->dyn_var_impl<ctup::BlazeStaticVector<Scalar, Dim>>::operator[](bt);
  }
};

} // namespace builder

namespace ctup {

template <typename T>
struct vector_t: public builder::custom_type<T> {
  static constexpr const char* type_name = "std::vector";
  typedef T dereference_type;
  dyn_var<void(int)> size = builder::with_name("size");
  dyn_var<int(void)> resize = builder::with_name("resize");
  dyn_var<T(void)> push_back = builder::with_name("push_back");
};

namespace backend {

typedef ctup::BlazeStaticVector<double, 8> blazeVecSIMDd;
typedef ctup::BlazeStaticVector<float, 16> blazeVecSIMDf;

//builder::dyn_var<double(double)> sin = builder::as_global("sin");
//builder::dyn_var<double(double)> cos = builder::as_global("cos");

template <typename Prim>
builder::dyn_var<Prim(Prim)> sin = builder::as_global("sin");
template <typename Prim>
builder::dyn_var<Prim(Prim)> cos = builder::as_global("cos");

builder::dyn_var<int(ctup::BlazeStaticVector<double,16>, ctup::BlazeStaticVector<double,16>, ctup::BlazeStaticVector<double,16>,
vector_t<ctup::BlazeStaticVector<double,16>>,vector_t<ctup::BlazeStaticVector<double,16>>,vector_t<ctup::BlazeStaticVector<double,16>>,vector_t<ctup::BlazeStaticVector<double,16>>,vector_t<ctup::BlazeStaticVector<double,16>>)> Sphere_Environment_Collision = builder::as_global("Sphere_Environment_Collision");

builder::dyn_var<int(dyn_var<ctup::BlazeStaticVector<double,16>>, dyn_var<ctup::BlazeStaticVector<double,16>>, dyn_var<ctup::BlazeStaticVector<double,16>>, dyn_var<double>,
 dyn_var<vector_t<ctup::BlazeStaticVector<double,16>>>, dyn_var<vector_t<ctup::BlazeStaticVector<double,16>>>, dyn_var<vector_t<ctup::BlazeStaticVector<double,16>>>, dyn_var<vector_t<double>>,
 dyn_var<ctup::BlazeStaticVector<double,16>>, dyn_var<ctup::BlazeStaticVector<double,16>>, dyn_var<ctup::BlazeStaticVector<double,16>>, dyn_var<double>,
 dyn_var<vector_t<ctup::BlazeStaticVector<double,16>>>, dyn_var<vector_t<ctup::BlazeStaticVector<double,16>>>, dyn_var<vector_t<ctup::BlazeStaticVector<double,16>>>, dyn_var<vector_t<double>>)> self_collision = builder::as_global("self_collision");

template<typename Scalar, int Dim>
builder::dyn_var<ctup::BlazeStaticVector<Scalar, Dim>(ctup::BlazeStaticVector<Scalar, Dim> &, double)> min = builder::as_global("min");

template<typename Scalar, int Dim>
builder::dyn_var<ctup::BlazeStaticVector<Scalar, Dim>(ctup::BlazeStaticVector<Scalar, Dim> &, double)> max = builder::as_global("max");

template<typename Scalar, int Dim>
builder::dyn_var<ctup::BlazeStaticVector<Scalar, Dim>(void)> abs = builder::as_global("abs");
} // namespace backend

} // namespace ctup

#endif
