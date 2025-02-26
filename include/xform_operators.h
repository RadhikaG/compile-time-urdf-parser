#ifndef XFORM_OPERATORS_H
#define XFORM_OPERATORS_H

#include "xform_impl.h"
#include <type_traits>

namespace ctup {

/*** Xform expressions **/

// type trait for checking if expr types are compatible with each other
template <typename Scalar>
struct is_acceptable_rhs_xform {
  static const bool value = false;
};

template <typename Scalar>
struct is_acceptable_rhs_xform<Xform<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Xform_expr<Scalar> &cast(const Xform<Scalar> &v) {
    return *new Xform_expr_leaf<Scalar>(v);
  }
};
template <typename Scalar>
struct is_acceptable_rhs_xform<Xform_expr<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Xform_expr<Scalar> &cast(const Xform_expr<Scalar> &v) {
    return v;
  }
};

/*** Translation expressions **/

template <typename Scalar>
struct is_acceptable_rhs_translation {
  static const bool value = false;
};

template <typename Scalar>
struct is_acceptable_rhs_translation<Translation<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Translation_expr<Scalar> &cast(const Translation<Scalar> &v) {
    return *new Translation_expr_leaf<Scalar>(v);
  }
};
template <typename Scalar>
struct is_acceptable_rhs_translation<Translation_expr<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Translation_expr<Scalar> &cast(const Translation_expr<Scalar> &v) {
    return v;
  }
};

template <typename T, typename = void>
struct is_acceptable_rhs_matrix {
  static const bool value = false;
};

template <template <typename> class T, typename Scalar>
struct is_acceptable_rhs_matrix<T<Scalar>> {
  static const bool value = std::is_base_of<Matrix<Scalar>, T<Scalar>>::value ||
                            std::is_base_of<Matrix_expr<Scalar>, T<Scalar>>::value;

  using scalar_type = Scalar;

  static const Matrix_expr<Scalar> &cast(const Matrix<Scalar> &v) {
    return *new Matrix_expr_leaf<Scalar>(v);
  }
  static const Matrix_expr<Scalar> &cast(const Matrix_expr<Scalar> &v) {
    return v;
  }
};

// todo: dealing with translation separately because translation needs to be migrated to
// storage, but translation expr is a matrix expr.
template <typename Scalar>
struct is_acceptable_rhs_matrix<Translation<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Matrix_expr<Scalar> &cast(const Translation<Scalar> &v) {
    return *new Translation_expr_leaf<Scalar>(v);
  }
};

} // namespace ctup

#endif
