#ifndef MATRIX_OPERATORS_H
#define MATRIX_OPERATORS_H

#include "matrix_layout_composite.h"
#include "matrix_layout.h"
#include <type_traits>

namespace ctup {

/** Individual matrix layout types **/

template <typename T, typename = void>
struct is_acceptable_rhs_matrix_layout {
  static const bool value = false;
};

template <template <typename> class T, typename Scalar>
struct is_acceptable_rhs_matrix_layout<T<Scalar>> {
  static const bool value = std::is_base_of<matrix_layout<Scalar>, T<Scalar>>::value |
      std::is_base_of<matrix_layout_expr<Scalar>, T<Scalar>>::value;

  using scalar_type = Scalar;

  static const matrix_layout_expr<Scalar> &cast(const matrix_layout<Scalar> &v) {
    return *new matrix_layout_expr_leaf<Scalar>(v);
  }

  static const matrix_layout_expr<Scalar> &cast(const matrix_layout_expr<Scalar> &v) {
    return v;
  }
};

template <typename E1, typename E2>
// sets return type to matrix_layout_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_matrix_layout<E2>::value,
                        const matrix_layout_expr<typename is_acceptable_rhs_matrix_layout<E1>::scalar_type> &>::type
// ...for matrix * matrix
operator*(const E1 &v1, const E2 &v2) {
  return *new matrix_layout_expr_mul<typename is_acceptable_rhs_matrix_layout<E1>::scalar_type>(
      is_acceptable_rhs_matrix_layout<E1>::cast(v1), is_acceptable_rhs_matrix_layout<E2>::cast(v2));
}

template <typename E1, typename E2>
// sets return type to matrix_layout_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_matrix_layout<E2>::value,
                        const matrix_layout_expr<typename is_acceptable_rhs_matrix_layout<E1>::scalar_type> &>::type
// ...for matrix - matrix
operator+(const E1 &v1, const E2 &v2) {
  return *new matrix_layout_expr_add<typename is_acceptable_rhs_matrix_layout<E1>::scalar_type>(
      is_acceptable_rhs_matrix_layout<E1>::cast(v1), is_acceptable_rhs_matrix_layout<E2>::cast(v2));
}

template <typename E1>
// sets return type to matrix_layout_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_matrix_layout<E1>::value,
                        const matrix_layout_expr<typename is_acceptable_rhs_matrix_layout<E1>::scalar_type> &>::type
// ...for -matrix
operator-(const E1 &v1) {
  return *new matrix_layout_expr_unary_minus<typename is_acceptable_rhs_matrix_layout<E1>::scalar_type>(
      is_acceptable_rhs_matrix_layout<E1>::cast(v1));
}

template <typename E1, typename E2>
// sets return type to matrix_layout_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_matrix_layout<E1>::value && is_acceptable_rhs_matrix_layout<E2>::value, void>::type
// ...for += expr
operator+=(E1 &v1, const E2 &v2) {
  v1 = v1 + v2;
}

/** Blocked matrix layouts with individual matrix blocks sitting inside them **/

template <typename T, typename = void>
struct is_acceptable_rhs_blocked_layout {
  static const bool value = false;
};

template <template <typename> class T, typename Scalar>
struct is_acceptable_rhs_blocked_layout<T<Scalar>> {
  static const bool value = std::is_base_of<blocked_layout<Scalar>, T<Scalar>>::value |
      std::is_base_of<blocked_layout_expr<Scalar>, T<Scalar>>::value;

  using scalar_type = Scalar;

  static const blocked_layout_expr<Scalar> &cast(const blocked_layout<Scalar> &v) {
    return *new blocked_layout_expr_leaf<Scalar>(v);
  }

  static const blocked_layout_expr<Scalar> &cast(const blocked_layout_expr<Scalar> &v) {
    return v;
  }
};

template <typename E1, typename E2>
// sets return type to blocked_layout_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_blocked_layout<E2>::value,
                        const blocked_layout_expr<typename is_acceptable_rhs_blocked_layout<E1>::scalar_type> &>::type
// ...for matrix * matrix
operator*(const E1 &v1, const E2 &v2) {
  return *new blocked_layout_expr_mul<typename is_acceptable_rhs_blocked_layout<E1>::scalar_type>(
      is_acceptable_rhs_blocked_layout<E1>::cast(v1), is_acceptable_rhs_blocked_layout<E2>::cast(v2));
}

}

#endif
