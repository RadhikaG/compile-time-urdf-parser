#ifndef MATRIX_OPERATORS_H
#define MATRIX_OPERATORS_H

#include "matrix_layout_composite.h"
#include "matrix_layout.h"
#include <type_traits>

namespace ctup {

/** Individual matrix layout types **/

template <typename Scalar>
struct is_acceptable_rhs_scalar {
  static const bool value = std::is_arithmetic<Scalar>::value;
  using scalar_type = Scalar;

  static const matrix_layout_expr<Scalar> &cast(const Scalar &v, size_t broadcast_rows, size_t broadcast_cols) {
    return *new matrix_layout_expr_scalar<Scalar>(v, broadcast_rows, broadcast_cols);
  }
};

template <typename Scalar>
struct is_acceptable_rhs_scalar<builder::dyn_var<Scalar>> {
  static const bool value = std::is_arithmetic<Scalar>::value;
  using scalar_type = Scalar;

  static const matrix_layout_expr<Scalar> &cast(const builder::dyn_var<Scalar> &v, size_t broadcast_rows, size_t broadcast_cols) {
    return *new matrix_layout_expr_scalar<Scalar>(v, broadcast_rows, broadcast_cols);
  }
};

template <typename T, typename = void>
struct is_acceptable_rhs_storable_or_expr {
  static const bool value = false;
};

// checks if T is some kind of matrix_layout or some kind of expr
template <template <typename> class T, typename Prim>
struct is_acceptable_rhs_storable_or_expr<T<Prim>> {
  static const bool value = std::is_base_of<zero_cst_status_storable<Prim>, T<Prim>>::value |
      std::is_base_of<matrix_layout_expr<Prim>, T<Prim>>::value;

  using prim_type = Prim;
  using scalar_type = inner_type_t<Prim>;

  static const matrix_layout_expr<Prim> &cast(const zero_cst_status_storable<Prim> &v) {
    return *new matrix_layout_expr_leaf<Prim>(v);
  }

  static const matrix_layout_expr<Prim> &cast(const matrix_layout_expr<Prim> &v) {
    return v;
  }
};

template <typename T, typename = void>
struct is_acceptable_rhs_blocked_layout {
  static const bool value = false;
};

template <template <typename> class T, typename Prim>
struct is_acceptable_rhs_blocked_layout<T<Prim>> {
  static const bool value = std::is_base_of<blocked_layout<Prim>, T<Prim>>::value |
      std::is_base_of<blocked_layout_expr<Prim>, T<Prim>>::value;

  using prim_type = Prim;
  using scalar_type = inner_type_t<Prim>;

  static const blocked_layout_expr<Prim> &cast(const blocked_layout<Prim> &v) {
    return *new blocked_layout_expr_leaf<Prim>(v);
  }

  static const blocked_layout_expr<Prim> &cast(const blocked_layout_expr<Prim> &v) {
    return v;
  }
};

//**** expr property helpers ****//

template <typename E1, typename E2>
inline constexpr bool are_both_blocked = is_acceptable_rhs_blocked_layout<E1>::value && is_acceptable_rhs_blocked_layout<E2>::value;

template <typename E1, typename E2>
inline constexpr bool are_both_storable = is_acceptable_rhs_storable_or_expr<E1>::value && is_acceptable_rhs_storable_or_expr<E2>::value;

template <typename E1, typename E2>
inline constexpr bool is_neither_storable = !is_acceptable_rhs_storable_or_expr<E1>::value && !is_acceptable_rhs_storable_or_expr<E2>::value;

template <typename E>
using prim_type_t = typename is_acceptable_rhs_storable_or_expr<E>::prim_type;

template <typename E>
using scalar_type_t = typename is_acceptable_rhs_storable_or_expr<E>::scalar_type;

template <typename E1, typename E2>
inline constexpr bool are_both_nonbatched = !is_Matrix_v<prim_type_t<E1>> && !is_Matrix_v<prim_type_t<E2>>;


template <typename E1, typename E2,
         typename = typename std::enable_if<
         is_acceptable_rhs_storable_or_expr<E1>::value || is_acceptable_rhs_storable_or_expr<E2>::value>::type
>
struct select_ret_prim {
  using E1_prim = prim_type_t<E1>;
  using E2_prim = prim_type_t<E2>;
  static const bool is_E1_prim_batched = is_Matrix_v<E1_prim>;
  static const bool is_E2_prim_batched = is_Matrix_v<E2_prim>;
  static const bool is_E1_storable = is_acceptable_rhs_storable_or_expr<E1>::value;
  static const bool is_E2_storable = is_acceptable_rhs_storable_or_expr<E2>::value;
  static const bool is_E1_arithmetic = is_acceptable_rhs_scalar<E1>::value;
  static const bool is_E2_arithmetic = is_acceptable_rhs_scalar<E2>::value;

  using ret_prim = 
        // if either E1 or E2 is batched, return batched expr
        std::conditional_t<is_E1_prim_batched, E1_prim,
        std::conditional_t<is_E2_prim_batched, E2_prim,
          // if either E1 or E2 is a storable matrix_layout, return storable expr
          // note that all batched exprs are storable
          std::conditional_t<is_E1_storable, E1_prim,
          std::conditional_t<is_E2_storable, E2_prim,
            // if neither E1 nor E2 is batched or storable, both must be non storable scalars
            // i.e. basic arithmetic types
            std::common_type<E1_prim, E2_prim>>>>>;
};

template <typename E1, typename E2>
using batching_common_type = typename select_ret_prim<E1, E2>::ret_prim;

//**** Operator overloads ****//

// ----------------------------------------
// Notes:
//
// In matrix_layout.h, we statically dispatch the following expr types:
// - scalar matrix ops: matrix_layout<scalar> = ...
// S0.  matrix_layout<scalar>
// - SPMD batched matrix ops: matrix_layout<batched> = ...
// B0.  matrix_layout<batched>
//
// In this file, we statically dispatch the following expr types:
// - scalar matrix ops: matrix_layout<scalar> = ...
// S1a. matrix_layout<scalar> * scalar
// S1b. scalar * matrix_layout<scalar>
// S2a. matrix_layout<scalar> * dyn_var<scalar>
// S2b. dyn_var<scalar> * matrix_layout<scalar>
// S3.  matrix_layout<scalar> * matrix_layout<scalar>
// - SPMD batched matrix ops: matrix_layout<batched> = ...
// B1a. matrix_layout<batched> * scalar
// B1b. scalar * matrix_layout<batched>
// B2a. matrix_layout<batched> * dyn_var<scalar>
// B2b. dyn_var<scalar> * matrix_layout<batched>
// B3a. matrix_layout<scalar> * matrix_layout<batched>
// B3b. matrix_layout<batched> * matrix_layout<scalar>
// B3c. matrix_layout<batched> * matrix_layout<batched>
// 
// We should use operators provided by the C++ standard and buildit for the following
// ops (so template substitution in this file should fail for these types:
// F0. scalar * scalar
// F1. dyn_var<scalar> * dyn_var<scalar>
// F2. scalar * dyn_var<scalar>
// F3. dyn_var<scalar> * scalar
// ----------------------------------------

// This template handles the following cases:
// S3.  matrix_layout<scalar> * matrix_layout<scalar>
// B3a. matrix_layout<scalar> * matrix_layout<batched>
// B3b. matrix_layout<batched> * matrix_layout<scalar>
// B3c. matrix_layout<batched> * matrix_layout<batched>
template <typename E1, typename E2>
// sets return type to matrix_layout_expr<batching_common_type>...
typename std::enable_if<are_both_storable<E1, E2> && !are_both_blocked<E1, E2>,
                        const matrix_layout_expr<batching_common_type<E1, E2>> &>::type
// ...for matrix * matrix (but not blocked * blocked, we specialize for that)
operator*(const E1 &v1, const E2 &v2) {
  return *new matrix_layout_expr_mul<batching_common_type<E1, E2>, prim_type_t<E1>, prim_type_t<E2>>(
      is_acceptable_rhs_storable_or_expr<E1>::cast(v1), is_acceptable_rhs_storable_or_expr<E2>::cast(v2));
}

// This template handles the following cases:
// S1a.  matrix_layout<scalar> * scalar
// S1b.  scalar * matrix_layout<scalar>
// B1a. matrix_layout<batched> * scalar
// B1b. scalar * matrix_layout<batched>
// S2a.  matrix_layout<scalar> * dyn_var<scalar>
// S2b.  dyn_var<scalar> * matrix_layout<scalar>
// B2a. matrix_layout<batched> * dyn_var<scalar>
// B2b. dyn_var<scalar> * matrix_layout<batched>
template <typename E1, typename E2>
// sets return type to matrix_layout_expr<Prim>...
typename std::enable_if<!are_both_storable<E1, E2> && !is_neither_storable<E1, E2>,
                        const matrix_layout_expr<batching_common_type<E1, E2>> &>::type
// ...for matrix * scalar or scalar * matrix (any matrix, even blocked)
operator*(const E1 &v1, const E2 &v2) {
  if constexpr (is_acceptable_rhs_scalar<E2>::value) {
    // matrix * scalar
    const matrix_layout_expr<typename is_acceptable_rhs_storable_or_expr<E1>::scalar_type> & v1_expr = is_acceptable_rhs_storable_or_expr<E1>::cast(v1);

    return *new matrix_layout_expr_cwise_mul<batching_common_type<E1, E2>, prim_type_t<E1>, prim_type_t<E2>>(
        v1_expr, is_acceptable_rhs_scalar<E2>::cast(v2, 
            // broadcast the scalar operand to same shape as v1
            v1_expr.get_expr_shape()[0], v1_expr.get_expr_shape()[1]));
  }
  else {
    // scalar * matrix
    const matrix_layout_expr<typename is_acceptable_rhs_storable_or_expr<E2>::scalar_type> & v2_expr = is_acceptable_rhs_storable_or_expr<E2>::cast(v2);

    return *new matrix_layout_expr_cwise_mul<batching_common_type<E1, E2>, prim_type_t<E2>, prim_type_t<E1>>(
        v2_expr, is_acceptable_rhs_scalar<E2>::cast(v1, 
            // broadcast the scalar operand within matrix_layout
            v2_expr.get_expr_shape()[0], v2_expr.get_expr_shape()[1]));
  }
}

template <typename E1, typename E2>
// sets return type to matrix_layout_expr<Prim>...
typename std::enable_if<are_both_storable<E1, E2> && !are_both_blocked<E1, E2>,
                        const matrix_layout_expr<batching_common_type<E1, E2>> &>::type
// ...for matrix + matrix //(but not blocked + blocked, we specialize for that) todo
operator+(const E1 &v1, const E2 &v2) {
  return *new matrix_layout_expr_add<batching_common_type<E1, E2>, prim_type_t<E1>, prim_type_t<E2>>(
      is_acceptable_rhs_storable_or_expr<E1>::cast(v1), is_acceptable_rhs_storable_or_expr<E2>::cast(v2));
}

template <typename E1>
// sets return type to matrix_layout_expr<Prim>...
typename std::enable_if<is_acceptable_rhs_storable_or_expr<E1>::value,
                        const matrix_layout_expr<prim_type_t<E1>> &>::type
// ...for -matrix
operator-(const E1 &v1) {
  return *new matrix_layout_expr_unary_minus<prim_type_t<E1>>(
      is_acceptable_rhs_storable_or_expr<E1>::cast(v1));
}

template <typename E1, typename E2>
// sets return type to void...
typename std::enable_if<are_both_storable<E1, E2>, void>::type
// ...for += expr
operator+=(E1 &v1, const E2 &v2) {
  v1 = v1 + v2;
}

template <typename E1>
// sets return type to matrix_layout_expr<Prim>...
typename std::enable_if<is_acceptable_rhs_storable_or_expr<E1>::value,
                        const matrix_layout_expr<prim_type_t<E1>> &>::type
// ...for transpose(matrix)
transpose(const E1 &v1) {
  return *new matrix_layout_expr_transpose<typename is_acceptable_rhs_storable_or_expr<E1>::scalar_type>(
      is_acceptable_rhs_storable_or_expr<E1>::cast(v1));
}

/** Blocked matrix layouts with individual matrix blocks sitting inside them **/

template <typename E1, typename E2>
// sets return type to blocked_layout_expr<Scalar>...
typename std::enable_if<are_both_blocked<E1, E2>,
                        const blocked_layout_expr<batching_common_type<E1, E2>> &>::type
// ...for blocked_matrix * blocked_matrix
operator*(const E1 &v1, const E2 &v2) {
  return *new blocked_layout_expr_mul<batching_common_type<E1, E2>, prim_type_t<E1>, prim_type_t<E2>>(
      is_acceptable_rhs_blocked_layout<E1>::cast(v1), is_acceptable_rhs_blocked_layout<E2>::cast(v2));
}

}

#endif
