#ifndef CMATRIX_OPERATOR_H
#define CMATRIX_OPERATOR_H

#include "spatial_algebra.h"
#include "Eigen/Dense"
#include <type_traits>

namespace SpatialAlgebra {


// A simple type trait to identify if 
// operators should be overloaded
template <typename T>
struct is_acceptable_rhs_matrix {
  static const bool value = false;
};

template <typename T>
struct is_acceptable_rhs_matrix<cmatrix<T>> {
  static const bool value = true;
  using output_type = T;
  static const cmatrix_expr<T>& cast(const cmatrix<T>& v) {
    return *new cmatrix_expr_leaf<T>(v);
  }
};

template <typename T>
struct is_acceptable_rhs_matrix<cmatrix_expr<T>> {
  static const bool value = true;
  using output_type = T;
  static const cmatrix_expr<T>& cast(const cmatrix_expr<T>& v) {
    return v;
  }
};


// Bin op overloads: 
// Return type: cmatrix_expr
// T1: [cmatrix, cmatrix_expr, int|float|double]
// T2: [cmatrix, cmatrix_expr, int|float|double]

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs_matrix<T2>::value, const cmatrix_expr<typename is_acceptable_rhs_matrix<T1>::output_type>&>::type
operator + (const T1& v1, const T2& v2) {
  return *new cmatrix_expr_add<typename is_acceptable_rhs_matrix<T1>::output_type>(is_acceptable_rhs_matrix<T1>::cast(v1), is_acceptable_rhs_matrix<T2>::cast(v2));
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs_matrix<T2>::value, const cmatrix_expr<typename is_acceptable_rhs_matrix<T1>::output_type>&>::type
operator - (const T1& v1, const T2& v2) {
  return *new cmatrix_expr_sub<typename is_acceptable_rhs_matrix<T1>::output_type>(is_acceptable_rhs_matrix<T1>::cast(v1), is_acceptable_rhs_matrix<T2>::cast(v2));
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs_matrix<T2>::value, const cmatrix_expr<typename is_acceptable_rhs_matrix<T1>::output_type>&>::type
operator * (const T1& v1, const T2& v2) {
  return *new cmatrix_expr_dot<typename is_acceptable_rhs_matrix<T1>::output_type>(is_acceptable_rhs_matrix<T1>::cast(v1), is_acceptable_rhs_matrix<T2>::cast(v2));
}

//template <typename T1, typename T2>
//typename std::enable_if<is_acceptable_rhs_matrix<T2>::value, const cmatrix_expr<typename is_acceptable_rhs_matrix<T1>::output_type>&>::type
//operator * (const T1& v1, const T2& v2) {
//  return *new cmatrix_expr_mul<typename is_acceptable_rhs_matrix<T1>::output_type>(is_acceptable_rhs_matrix<T1>::cast(v1), is_acceptable_rhs_matrix<T2>::cast(v2));
//}
//
//template <typename T1, typename T2>
//typename std::enable_if<is_acceptable_rhs_matrix<T2>::value, const cmatrix_expr<typename is_acceptable_rhs_matrix<T1>::output_type>&>::type
//operator / (const T1& v1, const T2& v2) {
//  return *new cmatrix_expr_div<typename is_acceptable_rhs_matrix<T1>::output_type>(is_acceptable_rhs_matrix<T1>::cast(v1), is_acceptable_rhs_matrix<T2>::cast(v2));
//}

}

#endif
