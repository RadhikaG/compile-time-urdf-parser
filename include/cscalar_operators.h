#ifndef CSCALAR_OPERATOR_H
#define CSCALAR_OPERATOR_H

#include "builder/dyn_var.h"
#include "cscalar_impl.h"
#include "Eigen/Dense"
#include <type_traits>

namespace SpatialAlgebra {


// A simple type trait to identify if 
// operators should be overloaded
template <typename T>
struct is_acceptable_rhs_scalar {
  static const bool value = false;
};

template <typename T>
struct is_acceptable_rhs_scalar<cscalar<T>> {
  static const bool value = true;
  using output_type = T;
  static const cscalar_expr<T>& cast(const cscalar<T>& v) {
    return *new cscalar_expr_leaf<T>(v);
  }
};

template <typename T>
struct is_acceptable_rhs_scalar<cscalar_expr<T>> {
  static const bool value = true;
  using output_type = T;
  static const cscalar_expr<T>& cast(const cscalar_expr<T>& v) {
    return v;
  }
};

template <>
struct is_acceptable_rhs_scalar<int> {
  static const bool value = true;
  using output_type = int;
  static const cscalar_expr<int>& cast(const int& v) {
    return * new cscalar_expr_leaf<int>(v);
  }
};

template <>
struct is_acceptable_rhs_scalar<float> {
  static const bool value = true;
  using output_type = float;
  static const cscalar_expr<float>& cast(const float& v) {
    return * new cscalar_expr_leaf<float>(v);
  }
};

template <>
struct is_acceptable_rhs_scalar<double> {
  static const bool value = true;
  using output_type = double;
  static const cscalar_expr<double>& cast(const double& v) {
    return * new cscalar_expr_leaf<double>(v);
  }
};

// Bin op overloads: 
// Return type: cscalar_expr
// T1: [cscalar, cscalar_expr, int|float|double]
// T2: [cscalar, cscalar_expr, int|float|double]

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs_scalar<T2>::value, const cscalar_expr<typename is_acceptable_rhs_scalar<T1>::output_type>&>::type
operator + (const T1& v1, const T2& v2) {
  return *new cscalar_expr_add<typename is_acceptable_rhs_scalar<T1>::output_type>(is_acceptable_rhs_scalar<T1>::cast(v1), is_acceptable_rhs_scalar<T2>::cast(v2));
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs_scalar<T2>::value, const cscalar_expr<typename is_acceptable_rhs_scalar<T1>::output_type>&>::type
operator - (const T1& v1, const T2& v2) {
  return *new cscalar_expr_sub<typename is_acceptable_rhs_scalar<T1>::output_type>(is_acceptable_rhs_scalar<T1>::cast(v1), is_acceptable_rhs_scalar<T2>::cast(v2));
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs_scalar<T2>::value, const cscalar_expr<typename is_acceptable_rhs_scalar<T1>::output_type>&>::type
operator * (const T1& v1, const T2& v2) {
  return *new cscalar_expr_mul<typename is_acceptable_rhs_scalar<T1>::output_type>(is_acceptable_rhs_scalar<T1>::cast(v1), is_acceptable_rhs_scalar<T2>::cast(v2));
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs_scalar<T2>::value, const cscalar_expr<typename is_acceptable_rhs_scalar<T1>::output_type>&>::type
operator / (const T1& v1, const T2& v2) {
  return *new cscalar_expr_div<typename is_acceptable_rhs_scalar<T1>::output_type>(is_acceptable_rhs_scalar<T1>::cast(v1), is_acceptable_rhs_scalar<T2>::cast(v2));
}

//template <typename T1, typename T2>
//typename std::enable_if<is_acceptable_rhs_scalar<T2>::value, bool>::type
//operator == (const T1& v1, const T2& v2) {
//  return false;
//}

// Arith-with-assignment op overloads: 
// Return type: none
// T1: [cscalar]
// T2: [cscalar, cscalar_expr, int|float|double]

template <typename T>
struct is_acceptable_lhs_scalar_assign {
  static const bool value = false;
};

template <typename T>
struct is_acceptable_lhs_scalar_assign<cscalar<T>> {
  static const bool value = true;
  using output_type = T;
};

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_lhs_scalar_assign<T1>::value, void>::type
operator += (T1& v1, const T2& v2) {
  v1 = v1 + v2;
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_lhs_scalar_assign<T1>::value, void>::type
operator -= (T1& v1, const T2& v2) {
  v1 = v1 - v2;
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_lhs_scalar_assign<T1>::value, void>::type
operator *= (T1& v1, const T2& v2) {
  v1 = v1 * v2;
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_lhs_scalar_assign<T1>::value, void>::type
operator /= (T1& v1, const T2& v2) {
  v1 = v1 / v2;
}
}

namespace Eigen {
template<typename T>
// inherit traits from arith type T
struct NumTraits<SpatialAlgebra::cscalar<T>> {
  typedef SpatialAlgebra::cscalar<T> Real;
  typedef SpatialAlgebra::cscalar<T> NonInteger;
  typedef SpatialAlgebra::cscalar<T> Literal;
  typedef SpatialAlgebra::cscalar<T> Nested;

  // modified from CppAD eigen overloads
  // https://github.com/coin-or/CppAD/blob/master/include/cppad/example/cppad_eigen.hpp

  // machine epsilon with type of real part of x
  // (use assumption that T is not complex)
  static SpatialAlgebra::cscalar<T> epsilon(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar<T> >::epsilon(); }

  // relaxed version of machine epsilon for comparison of different
  // operations that should result in the same value
  static SpatialAlgebra::cscalar<T> dummy_precision(void)
  {   return 100. *
          std::numeric_limits< SpatialAlgebra::cscalar<T> >::epsilon();
  }

  // minimum normalized positive value
  static SpatialAlgebra::cscalar<T> lowest(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar<T> >::min(); }

  // maximum finite value
  static SpatialAlgebra::cscalar<T> highest(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar<T> >::max(); }

  // number of decimal digits that can be represented without change.
  static int digits10(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar<T> >::digits10; }

  // not a number
  static SpatialAlgebra::cscalar<T> quiet_NaN(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar<T> >::quiet_NaN(); }

  // positive infinite value
  static SpatialAlgebra::cscalar<T> infinity(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar<T> >::infinity(); }

  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 3,
    MulCost = 3
  };
};

template<typename T, typename BinOp>
struct ScalarBinaryOpTraits<SpatialAlgebra::cscalar<T>, SpatialAlgebra::cscalar<T>, BinOp>
{
  typedef SpatialAlgebra::cscalar<T> ReturnType;
};

// Matrix, X dim, cscalar, int|float|double
typedef Matrix<SpatialAlgebra::cscalar<int>, Dynamic, Dynamic> MatrixXcsi;
typedef Matrix<SpatialAlgebra::cscalar<float>, Dynamic, Dynamic> MatrixXcsf;
typedef Matrix<SpatialAlgebra::cscalar<double>, Dynamic, Dynamic> MatrixXcsd;
}

#endif

