#ifndef BARRAY_OPERATOR_H
#define BARRAY_OPERATOR_H

#include "cscalar.h"
#include "Eigen/Dense"

namespace SpatialAlgebra {


// A simple type trait to identify if 
// operators should be overloaded
template <typename T>
struct is_acceptable_rhs {
  static const bool value = false;
};

template <typename T>
struct is_acceptable_rhs<cscalar<T>> {
  static const bool value = true;
  using output_type = T;
  static const cscalar_expr<T>& cast(const cscalar<T>& v) {
    return *new cscalar_expr_leaf<T>(v);
  }
};

template <typename T>
struct is_acceptable_rhs<cscalar_expr<T>> {
  static const bool value = true;
  using output_type = T;
  static const cscalar_expr<T>& cast(const cscalar_expr<T>& v) {
    return v;
  }
};

template <>
struct is_acceptable_rhs<int> {
  static const bool value = true;
  using output_type = int;
  static const cscalar_expr<int>& cast(const int& v) {
    return * new cscalar_expr_leaf<int>(v);
  }
};

template <>
struct is_acceptable_rhs<float> {
  static const bool value = true;
  using output_type = float;
  static const cscalar_expr<float>& cast(const float& v) {
    return * new cscalar_expr_leaf<float>(v);
  }
};

template <>
struct is_acceptable_rhs<double> {
  static const bool value = true;
  using output_type = double;
  static const cscalar_expr<double>& cast(const double& v) {
    return * new cscalar_expr_leaf<double>(v);
  }
};


template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs<T2>::value, const cscalar_expr<typename is_acceptable_rhs<T1>::output_type>&>::type
operator + (const T1& v1, const T2& v2) {
  return *new cscalar_expr_add<typename is_acceptable_rhs<T1>::output_type>(is_acceptable_rhs<T1>::cast(v1), is_acceptable_rhs<T2>::cast(v2));
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs<T2>::value, const cscalar_expr<typename is_acceptable_rhs<T1>::output_type>&>::type
operator - (const T1& v1, const T2& v2) {
  return *new cscalar_expr_sub<typename is_acceptable_rhs<T1>::output_type>(is_acceptable_rhs<T1>::cast(v1), is_acceptable_rhs<T2>::cast(v2));
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs<T2>::value, const cscalar_expr<typename is_acceptable_rhs<T1>::output_type>&>::type
operator * (const T1& v1, const T2& v2) {
  return *new cscalar_expr_mul<typename is_acceptable_rhs<T1>::output_type>(is_acceptable_rhs<T1>::cast(v1), is_acceptable_rhs<T2>::cast(v2));
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs<T2>::value, const cscalar_expr<typename is_acceptable_rhs<T1>::output_type>&>::type
operator / (const T1& v1, const T2& v2) {
  return *new cscalar_expr_div<typename is_acceptable_rhs<T1>::output_type>(is_acceptable_rhs<T1>::cast(v1), is_acceptable_rhs<T2>::cast(v2));
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs<T2>::value, const cscalar_expr<typename is_acceptable_rhs<T1>::output_type>&>::type
operator += (const T1& v1, const T2& v2) {
  return (v1 + v2);
}

template <typename T1, typename T2>
typename std::enable_if<is_acceptable_rhs<T2>::value, const cscalar_expr<typename is_acceptable_rhs<T1>::output_type>&>::type
operator *= (const T1& v1, const T2& v2) {
  return (v1 = v1 * v2);
}

}

namespace Eigen {
template<typename T>
// inherit traits from arith type T
struct NumTraits<SpatialAlgebra::cscalar_expr<T>> : NumTraits<T> {
  typedef SpatialAlgebra::cscalar_expr<T> Real;
  typedef SpatialAlgebra::cscalar_expr<T> NonInteger;
  typedef SpatialAlgebra::cscalar_expr<T> Nested;

  // modified from CppAD eigen overloads
  // https://github.com/coin-or/CppAD/blob/master/include/cppad/example/cppad_eigen.hpp

  // machine epsilon with type of real part of x
  // (use assumption that T is not complex)
  static SpatialAlgebra::cscalar_expr<T> epsilon(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar_expr<T> >::epsilon(); }

  // relaxed version of machine epsilon for comparison of different
  // operations that should result in the same value
  static SpatialAlgebra::cscalar_expr<T> dummy_precision(void)
  {   return 100. *
          std::numeric_limits< SpatialAlgebra::cscalar_expr<T> >::epsilon();
  }

  // minimum normalized positive value
  static SpatialAlgebra::cscalar_expr<T> lowest(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar_expr<T> >::min(); }

  // maximum finite value
  static SpatialAlgebra::cscalar_expr<T> highest(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar_expr<T> >::max(); }

  // number of decimal digits that can be represented without change.
  static int digits10(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar_expr<T> >::digits10; }

  // not a number
  static SpatialAlgebra::cscalar_expr<T> quiet_NaN(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar_expr<T> >::quiet_NaN(); }

  // positive infinite value
  static SpatialAlgebra::cscalar_expr<T> infinity(void)
  {   return std::numeric_limits< SpatialAlgebra::cscalar_expr<T> >::infinity(); }

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
struct ScalarBinaryOpTraits<SpatialAlgebra::cscalar_expr<T>, T, BinOp>{
  typedef SpatialAlgebra::cscalar_expr<T> ReturnType;
};
template<typename T, typename BinOp>
struct ScalarBinaryOpTraits<T, SpatialAlgebra::cscalar_expr<T>, BinOp>
{
  typedef SpatialAlgebra::cscalar_expr<T> ReturnType;
};

// Matrix, X dim, cscalar, int|float|double
typedef Matrix<SpatialAlgebra::cscalar_expr<int>, Dynamic, Dynamic> MatrixXcsi;
typedef Matrix<SpatialAlgebra::cscalar_expr<float>, Dynamic, Dynamic> MatrixXcsf;
typedef Matrix<SpatialAlgebra::cscalar_expr<double>, Dynamic, Dynamic> MatrixXcsd;

//template<typename T>
//// inherit traits from arith type T
//struct NumTraits<builder::dyn_var<T>> : NumTraits<T> {
//  typedef builder::dyn_var<T> Real;
//  typedef builder::dyn_var<T> NonInteger;
//  typedef builder::dyn_var<T> Nested;
//
//  // machine epsilon with type of real part of x
//  // (use assumption that T is not complex)
//  static builder::dyn_var<T> epsilon(void)
//  {   return std::numeric_limits< builder::dyn_var<T> >::epsilon(); }
//
//  // relaxed version of machine epsilon for comparison of different
//  // operations that should result in the same value
//  static builder::dyn_var<T> dummy_precision(void)
//  {   return 100. *
//          std::numeric_limits< builder::dyn_var<T> >::epsilon();
//  }
//
//  // minimum normalized positive value
//  static builder::dyn_var<T> lowest(void)
//  {   return std::numeric_limits< builder::dyn_var<T> >::min(); }
//
//  // maximum finite value
//  static builder::dyn_var<T> highest(void)
//  {   return std::numeric_limits< builder::dyn_var<T> >::max(); }
//
//  // number of decimal digits that can be represented without change.
//  static int digits10(void)
//  {   return std::numeric_limits< builder::dyn_var<T> >::digits10; }
//
//  // not a number
//  static builder::dyn_var<T> quiet_NaN(void)
//  {   return std::numeric_limits< builder::dyn_var<T> >::quiet_NaN(); }
//
//  // positive infinite value
//  static builder::dyn_var<T> infinity(void)
//  {   return std::numeric_limits< builder::dyn_var<T> >::infinity(); }
//
//  enum {
//    IsComplex = 0,
//    IsInteger = 0,
//    IsSigned = 1,
//    RequireInitialization = 1,
//    ReadCost = 1,
//    AddCost = 3,
//    MulCost = 3
//  };
//};
//
//template<typename T, typename BinOp>
//struct ScalarBinaryOpTraits<builder::dyn_var<T>, T, BinOp>{
//  typedef builder::dyn_var<T> ReturnType;
//};
//template<typename T, typename BinOp>
//struct ScalarBinaryOpTraits<T, builder::dyn_var<T>, BinOp>
//{
//  typedef builder::dyn_var<T> ReturnType;
//};


//typedef Matrix<builder::dyn_var<int>, Dynamic, Dynamic> MatrixXcsi;
//typedef Matrix<builder::dyn_var<float>, Dynamic, Dynamic> MatrixXcsf;
//typedef Matrix<builder::dyn_var<double>, Dynamic, Dynamic> MatrixXcsd;
}

#endif

