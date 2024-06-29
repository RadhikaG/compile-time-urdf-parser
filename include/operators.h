#ifndef BARRAY_OPERATOR_H
#define BARRAY_OPERATOR_H

#include "cscalar.h"
#include <Eigen/Dense>

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
  return (v1 * v2);
}

}

namespace Eigen {
template<typename T>
// inherit traits from arith type T
struct NumTraits<SpatialAlgebra::cscalar_expr<T>> : NumTraits<T> {
  typedef SpatialAlgebra::cscalar_expr<T> Real;
  typedef SpatialAlgebra::cscalar_expr<T> NonInteger;
  typedef SpatialAlgebra::cscalar_expr<T> Nested;

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

// Matrix, X dim, cscalar, int|float|double
typedef Matrix<SpatialAlgebra::cscalar_expr<int>, Dynamic, Dynamic> MatrixXcsi;
typedef Matrix<SpatialAlgebra::cscalar_expr<float>, Dynamic, Dynamic> MatrixXcsf;
typedef Matrix<SpatialAlgebra::cscalar_expr<double>, Dynamic, Dynamic> MatrixXcsd;
}

#endif

