#ifndef CSCALAR_IMPL_H
#define CSCALAR_IMPL_H
#include "builder/dyn_var.h"
#include "builder/static_var.h"
#include <Eigen/Dense>

namespace SpatialAlgebra {

// T is arith type: int, float, double
template<typename T>
struct cscalar {
  builder::dyn_var<T*> m_value;

  // compile time optimization fields
  builder::static_var<int> is_constant = true;
  builder::static_var<int> is_zero = true;
  builder::static_var<int> is_one = false;
  builder::static_var<T> constant_val = 0;

  cscalar() {
  }

  cscalar(builder::dyn_var<T*>& dyn_val) : is_constant(false), is_zero(false), m_value(dyn_val) {
  }

  cscalar(T value) : is_constant(true), constant_val(value) {
    if (value == 0)
      is_zero = true;
    if (value == 1)
      is_one = true;
  }

  cscalar operator+(const cscalar& other) const {
    if (is_constant && other.is_constant)
      return cscalar(constant_val + other.constant_val);
    if (is_zero)
      return other;
    if (other.is_zero)
      return *this;

    return cscalar(m_value + other.m_value);
  }

  cscalar operator-(const cscalar& other) const {
    if (is_constant && other.is_constant)
      return cscalar(constant_val - other.constant_val);
    if (is_zero)
      return cscalar(-other.m_value);
    if (other.is_zero)
      return *this;

    return cscalar(m_value - other.m_value);
  }

  cscalar operator*(const cscalar& other) const {
    if (is_constant && other.is_constant)
      return cscalar(constant_val * other.constant_val);
    if (is_one)
      return other;
    if (other.is_one)
      return *this;

    return cscalar(m_value * other.m_value);
  }

  cscalar operator/(const cscalar& other) const {
    if (is_constant && other.is_constant)
      return cscalar(constant_val / other.constant_val);
    if (is_zero)
      return cscalar(0);
    //if (other.is_zero)
    //  assert
    if (other.is_one)
      return *this;

    return cscalar(m_value / other.m_value);
  }

  cscalar operator*=(const cscalar& other) {
    if (is_constant && other.is_constant)
      this->constant_val *= other.constant_val;
    else
      
    
    return *this;
  }

  void operator=(const T& value) {
    is_constant = true;
    constant_val = value;
    if (value == 0)
      is_zero = true;
    else if (value == 1)
      is_one = true;
  }

  void operator=(const cscalar& other) {
    *this = other;
  }

  friend std::ostream& operator<<(std::ostream& os, const cscalar& obj) {
    if (obj.is_constant)
      return os << obj.constant_val;
    else
      return os << "dyn";
  }
};

typedef cscalar<int> cscalari;
typedef cscalar<float> cscalarf;
typedef cscalar<double> cscalard;
}

namespace Eigen {
template<typename T>
// inherit traits from arith type T
struct NumTraits<SpatialAlgebra::cscalar<T>> : NumTraits<T> {
  typedef SpatialAlgebra::cscalar<T> Real;
  typedef SpatialAlgebra::cscalar<T> NonInteger;
  typedef SpatialAlgebra::cscalar<T> Nested;

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
typedef Matrix<SpatialAlgebra::cscalari, Dynamic, Dynamic> MatrixXcsi;
typedef Matrix<SpatialAlgebra::cscalarf, Dynamic, Dynamic> MatrixXcsf;
typedef Matrix<SpatialAlgebra::cscalard, Dynamic, Dynamic> MatrixXcsd;
}

#endif
