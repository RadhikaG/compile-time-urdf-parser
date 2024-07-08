#ifndef CSCALAR_IMPL_H
#define CSCALAR_IMPL_H
#include "builder/dyn_var.h"
#include "builder/static_var.h"

namespace SpatialAlgebra {

template <typename T>
struct cscalar;

// Base class for all expressions that can appear on the RHS of a =
template <typename T>
struct cscalar_expr {
  virtual const builder::dyn_var<T> get_value_at() const { return 0; }
  operator cscalar<T>() const {
    return get_value_at();
  }
};

//template <typename T>
//struct cscalar_expr_const;
// class for leaf cscalar exprs in AST consisting of one cscalar
template <typename T>
struct cscalar_expr_leaf;

// T is arith type: int, float, double
template<typename T>
struct cscalar {
  builder::dyn_var<T> m_value;

  // compile time optimization fields
  builder::static_var<int> is_constant = false;
  builder::static_var<int> is_zero = false;
  builder::static_var<int> is_one = false;
  builder::static_var<T> constant_val = 0;

  // Delete the default implementations of the operators
  // and copy constructors
  //cscalar(const cscalar& other) = delete;

  cscalar() {
  }

  cscalar(builder::dyn_var<T>& dyn_val) : is_constant(false), is_zero(false), is_one(false), m_value(dyn_val) {}

  cscalar(const builder::dyn_var<T>& dyn_val) : is_constant(false), is_zero(false), is_one(false), m_value(dyn_val) {}

  cscalar(T value) : is_constant(true), constant_val(value) {
    if (value == 0)
      is_zero = true;
    else if (value == 1)
      is_one = true;
  }

  // Assignment operator overloads

  void operator=(const cscalar_expr<T> & rhs) {
    // need to trigger get_value_at calls for each sub-expr inside expr tree
    // to materialize AST into code
    m_value = rhs.get_value_at();
  }

  void operator=(const T& value) {
    m_value = value;
    is_constant = true;
    constant_val = value;
    if (value == 0)
      is_zero = true;
    else if (value == 1)
      is_one = true;
  }

  void operator=(const cscalar& other) {
    // this actually calls operator = for cscalar_expr<T> rhs
    *this = cscalar_expr_leaf<T>(other);
  }
  //void operator=(const cscalar& other) {
  //  // this actually calls operator = for cscalar_expr<T> rhs
  //  if (other.is_constant) {
  //    *this = other.constant_val;
  //  }
  //  else
  //    m_value = other.m_value;
  //}

  //const cscalar& operator+(const cscalar& other) const {
  //  if (is_constant && other.is_constant) {
  //    return *new cscalar(constant_val + other.constant_val);
  //  }
  //  if (is_zero)
  //    return *new cscalar(other.m_value);
  //  if (other.is_zero)
  //    return *this;

  //  return *new cscalar(m_value + other.m_value);
  //}

  //const cscalar& operator-(const cscalar& other) const {
  //  if (is_constant && other.is_constant)
  //    return *new cscalar(constant_val - other.constant_val);
  //  if (is_zero)
  //    return *new cscalar(-other.m_value);
  //  if (other.is_zero)
  //    return *this;

  //  return *new cscalar(m_value - other.m_value);
  //}

  //const cscalar& operator*(const cscalar& other) const {
  //  if (is_constant && other.is_constant)
  //    return *new cscalar(constant_val * other.constant_val);
  //  if (is_one)
  //    return other;
  //  if (other.is_one)
  //    return *this;

  //  return *new cscalar(m_value * other.m_value);
  //}

  //const cscalar& operator/(const cscalar& other) const {
  //  if (is_constant && other.is_constant)
  //    return *new cscalar(constant_val / other.constant_val);
  //  if (is_zero)
  //    return *new cscalar(0);
  //  //if (other.is_zero)
  //  //  assert
  //  if (other.is_one)
  //    return *this;

  //  return *new cscalar(m_value / other.m_value);
  //}

  //void operator+=(const cscalar& other) {
  //  *this = *this + other;
  //}

  //void operator-=(const cscalar& other) {
  //  *this = *this - other;
  //}

  //void operator*=(const cscalar& other) {
  //  *this = *this * other;
  //}

  //void operator/=(const cscalar& other) {
  //  *this = *this / other;
  //}

  friend std::ostream& operator<<(std::ostream& os, const cscalar& obj) {
    if (obj.is_constant)
      return os << obj.constant_val;
    else
      return os << "dyn";
  }

  void finalize() {
    if (!is_constant) return;
    *this = cscalar_expr_leaf<T>((T)constant_val);
  }
};

typedef cscalar<int> cscalari;
typedef cscalar<float> cscalarf;
typedef cscalar<double> cscalard;


//// RHS class for constants in the generated code
//template <typename T>
//struct cscalar_expr_const: public cscalar_expr<T> {
//  T m_value;
//  cscalar_expr_const (const T& value): m_value(value) {}
//  
//  const builder::dyn_var<T> get_value_at() const {
//    // Always return the same constant value
//    return m_value;
//  }
//};

// RHS class for regular leaf cscalars
template <typename T>
struct cscalar_expr_leaf: public cscalar_expr<T> {
  const struct cscalar<T>& m_cscalar;
  cscalar_expr_leaf (const T& value): m_cscalar(value) {}
  cscalar_expr_leaf (const struct cscalar<T>& value): m_cscalar(value) {}
  
  const builder::dyn_var<T> get_value_at() const {
    if (m_cscalar.is_constant) {
      return m_cscalar.constant_val;        
    }
    return const_cast<builder::dyn_var<T>&>(m_cscalar.m_value);
  }
};

// Non leaf RHS expr classes

// Equality
template <typename T>
struct cscalar_expr_eq: public cscalar_expr<T> {
  const struct cscalar_expr<T>& expr1;
  const struct cscalar_expr<T>& expr2;

  cscalar_expr_eq(const struct cscalar_expr<T>& expr1, const struct cscalar_expr<T>& expr2):
    expr1(expr1), expr2(expr2) {
  }

  const builder::dyn_var<T> get_value_at() const {
    return expr1.get_value_at() == expr2.get_value_at();
  }
};

// Addition
template <typename T>
struct cscalar_expr_add: public cscalar_expr<T> {
  const struct cscalar_expr<T>& expr1;
  const struct cscalar_expr<T>& expr2;

  cscalar_expr_add(const struct cscalar_expr<T>& expr1, const struct cscalar_expr<T>& expr2):
    expr1(expr1), expr2(expr2) {
  }

  const builder::dyn_var<T> get_value_at() const {
    return expr1.get_value_at() + expr2.get_value_at();
  }
};

// Subtraction
template <typename T>
struct cscalar_expr_sub: public cscalar_expr<T> {
  const struct cscalar_expr<T>& expr1;
  const struct cscalar_expr<T>& expr2;

  cscalar_expr_sub(const struct cscalar_expr<T>& expr1, const struct cscalar_expr<T>& expr2):
    expr1(expr1), expr2(expr2) {
  }

  const builder::dyn_var<T> get_value_at() const {
    return expr1.get_value_at() - expr2.get_value_at();
  }
};

// Multiplication
template <typename T>
struct cscalar_expr_mul: public cscalar_expr<T> {
  const struct cscalar_expr<T>& expr1;
  const struct cscalar_expr<T>& expr2;

  cscalar_expr_mul(const struct cscalar_expr<T>& expr1, const struct cscalar_expr<T>& expr2):
    expr1(expr1), expr2(expr2) {
  }

  const builder::dyn_var<T> get_value_at() const {
    return expr1.get_value_at() * expr2.get_value_at();
  }
};

// Multiplication Assignment
template <typename T>
struct cscalar_expr_mul_ass: public cscalar_expr<T> {
  const struct cscalar_expr<T>& expr1;
  const struct cscalar_expr<T>& expr2;

  cscalar_expr_mul_ass(const struct cscalar_expr<T>& expr1, const struct cscalar_expr<T>& expr2):
    expr1(expr1), expr2(expr2) {
  }

  const builder::dyn_var<T> get_value_at() const {
    return expr1.get_value_at() * expr2.get_value_at();
  }
};

template <typename T>
struct cscalar_expr_div: public cscalar_expr<T> {
  const struct cscalar_expr<T>& expr1;
  const struct cscalar_expr<T>& expr2;

  cscalar_expr_div(const struct cscalar_expr<T>& expr1, const struct cscalar_expr<T>& expr2):
    expr1(expr1), expr2(expr2) {
  }

  const builder::dyn_var<T> get_value_at() const {
    return expr1.get_value_at() / expr2.get_value_at();
  }
};

}


#endif
