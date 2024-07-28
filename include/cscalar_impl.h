#ifndef CSCALAR_IMPL_H
#define CSCALAR_IMPL_H
#include "builder/dyn_var.h"
#include "builder/static_var.h"

using builder::dyn_var;
using builder::static_var;

namespace SpatialAlgebra {

template <typename T>
struct cscalar;

// Base class for all expressions that can appear on the RHS of a =
template <typename T>
struct cscalar_expr {
  // each cscalar_expr is only created once and these values remain unchanged
  int is_constant;
  int is_zero;
  int is_one;
  T constant_val;

  virtual const dyn_var<T> get_value_at() const { return 0; }
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
  dyn_var<T> m_value;

  // compile time optimization fields
  static_var<int> is_constant = false;
  static_var<int> is_zero = false;
  static_var<int> is_one = false;
  static_var<T> constant_val = 0;
  //int is_constant = false;
  //int is_zero = false;
  //int is_one = false;
  //T constant_val = 0;
  // default constructor

  cscalar() {}

  ~cscalar() {}

  // Delete the default implementations of the operators
  // and copy constructors
  //cscalar(const cscalar& other) = delete;
  cscalar(const cscalar& other) : m_value(other.m_value) {
    std::cout << "copy ctor\n";
    is_constant = other.is_constant;
    is_zero = other.is_zero;
    is_one = other.is_one;
    constant_val = other.constant_val;
  }

  cscalar(T value) : constant_val(value) {
    std::cout << "value ctor\n";
    is_constant = true;
    constant_val = value;
    if (value == 0)
      is_zero = true;
    else if (value == 1)
      is_one = true;
  }

  cscalar(dyn_var<T>& dyn_val) : m_value(dyn_val) {
    is_constant = false;
    is_zero = false;
    is_one = false;
  }

  cscalar(const dyn_var<T>& dyn_val) : m_value(dyn_val) {
    is_constant = false;
    is_zero = false;
    is_one = false;
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
//  const dyn_var<T> get_value_at() const {
//    // Always return the same constant value
//    return m_value;
//  }
//};

// RHS class for regular leaf cscalars
template <typename T>
struct cscalar_expr_leaf: public cscalar_expr<T> {
  const struct cscalar<T>& m_cscalar;
  cscalar_expr_leaf (const T& value): m_cscalar(value) { set_compile_time_opt(); }
  cscalar_expr_leaf (const struct cscalar<T>& value): m_cscalar(value) { set_compile_time_opt(); }

  void set_compile_time_opt() {
    cscalar_expr<T>::is_constant = m_cscalar.is_constant;
    cscalar_expr<T>::constant_val = m_cscalar.constant_val;
    cscalar_expr<T>::is_zero = m_cscalar.is_zero;
    cscalar_expr<T>::is_one = m_cscalar.is_one;
  }
  
  const dyn_var<T> get_value_at() const {
    return const_cast<dyn_var<T>&>(m_cscalar.m_value);
  }
};

// Non leaf RHS expr classes

// Addition
template <typename T>
struct cscalar_expr_add: public cscalar_expr<T> {
  const struct cscalar_expr<T>& expr1;
  const struct cscalar_expr<T>& expr2;

  cscalar_expr_add(const struct cscalar_expr<T>& expr1, const struct cscalar_expr<T>& expr2):
    expr1(expr1), expr2(expr2) {
  }

  const dyn_var<T> get_value_at() const {
    if (expr1.is_zero && expr2.is_zero)
      return 0;
    if (expr1.is_zero)
      return expr2.get_value_at();
    if (expr2.is_zero)
      return expr1.get_value_at();

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

  const dyn_var<T> get_value_at() const {
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

  const dyn_var<T> get_value_at() const {
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

  const dyn_var<T> get_value_at() const {
    return expr1.get_value_at() / expr2.get_value_at();
  }
};

}


#endif
