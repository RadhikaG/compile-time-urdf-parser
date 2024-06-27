#ifndef CMATRIX_IMPL_H
#define CMATRIX_IMPL_H
#include "builder/forward_declarations.h"
#include <vector>

namespace SpatialAlgebra {

std::vector<int> match_expr_sizes(std::vector<int>, std::vector<int>);

// Base class for all expressions that can appear on the RHS of a =
template<typename T>
struct cmatrix_expr {
  virtual const builder::dyn_var<T> get_value_at(std::vector<builder::dyn_var<int>*> indices) const {
    return 0;
  }
  virtual const std::vector<int> get_expr_size(void) const {
    return {};
  }
};

//template<typename T>
//struct cmatrix_expr_const;
template<typename T>
struct cmatrix_expr_array;

template<typename T>
struct cmatrix {
  int m_dims;

  std::vector<int> m_sizes;

  builder::dyn_var<T*> m_buffer;

  // statically known sparsity tracking
  // we resolve as many zeros at compile-time as possible
  // for skipping zeros during matmul
  std::vector<builder::static_var<int>*> m_zero_buffer;

  cmatrix(const std::vector<int>& sizes, const builder::dyn_var<T*>& buffer):
      m_dims(sizes.size()), m_sizes(std::move(sizes)), m_buffer(buffer) {
  }

  cmatrix(const std::vector<int>& sizes):
      m_dims(sizes.size()), m_sizes(std::move(sizes)), m_buffer({}) {
  }

  static cmatrix<T> eye(int n) {
    
  }

  static cmatrix<T> zeros(const std::vector<int>& sizes) {
    
  }
};

// Building blocks for AST nodes

// Leaf node for AST
template<typename T>
struct cmatrix_expr_array: public cmatrix_expr<T> {
  const struct cmatrix<T>& m_matrix;
  cmatrix_expr_array(const struct cmatrix<T>& matrix): m_matrix(matrix) {}

  const builder::dyn_var<T> get_value_at(std::vector<builder::dyn_var<int>*> indices) const {
    return const_cast<builder::dyn_var<T*>&>(m_matrix.m_buffer)[m_matrix.compute_flat_index(indices)];
  }

  const std::vector<int> get_expr_size(void) const {
    return m_matrix.m_sizes;
  }
};

// Non leaf nodes, binary ops

template<typename T>
struct cmatrix_expr_vstack: public cmatrix_expr<T> {
  const struct cmatrix_expr<T>& expr1;
  const struct cmatrix_expr<T>& expr2;

  std::vector<int> computed_sizes;

  cmatrix_expr_vstack(const struct cmatrix_expr<T>& expr1, const struct cmatrix_expr<T>& expr2): 
      expr1(expr1), expr2(expr2) {
    //computed_sizes = match_expr_sizes(expr1.get_expr_size(), expr2.get_expr_size());
  }

  const std::vector<int> get_expr_size(void) const {
    return computed_sizes;
  }

  const builder::dyn_var<T> get_value_at(std::vector<builder::dyn_var<int>*> indices) const {
    //return expr1.get_value_at(indices) + expr2.get_value_at(indices);
    // todo
  }
};

template<typename T>
struct cmatrix_expr_hstack: public cmatrix_expr<T> {
  const struct cmatrix_expr<T>& expr1;
  const struct cmatrix_expr<T>& expr2;

  std::vector<int> computed_sizes;

  cmatrix_expr_hstack(const struct cmatrix_expr<T>& expr1, const struct cmatrix_expr<T>& expr2): 
      expr1(expr1), expr2(expr2) {
    //computed_sizes = match_expr_sizes(expr1.get_expr_size(), expr2.get_expr_size());
  }

  const std::vector<int> get_expr_size(void) const {
    return computed_sizes;
  }

  const builder::dyn_var<T> get_value_at(std::vector<builder::dyn_var<int>*> indices) const {
    //return expr1.get_value_at(indices) + expr2.get_value_at(indices);
    // todo
  }
};

template<typename T>
struct cmatrix_expr_matmul: public cmatrix_expr<T> {
  const struct cmatrix_expr<T>& expr1;
  const struct cmatrix_expr<T>& expr2;

  std::vector<int> computed_sizes;

  cmatrix_expr_matmul(const struct cmatrix_expr<T>& expr1, const struct cmatrix_expr<T>& expr2): 
      expr1(expr1), expr2(expr2) {
    //computed_sizes = match_expr_sizes(expr1.get_expr_size(), expr2.get_expr_size());
  }

  const std::vector<int> get_expr_size(void) const {
    return computed_sizes;
  }

  const builder::dyn_var<T> get_value_at(std::vector<builder::dyn_var<int>*> indices) const {
    //return expr1.get_value_at(indices) + expr2.get_value_at(indices);
    // todo
  }
};

}

#endif
