#ifndef CMATRIX_IMPL_H
#define CMATRIX_IMPL_H
#include "forward_declarations.h"
#include "builder/static_var.h"
#include "builder/dyn_var.h"
#include <vector>

namespace SpatialAlgebra {

std::vector<int> match_expr_sizes(std::vector<int>, std::vector<int>);

// Base class for all expressions that can appear on the RHS of a =
template<typename T>
struct cmatrix_expr {
  virtual const cscalar_expr<T>& get_value_at(std::vector<builder::static_var<int>*> indices) const { return *new cscalar_expr_leaf<T>(0); }
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
  int flattened_size;

  std::vector<int> m_sizes;

  // cmatrix is internally an array of cscalars
  // cscalar tracks whether value is static or dynamic
  std::vector<cscalar<T>*> m_buffer;

  //// statically known sparsity tracking
  //// we resolve as many zeros at compile-time as possible
  //// for skipping zeros during matmul
  //std::vector<builder::static_var<int>*> m_zero_buffer;

  cmatrix(const std::vector<int>& sizes):
      m_dims(sizes.size()), m_sizes(std::move(sizes)) {
    flattened_size = 1;
    for (auto &size : m_sizes) {
      flattened_size *= size;
    }

    for (int i = 0; i < flattened_size; i++) {
      cscalar<T> * cs_entry = new cscalar<T>;
      m_buffer.push_back(cs_entry);
    }
  }

  cmatrix(const std::vector<int>& sizes, const std::vector<cscalar<T>>& buffer) :
      m_dims(sizes.size()), m_sizes(std::move(sizes)), m_buffer(std::move(buffer))
  {
    flattened_size = 1;
    for (auto &size : m_sizes) {
      flattened_size *= size;
    }
  }

  ~cmatrix() {
    for (int i = 0; i < flattened_size; i++) {
      cscalar<T> * popped_cs = m_buffer.back(); 
      m_buffer.pop_back();
      delete popped_cs;
    }
  }

  void unroll(std::vector<builder::static_var<int>*> indices, const cmatrix_expr<T>& rhs) {
    unsigned int index = indices.size();

    if (index == m_sizes.size()) {
      // this equality operator emits code corresponding to the AST associated
      // with each expr of cscalars
      // should trigger the = operator for cscalar against cscalar_exprs.
      *m_buffer[compute_flat_index(indices)] = rhs.get_value_at(indices);
      return;
    }

    builder::static_var<int> i = 0;
    indices.push_back(&i);

    for (; i < m_sizes[index]; i = i+1) {
      unroll(indices, rhs);
    }
  }

  void operator=(const cmatrix_expr<T>& rhs) {
    match_expr_sizes(m_sizes, rhs.get_expr_size());
    unroll({}, rhs);
  }

  int compute_flat_index(std::vector<builder::static_var<int>*> indices, unsigned int dim = -1) const {
    if (dim == -1)
      dim = m_sizes.size() - 1;
    if (dim == 0)
      return *indices[dim];
    return *(indices[dim]) + (int) m_sizes[dim] * compute_flat_index(indices, dim-1);
  }

  static cmatrix<T>& eye(int n) {
    
  }

  static cmatrix<T>& zeros(const std::vector<int>& sizes) {
    
  }
};

// Building blocks for AST nodes

// Leaf node for AST
template<typename T>
struct cmatrix_expr_leaf: public cmatrix_expr<T> {
  const struct cmatrix<T>& m_matrix;
  cmatrix_expr_leaf(const struct cmatrix<T>& matrix): m_matrix(matrix) {}

  const cscalar_expr<T>& get_value_at(std::vector<builder::static_var<int>*> indices) const {
    return *new cscalar_expr_leaf<T>(*(m_matrix.m_buffer[m_matrix.compute_flat_index(indices)]));
  }

  const std::vector<int> get_expr_size(void) const {
    return m_matrix.m_sizes;
  }
};

// Non leaf nodes, binary ops

template<typename T>
struct cmatrix_expr_add: public cmatrix_expr<T> {
  const struct cmatrix_expr<T>& expr1;
  const struct cmatrix_expr<T>& expr2;

  std::vector<int> computed_sizes;

  cmatrix_expr_add(const struct cmatrix_expr<T>& expr1, const struct cmatrix_expr<T>& expr2): 
      expr1(expr1), expr2(expr2) {
    computed_sizes = match_expr_sizes(expr1.get_expr_size(), expr2.get_expr_size());
  }

  const std::vector<int> get_expr_size(void) const {
    return computed_sizes;
  }

  const cscalar_expr<T>& get_value_at(std::vector<builder::static_var<int>*> indices) const {
    return expr1.get_value_at(indices) + expr2.get_value_at(indices);
  }
};

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

  const builder::dyn_var<T> get_value_at(std::vector<builder::static_var<int>*> indices) const {
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
