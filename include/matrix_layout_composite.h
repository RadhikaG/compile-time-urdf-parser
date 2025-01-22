#ifndef MATRIX_LAYOUT_COMPOSITE_H
#define MATRIX_LAYOUT_COMPOSITE_H
#include "matrix_layout.h"
#include <utility>
#include <vector>

using builder::dyn_var;
using builder::static_var;

namespace ctup {

template <typename Scalar>
struct blocked_layout_expr : public zero_cst_status_checkable {
  void trigger_abstract_method_assert() const {
    assert(false && "abstract blocked_layout_expr method call");
  }
  virtual bool is_nonzero(size_t i, size_t j) const override {
    trigger_abstract_method_assert();
    return false;
  }

  virtual bool is_nonconstant(size_t i, size_t j) const override {
    trigger_abstract_method_assert();
    return false;
  }

  virtual bool is_zero(size_t i, size_t j) const override {
    return !is_nonzero(i, j);
  }

  virtual bool is_constant(size_t i, size_t j) const override {
    return !is_nonconstant(i, j);
  }

  virtual std::vector<size_t> get_expr_shape() const {
    trigger_abstract_method_assert();
    return std::vector<size_t>();
  }

  /* operations on blocks */

  virtual bool is_block_nonzero(size_t b_i, size_t b_j) const {
    trigger_abstract_method_assert();
    return false;
  }

  virtual bool is_block_zero(size_t b_i, size_t b_j) const {
    !is_block_zero(b_i, b_j);
  }

  virtual matrix_layout_expr<Scalar> &gen_block_expr(size_t b_i, size_t b_j) const {
    trigger_abstract_method_assert();
    return matrix_layout_expr<Scalar>();
  }

  virtual std::vector<size_t> get_block_expr_shape(size_t b_i, size_t b_j) const {
    trigger_abstract_method_assert();
    return {};
  }
};

// base class for composite matrix_layouts that hold
// other layouts within them.
template <typename Scalar>
struct blocked_layout : public zero_cst_status_checkable {
  const size_t shape[2];
  const size_t max_idx;

  std::map<std::pair<size_t, size_t>, typename matrix_layout<Scalar>::Ptr> block_idx_to_block;

  // block_idx to top left corner of block
  // std::vector because each block guaranteed to have a top left corner
  std::vector<std::vector<std::pair<size_t, size_t>>> block_idx_to_top_left_corner;
  std::vector<std::vector<std::pair<size_t, size_t>>> block_idx_to_shape;
  size_t n_blocks_r;
  size_t n_blocks_c;

  blocked_layout(size_t _n_rows, size_t _n_cols) : 
      shape{_n_rows, _n_cols}, max_idx(_n_rows * _n_cols) {
  }

  size_t get_flattened_index(size_t i, size_t j) const {
    size_t  flattened = i * shape[1] + j;
    assert(flattened < shape[0] * shape[1] && "out of bounds");
    return flattened;
  }

  // e.g. for 4x4 homogeneous transform, row_partitions = {0, 3}, col_partitions = {0, 3}
  void set_partitions(std::vector<size_t> row_partitions, std::vector<size_t> col_partitions) {
    n_blocks_r = row_partitions.size();
    n_blocks_c = col_partitions.size();

    for (static_var<size_t> i = 0; i < n_blocks_r; i = i+1) {
      block_idx_to_top_left_corner.push_back({});
      block_idx_to_top_left_corner[i].resize(n_blocks_c);

      block_idx_to_shape.push_back({});
      block_idx_to_shape[i].resize(n_blocks_c);
    }

    std::pair<size_t, size_t> top_left_loc;
    size_t next_block_start_r, next_block_start_c;

    for (static_var<size_t> b_i = 0; b_i < n_blocks_r; b_i = b_i+1) {
      for (static_var<size_t> b_j = 0; b_j < n_blocks_c; b_j = b_j+1) {
        block_idx_to_top_left_corner[b_i][b_j] = 
            std::make_pair(row_partitions[b_i], col_partitions[b_j]);

        if (b_i == n_blocks_r-2)
          next_block_start_r = shape[0];
        else
          next_block_start_r = row_partitions[b_i+1];
        if (b_j == n_blocks_c-2)
          next_block_start_c = shape[1];
        else
          next_block_start_c = col_partitions[b_j+1];

        block_idx_to_shape[b_i][b_j] = 
            std::make_pair(next_block_start_r - row_partitions[b_i],
                next_block_start_c - col_partitions[b_j]
            );
      }
    }
  }

  bool is_block_set(size_t b_i, size_t b_j) const {
    auto block_idx = std::make_pair(b_i, b_j);
    if (block_idx_to_block.find(block_idx) == block_idx_to_block.end())
      return false;
    return true;
  }
  
  std::pair<size_t, size_t> get_block_idx(size_t i, size_t j) const {
    for (static_var<size_t> b_i = 0; b_i < n_blocks_r; b_i = b_i+1) {
      for (static_var<size_t> b_j = 0; b_j < n_blocks_c; b_j = b_j+1) {

        auto block_top_left = block_idx_to_top_left_corner[b_i][b_j];
        size_t block_top_left_r = block_top_left.first;
        size_t block_top_left_c = block_top_left.second;

        auto block_shape = block_idx_to_shape[b_i][b_j];
        size_t n_block_rows = block_shape.first;
        size_t n_block_cols = block_shape.second;

        if ((block_top_left_r <= i && i < block_top_left_r+n_block_rows) && \
            (block_top_left_c <= j && j < block_top_left_c+n_block_cols)) {
          return std::make_pair(b_i, b_j);
        }
      }
      assert(false && "we should've found some block");
      return std::make_pair(0, 0);
    }
  }

  std::pair<size_t, size_t> get_relative_idx_wrt_block(size_t i, size_t j) const {
    auto block_idx = get_block_idx(i, j);
    auto block_top_left = block_idx_to_top_left_corner[block_idx.first][block_idx.second];

    return std::make_pair(i - block_top_left.first, j - block_top_left.second);
  }

  void set_new_block(size_t b_i, size_t b_j, typename matrix_layout<Scalar>::Ptr block) {
    assert(!is_block_set(b_i, b_j) && "block already exists");

    block_idx_to_block[std::make_pair(b_i, b_j)] = block;
  }

  bool is_zero(size_t i, size_t j) const override {
    auto block_idx = get_block_idx(i, j);

    // unset blocks are assumed to be zero
    if (!is_block_set(block_idx.first, block_idx.second))
      return true;

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    return block_idx_to_block[block_idx]->is_zero(rel_idx.first, rel_idx.second);
  }

  bool is_constant(size_t i, size_t j) const override {
    auto block_idx = get_block_idx(i, j);

    // unset blocks are assumed to be zero
    if (!is_block_set(block_idx.first, block_idx.second))
      return true;

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    return block_idx_to_block[block_idx]->is_constant(rel_idx.first, rel_idx.second);
  }

  bool is_nonzero(size_t i, size_t j) const override {
    return !is_zero(i, j);
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    return !is_constant(i, j);
  }

  builder::builder get_entry(size_t i, size_t j) const {
    auto block_idx = get_block_idx(i, j);

    // unset blocks are assumed to be zero
    if (!is_block_set(block_idx.first, block_idx.second))
      return 0;

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    return block_idx_to_block[block_idx]->get_entry(rel_idx.first, rel_idx.second);
  }

  Scalar get_constant_entry(size_t i, size_t j) const {
    auto block_idx = get_block_idx(i, j);

    // unset blocks are assumed to be zero
    if (!is_block_set(block_idx.first, block_idx.second))
      return 0;

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    return block_idx_to_block[block_idx]->get_constant_entry(rel_idx.first, rel_idx.second);
  }

  void set_entry_to_constant(size_t i, size_t j, Scalar val) { 
    auto block_idx = get_block_idx(i, j);

    assert(!is_block_set(block_idx.first, block_idx.second) && 
        "must have inner matrix layout associated with block");

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    block_idx_to_block[block_idx]->set_entry_to_constant(rel_idx.first, rel_idx.second, val);
  }

  void set_entry_to_nonconstant(size_t i, size_t j, const builder::builder &entry) {
    auto block_idx = get_block_idx(i, j);

    assert(!is_block_set(block_idx.first, block_idx.second) && 
        "must have inner matrix layout associated with block");

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    block_idx_to_block[block_idx]->set_entry_to_nonconstant(rel_idx.first, rel_idx.second, entry);
  }

  void operator=(const blocked_layout_expr<Scalar> &rhs) {
    for (static_var<size_t> b_i = 0; b_i < n_blocks_r; b_i = b_i+1) {
      for (static_var<size_t> b_j = 0; b_j < n_blocks_c; b_j = b_j+1) {
        if (rhs.is_block_nonzero(b_i, b_j)) {
          assert(is_block_set(b_i, b_j) && "block must have some inner matrix layout associated with it");

          auto block_ptr = block_idx_to_block[std::make_pair(b_i, b_j)];

          assert(block_ptr->shape[0] == rhs.get_block_expr_shape(b_i, b_j)[0] && \
              block_ptr[b_i][b_j]->shape[1] == rhs.get_block_expr_shape(b_i, b_j)[1] &&
              "block shape of LHS and RHS must be same");

          *block_ptr = rhs.gen_block_expr(b_i, b_j);
        }
      }
    }
  }
};

template <typename Scalar>
struct blocked_layout_expr_leaf : public blocked_layout_expr<Scalar> {
  const struct blocked_layout<Scalar> &m_mat;

  std::vector<size_t> expr_shape;

  blocked_layout_expr_leaf(const struct blocked_layout<Scalar> &blocked_mat)
      : m_mat(blocked_mat) {
    expr_shape.push_back(m_mat.shape[0]);
    expr_shape.push_back(m_mat.shape[1]);
  }

  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  bool is_block_nonzero(size_t b_i, size_t b_j) const override {
    if (m_mat.is_block_set(b_i, b_j))
      return true;
    return false;
  }

  std::vector<size_t> get_block_expr_shape(size_t b_i, size_t b_j) const override {
    auto block_shape = m_mat.block_idx_to_shape[b_i][b_j];
    return {block_shape.first, block_shape.second};
  }

  const matrix_layout_expr<Scalar> &gen_block_expr(size_t b_i, size_t b_j) const override {
    // todo check null
    return m_mat.block_idx_to_block[b_i][b_j];
  }
};

template <typename Scalar>
struct blocked_layout_expr_mul : public blocked_layout_expr<Scalar> {
  const struct blocked_layout_expr<Scalar> &expr1;
  const struct blocked_layout_expr<Scalar> &expr2;

  std::vector<size_t> expr_shape;

  blocked_layout_expr_mul(const struct blocked_layout_expr<Scalar> &expr1, const struct blocked_layout_expr<Scalar> &expr2)
      : expr1(expr1), expr2(expr2) {
    std::vector<size_t> shape1, shape2;
    shape1 = expr1.get_expr_shape();
    shape2 = expr2.get_expr_shape();

    assert(shape1[1] == shape2[0] && "inner dim of matmul expr must match");

    expr_shape.push_back(shape1[0]);
    expr_shape.push_back(shape2[1]);
  }

  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  bool is_block_nonzero(size_t b_i, size_t b_j) const override {
    const size_t inner_dim = expr1.get_expr_shape()[1];
    for (static_var<size_t> b_k = 0; b_k < inner_dim; b_k = b_k + 1) {
      // when summing up products of inner_dim, if any one product is nonzero
      // then (i, j) is guaranteed to be nonzero.
      if (expr1.is_nonzero(b_i, b_k) && expr2.is_nonzero(b_k, b_j))
        return true;
    }
    return false;
  }

  std::vector<size_t> get_block_expr_shape(size_t b_i, size_t b_j) const override {
    // todo recurse shape
    return {expr1.get_block_expr_shape(b_i, block_shape.second};
  }

  const matrix_layout_expr<Scalar> &gen_mac_chain_recurse(size_t b_i, size_t b_j, size_t b_k) const {
    if (b_k == expr1.get_expr_shape()[1])
      return expr1.gen_block_expr(b_i, b_k) * expr2.gen_block_expr(b_k, b_j);
    else
      return expr1.gen_block_expr(b_i, b_k) * expr2.gen_block_expr(b_k, b_j) + gen_mac_chain_recurse(b_i, b_j, b_k+1);

  }

  const matrix_layout_expr<Scalar> &gen_block_expr(size_t b_i, size_t b_j) const override {
    return gen_mac_chain_recurse(b_i, b_j, 0);
  }
};

}

#endif
