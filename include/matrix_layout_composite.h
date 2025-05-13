#ifndef MATRIX_LAYOUT_COMPOSITE_H
#define MATRIX_LAYOUT_COMPOSITE_H
#include "matrix_layout.h"
#include <utility>
#include <vector>

using builder::dyn_var;
using builder::static_var;

namespace ctup {

template <typename Prim>
struct blocked_layout_expr : public zero_cst_status_checkable {
  using Scalar = inner_type_t<Prim>;

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

  virtual std::vector<size_t> get_expr_n_blocks_shape() const {
    trigger_abstract_method_assert();
    return std::vector<size_t>();
  }

  virtual bool is_block_set(size_t b_i, size_t b_j) const {
    trigger_abstract_method_assert();
    return false;
  }

  virtual bool is_block_unset(size_t b_i, size_t b_j) const {
    return !is_block_set(b_i, b_j);
  }

  virtual bool is_block_nonzero(size_t b_i, size_t b_j) const {
    trigger_abstract_method_assert();
    return false;
  }

  virtual bool is_block_zero(size_t b_i, size_t b_j) const {
    return !is_block_nonzero(b_i, b_j);
  }

  virtual const matrix_layout_expr<Prim> &gen_block_expr(size_t b_i, size_t b_j) const {
    trigger_abstract_method_assert();
    return *new matrix_layout_expr<Scalar>();
  }

  virtual std::vector<size_t> get_block_inner_shape(size_t b_i, size_t b_j) const {
    trigger_abstract_method_assert();
    return {};
  }
};

// fwd decl
template <typename Prim>
struct blocked_layout_expr_leaf;

// base class for composite matrix_layouts that hold
// other layouts within them.
template <typename Prim>
struct blocked_layout : public zero_cst_status_storable<Prim> {
  using Scalar = inner_type_t<Prim>;

  const size_t shape[2];

  std::map<std::pair<size_t, size_t>, typename matrix_layout<Prim>::Ptr> block_idx_to_block;

  // block_idx to top left corner of block
  // std::vector because each block guaranteed to have a top left corner
  std::vector<std::vector<std::pair<size_t, size_t>>> block_idx_to_top_left_corner;
  std::vector<std::vector<std::pair<size_t, size_t>>> block_idx_to_shape;
  size_t n_blocks_r;
  size_t n_blocks_c;

  blocked_layout(size_t _n_rows, size_t _n_cols) : 
      shape{_n_rows, _n_cols} {}

  size_t get_flattened_index(size_t i, size_t j) const {
    size_t flattened = i * shape[1] + j;
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

        if (b_i == n_blocks_r-1)
          next_block_start_r = shape[0];
        else
          next_block_start_r = row_partitions[b_i+1];

        if (b_j == n_blocks_c-1)
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

  bool is_block_nonzero(size_t b_i, size_t b_j) const {
    if (is_block_set(b_i, b_j)) {
      auto block_idx = std::make_pair(b_i, b_j);

      auto block_ptr = block_idx_to_block.at(block_idx);

      for (static_var<size_t> ii = 0; ii < block_ptr->get_n_rows(); ii=ii+1) {
        for (static_var<size_t> jj = 0; jj < block_ptr->get_n_cols(); jj=jj+1) {
          if (block_ptr->is_nonzero(ii, jj))
            return true;
        }
      }
      return false;
    }
    // if no matrix layout associated with block then guaranteed zero
    return false;
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
    }
    assert(false && "we should've found some block");
    return std::make_pair(0, 0);
  }

  std::pair<size_t, size_t> get_relative_idx_wrt_block(size_t i, size_t j) const {
    auto block_idx = get_block_idx(i, j);
    auto block_top_left = block_idx_to_top_left_corner[block_idx.first][block_idx.second];

    return std::make_pair(i - block_top_left.first, j - block_top_left.second);
  }

  void set_new_block(size_t b_i, size_t b_j, typename matrix_layout<Prim>::Ptr block) {
    assert(!is_block_set(b_i, b_j) && "block already exists");

    block_idx_to_block[std::make_pair(b_i, b_j)] = block;
  }

  bool is_zero(size_t i, size_t j) const override {
    auto block_idx = get_block_idx(i, j);

    // unset blocks are assumed to be zero
    if (!is_block_set(block_idx.first, block_idx.second))
      return true;

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    return block_idx_to_block.at(block_idx)->is_zero(rel_idx.first, rel_idx.second);
  }

  bool is_constant(size_t i, size_t j) const override {
    auto block_idx = get_block_idx(i, j);

    // unset blocks are assumed to be zero
    if (!is_block_set(block_idx.first, block_idx.second))
      return true;

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    return block_idx_to_block.at(block_idx)->is_constant(rel_idx.first, rel_idx.second);
  }

  bool is_nonzero(size_t i, size_t j) const override {
    return !is_zero(i, j);
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    return !is_constant(i, j);
  }

  builder::builder get_entry(size_t i, size_t j) const override {
    auto block_idx = get_block_idx(i, j);

    // unset blocks are assumed to be zero
    if (!is_block_set(block_idx.first, block_idx.second))
      return 0;

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    return block_idx_to_block.at(block_idx)->get_entry(rel_idx.first, rel_idx.second);
  }

  Scalar get_constant_entry(size_t i, size_t j) const override {
    auto block_idx = get_block_idx(i, j);

    // unset blocks are assumed to be zero
    if (!is_block_set(block_idx.first, block_idx.second))
      return 0;

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    return block_idx_to_block.at(block_idx)->get_constant_entry(rel_idx.first, rel_idx.second);
  }

  void set_entry_to_constant(size_t i, size_t j, Scalar val) override {
    auto block_idx = get_block_idx(i, j);

    bool found_block = is_block_set(block_idx.first, block_idx.second);

    // future todo: have some way to access block of zeros because no matrix_layout
    // associated with blocks of zeros currently
    if (!found_block && val < std::abs(1e-5))
      return;

    assert(is_block_set(block_idx.first, block_idx.second) && 
        "must have inner matrix layout associated with block");

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    block_idx_to_block[block_idx]->set_entry_to_constant(rel_idx.first, rel_idx.second, val);
  }

  void set_entry_to_nonconstant(size_t i, size_t j, const builder::builder &entry) override {
    auto block_idx = get_block_idx(i, j);

    assert(is_block_set(block_idx.first, block_idx.second) && 
        "must have inner matrix layout associated with block");

    auto rel_idx = get_relative_idx_wrt_block(i, j);
    block_idx_to_block[block_idx]->set_entry_to_nonconstant(rel_idx.first, rel_idx.second, entry);
  }

  dyn_var<EigenMatrix<Prim>> denseify() const override {
    dyn_var<EigenMatrix<Prim>> mat;
    mat.set_matrix_fixed_size(shape[0], shape[1]);
    for (static_var<size_t> i = 0; i < shape[0]; i = i+1) {
      for (static_var<size_t> j = 0; j < shape[1]; j = j+1) {
        mat.coeffRef(i, j) = get_entry(i, j);
      }
    }
    return mat;
  }

  void set_matrix(const dyn_var<EigenMatrix<Scalar>> &mat) override {
    // this func will usually not work for blocked_layout because
    // blocked_layout usually has blocks zero'd out / unavailable at compile-time
    // if the blocked_layout is fully element-wise sparse only then will
    // this func work
    for (static_var<size_t> i = 0; i < shape[0]; i = i+1) {
      for (static_var<size_t> j = 0; j < shape[1]; j = j+1) {
        set_entry_to_nonconstant(i, j, const_cast<dyn_var<EigenMatrix<Scalar>>&>(mat).coeffRef(i, j));
      }
    }
  }

  void set_zero() override {
    for (static_var<size_t> i = 0; i < shape[0]; i = i+1) {
      for (static_var<size_t> j = 0; j < shape[1]; j = j+1) {
        set_entry_to_constant(i, j, 0);
      }
    }
  }

  bool is_batching_enabled() const override {
    return false;
  }

  std::vector<size_t> get_expr_shape() const override {
    return {shape[0], shape[1]};
  }

  void operator=(const blocked_layout_expr<Prim> &rhs) {
    for (static_var<size_t> b_i = 0; b_i < n_blocks_r; b_i = b_i+1) {
      for (static_var<size_t> b_j = 0; b_j < n_blocks_c; b_j = b_j+1) {
        if (rhs.is_block_nonzero(b_i, b_j)) {
          // future todo: LHS should be able to propagate nonzero block from RHS to LHS
          // we don't do this currently
          assert(is_block_set(b_i, b_j) && "LHS block must have some inner matrix layout associated with it");

          auto block_ptr = block_idx_to_block.at(std::make_pair(b_i, b_j));

          // future todo: RHS should be able to set block shapes of LHS in the future
          // LHS in the future should be any generic matrix that can have blocks
          // propagated from RHS
          assert(block_ptr->shape[0] == rhs.get_block_inner_shape(b_i, b_j)[0] && \
              block_ptr->shape[1] == rhs.get_block_inner_shape(b_i, b_j)[1] &&
              "block shape of LHS and RHS must be same");

          *block_ptr = rhs.gen_block_expr(b_i, b_j);
        }
        else {
          if (is_block_set(b_i, b_j)) {
            // ... but corresp. LHS block has matrix layout associated with it
            // ... block will need its individual entries assigned to zero then
            auto block_ptr = block_idx_to_block.at(std::make_pair(b_i, b_j));
            block_ptr->set_zero();
          }
          // else do nothing, LHS block already zero
        }
      }
    }
  }

  void operator=(const blocked_layout<Prim> &mat) {
    *this = blocked_layout_expr_leaf<Prim>(mat);
  }
};

template <typename Prim>
struct blocked_layout_expr_leaf : public blocked_layout_expr<Prim> {
  const struct blocked_layout<Prim> &m_mat;

  std::vector<size_t> expr_shape;
  std::vector<size_t> expr_block_shape;

  blocked_layout_expr_leaf(const struct blocked_layout<Prim> &blocked_mat)
      : m_mat(blocked_mat) {
    expr_shape.push_back(m_mat.shape[0]);
    expr_shape.push_back(m_mat.shape[1]);

    expr_block_shape.push_back(m_mat.n_blocks_r);
    expr_block_shape.push_back(m_mat.n_blocks_c);
  }

  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  std::vector<size_t> get_expr_n_blocks_shape() const override {
    return expr_block_shape;
  }

  bool is_block_set(size_t b_i, size_t b_j) const override {
    return m_mat.is_block_set(b_i, b_j);
  }

  bool is_block_nonzero(size_t b_i, size_t b_j) const override {
    return m_mat.is_block_nonzero(b_i, b_j);
  }

  std::vector<size_t> get_block_inner_shape(size_t b_i, size_t b_j) const override {
    auto block_shape = m_mat.block_idx_to_shape[b_i][b_j];
    return {block_shape.first, block_shape.second};
  }

  const matrix_layout_expr<Prim> &gen_block_expr(size_t b_i, size_t b_j) const override {
    // future todo: this should return a matrix_layout of zeros
    assert(m_mat.is_block_set(b_i, b_j) && 
            "(b_i, b_j) has no inner matrix_layout associated with it because it is zero, \
            check if block is nonzero before calling gen_block_expr");
    return *new matrix_layout_expr_leaf<Prim>(*m_mat.block_idx_to_block.at(std::make_pair(b_i, b_j)));
  }
};

template <typename PRet, typename P1, typename P2>
struct blocked_layout_expr_mul : public blocked_layout_expr<PRet> {
  const struct blocked_layout_expr<P1> &expr1;
  const struct blocked_layout_expr<P2> &expr2;

  std::vector<size_t> expr_shape;
  std::vector<size_t> expr_block_shape;

  blocked_layout_expr_mul(const struct blocked_layout_expr<P1> &expr1, const struct blocked_layout_expr<P2> &expr2)
      : expr1(expr1), expr2(expr2) {
    std::vector<size_t> shape1, shape2;
    shape1 = expr1.get_expr_shape();
    shape2 = expr2.get_expr_shape();

    assert(shape1[1] == shape2[0] && "inner dim of matmul expr must match");

    expr_shape.push_back(shape1[0]);
    expr_shape.push_back(shape2[1]);

    shape1 = expr1.get_expr_n_blocks_shape();
    shape2 = expr2.get_expr_n_blocks_shape();

    assert(shape1[1] == shape2[0] && "inner number of blocks of matmul expr must match");

    expr_block_shape.push_back(shape1[0]);
    expr_block_shape.push_back(shape2[1]);
  }

  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  std::vector<size_t> get_expr_n_blocks_shape() const override {
    return expr_block_shape;
  }

  bool is_block_set(size_t b_i, size_t b_j) const override {
    const size_t inner_dim = expr1.get_expr_n_blocks_shape()[1];
    for (static_var<size_t> b_k = 0; b_k < inner_dim; b_k = b_k + 1) {
      // when summing up products of inner_dim, if any one product is set
      // then block (b_i, b_j) is guaranteed to be set.
      if (expr1.is_block_set(b_i, b_k) && expr2.is_block_set(b_k, b_j))
        return true;
    }
    return false;
  }

  bool is_block_nonzero(size_t b_i, size_t b_j) const override {
    const size_t inner_dim = expr1.get_expr_n_blocks_shape()[1];
    for (static_var<size_t> b_k = 0; b_k < inner_dim; b_k = b_k + 1) {
      // when summing up products of inner_dim, if any one product is nonzero
      // then (i, j) is guaranteed to be nonzero.
      if (expr1.is_block_nonzero(b_i, b_k) && expr2.is_block_nonzero(b_k, b_j))
        return true;
    }
    return false;
  }

  std::vector<size_t> get_block_inner_shape(size_t b_i, size_t b_j) const override {
    std::vector<size_t> inner_shape1;
    std::vector<size_t> inner_shape2;

    // * operator in matrix_layout matmul should trigger an assert of its own if dims
    // are mismatched for block * block, so technically this error check is redundant
    // kept this check to make bug catching easier.
    const size_t inner_dim = expr1.get_expr_n_blocks_shape()[1];
    for (static_var<size_t> b_k = 0; b_k < inner_dim; b_k = b_k + 1) {
      inner_shape1 = expr1.get_block_inner_shape(b_i, b_k);
      inner_shape2 = expr2.get_block_inner_shape(b_k, b_j);
      assert(inner_shape1[1] == inner_shape2[0] && "inner dims within each block in MAC chain must match, bad blocked partitions");
    }
    return {inner_shape1[0], inner_shape2[1]};
  }

  const matrix_layout_expr<PRet> &gen_mac_chain_recurse(size_t b_i, size_t b_j, size_t b_k) const {
    // check block matmul validity
    std::vector<size_t> inner_shape1 = expr1.get_block_inner_shape(b_i, b_k);
    std::vector<size_t> inner_shape2 = expr2.get_block_inner_shape(b_k, b_j);
    assert(inner_shape1[1] == inner_shape2[0] && "sub block inner dims must match, bad blocked partitions");

    const size_t OOB = expr1.get_expr_n_blocks_shape()[1];

    static_var<size_t> next_nz;
    for (next_nz = b_k+1; next_nz < OOB; next_nz = next_nz+1) {
      if (expr1.is_block_nonzero(b_i, next_nz) && expr2.is_block_nonzero(next_nz, b_j)) {
        break;
      }
    }

    if (expr1.is_block_zero(b_i, b_k) || expr2.is_block_zero(b_k, b_j)) {
      if (next_nz == OOB)
        assert(false && "mul gen_block_expr was called for an empty expr, should have checked if result is zero prior");
      else
        return gen_mac_chain_recurse(b_i, b_j, next_nz);
    }
    else {
      if (next_nz == OOB)
        return expr1.gen_block_expr(b_i, b_k) * expr2.gen_block_expr(b_k, b_j);
      else
        return expr1.gen_block_expr(b_i, b_k) * expr2.gen_block_expr(b_k, b_j) + gen_mac_chain_recurse(b_i, b_j, next_nz);
    }
  }

  const matrix_layout_expr<PRet> &gen_block_expr(size_t b_i, size_t b_j) const override {
    return gen_mac_chain_recurse(b_i, b_j, 0);
  }
};

}

#endif
