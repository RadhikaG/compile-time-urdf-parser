#ifndef MATRIX_LAYOUT_H
#define MATRIX_LAYOUT_H
#include "backend.h"
#include "builder/builder_base.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "builder/static_var.h"
#include "builder/lib/utils.h"
#include <memory>
#include <unordered_map>
#include <vector>

#include <assert.h>
#include <sstream>

#define SIMD_WIDTH 8

using builder::dyn_var;
using builder::static_var;

namespace ctup {

struct zero_cst_status_checkable {
  virtual bool is_nonzero(size_t i, size_t j) const = 0;
  virtual bool is_zero(size_t i, size_t j) const = 0;
  virtual bool is_nonconstant(size_t i, size_t j) const = 0;
  virtual bool is_constant(size_t i, size_t j) const = 0;
  virtual ~zero_cst_status_checkable() = default;
};

template <typename Scalar>
struct zero_cst_status_storable : public zero_cst_status_checkable {
  virtual builder::builder get_entry(size_t i, size_t j) const = 0;
  virtual Scalar get_constant_entry(size_t i, size_t j) const = 0;
  virtual dyn_var<EigenMatrix<Scalar>> denseify() const = 0;
  virtual void set_matrix(const dyn_var<EigenMatrix<Scalar>> &mat) = 0;
  virtual void set_entry_to_constant(size_t i, size_t j, Scalar val) = 0;
  virtual void set_entry_to_nonconstant(size_t i, size_t j, const builder::builder &entry) = 0;
  virtual void set_zero() = 0;
  virtual std::vector<size_t> get_expr_shape() const;
  virtual bool is_batching_enabled() const;
  virtual ~zero_cst_status_storable() = default;
};

template <typename Scalar>
struct inner_type { 
  using type = Scalar;
};

template <typename Scalar, int _Rows, int _Cols>
struct inner_type<ctup::EigenMatrix<Scalar, _Rows, _Cols>> {
  using type = Scalar;
};

template <typename Scalar>
struct is_Matrix : std::false_type {};

template <typename Scalar ,int _Rows, int _Cols>
struct is_Matrix<ctup::EigenMatrix<Scalar, _Rows, _Cols>> : std::true_type {};

template <typename Prim>
inline constexpr bool is_Matrix_v = is_Matrix<Prim>::value;

template <typename Prim>
using inner_type_t = typename inner_type<Prim>::type;

enum iteration_order {
  DENSE,
  SPARSE,
  TILE,
};

enum backend_mem_representation {
  UNROLLED,
  FLATTENED,
  EIGENMATRIX,
};

enum compression {
  UNCOMPRESSED,
  COMPRESSED,
  SINGLETON,
};

enum var_type {
  ZERO,
  NONZERO_CST,
  NONCST,
};

template <typename Prim>
struct sparse_entry {
  dyn_var<Prim> dyn_entry;
  static_var<inner_type_t<Prim>> static_entry; // fill value
  sparse_entry() {}
  sparse_entry(const dyn_var<Prim> &de) : dyn_entry(de) {}
  sparse_entry(Prim &se) : static_entry(se) {}
};

struct in_memory_sparsity_repr : public zero_cst_status_checkable {
  typedef std::shared_ptr<in_memory_sparsity_repr> Ptr;

  size_t n_rows;
  size_t n_cols;

  in_memory_sparsity_repr(size_t r, size_t c) : n_rows(r), n_cols(c) {}

  void trigger_abstract_method_assert() const {
    assert(false && "abstract in_memory_sparsity_repr method call");
  }

  std::string append_idx_error_msg(std::string error_msg, size_t i, size_t j) const {
    auto err_stream = std::stringstream{};
    err_stream << error_msg << ": "
               << "(" << i << "," << j << ")";
    return err_stream.str();
  }

  size_t get_flattened_index(size_t i, size_t j) const {
    size_t  flattened = i * n_cols + j;

    std::string error_msg = append_idx_error_msg("index out of bounds", i, j);
    assert(flattened < n_rows * n_cols && error_msg.c_str());

    return flattened;
  }

  virtual size_t get_dense_to_sparse_idx(size_t i, size_t j) const {
    trigger_abstract_method_assert();
    return -1;
  }

  virtual bool is_nonzero(size_t i, size_t j) const override {
    trigger_abstract_method_assert();
    return false;
  }

  virtual bool is_zero(size_t i, size_t j) const override { return !is_nonzero(i, j); };

  virtual bool is_nonconstant(size_t i, size_t j) const override {
    trigger_abstract_method_assert();
    return false;
  }

  virtual bool is_constant(size_t i, size_t j) const override { return !is_nonconstant(i, j); };

  virtual void mark_constant(size_t i, size_t j, bool is_nonzero) { trigger_abstract_method_assert(); }
  virtual void mark_nonconstant(size_t i, size_t j) { trigger_abstract_method_assert(); }
};

struct uncompressed_repr : public in_memory_sparsity_repr {
  static_var<int[]> nonzero_status_arr;

  uncompressed_repr(size_t r, size_t c) : in_memory_sparsity_repr(r, c) {
    nonzero_status_arr.resize(n_rows * n_cols);

    // default conservative value: NONCST
    for (static_var<size_t> i = 0; i < n_rows * n_cols; i = i+1) {
      nonzero_status_arr[i] = NONCST;
    }
  }

  size_t get_dense_to_sparse_idx(size_t i, size_t j) const override {
    return get_flattened_index(i, j);
  }

  bool is_nonzero(size_t i, size_t j) const override {
    return !(nonzero_status_arr[get_flattened_index(i, j)] == ZERO);
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    return nonzero_status_arr[get_flattened_index(i, j)] == NONCST;
  }

  void mark_constant(size_t i, size_t j, bool is_nonzero) override {
    if (is_nonzero)
      nonzero_status_arr[get_flattened_index(i, j)] = NONZERO_CST;
    else
      nonzero_status_arr[get_flattened_index(i, j)] = ZERO;
  }

  void mark_nonconstant(size_t i, size_t j) override {
    nonzero_status_arr[get_flattened_index(i, j)] = NONCST;
  }
};

struct coo_repr : public in_memory_sparsity_repr {
  // use coo when you want compile-time index data structure size and
  // run-time nonconstant data structure to scale with O(nnz)

  // many-to-one relationship from dense (logical) to sparse (physical) entries
  // many dense entries may point to the same sparse entry
  // this allows for repeated values
  std::unordered_map<size_t, size_t> dense_to_sparse_nonzero_cst_idx;
  std::unordered_map<size_t, size_t> dense_to_sparse_noncst_idx;

  // marks physical entries that were previously assigned to some value,
  // that are now open to be overwritten (dirty).
  // this is because we do not delete any physical entries, just realloc them.
  std::vector<size_t> dirty_rt_indices_nonzero_csts;
  std::vector<size_t> dirty_rt_indices_noncsts;

  coo_repr(size_t r, size_t c) : in_memory_sparsity_repr(r, c) {
    // initializing nonzeros and nonconstants as empty
    // default value for all entries: ZERO
  }

  // this function only valid for nonzero indicies
  // errors out for zero'd entries
  size_t get_dense_to_sparse_idx(size_t i, size_t j) const override {
    size_t flattened_idx = get_flattened_index(i, j);

    std::string error_msg = append_idx_error_msg("no sparse entry found for", i, j);
    assert(is_nonzero(i, j) && error_msg.c_str());

    if (is_constant(i, j)) {
      return dense_to_sparse_nonzero_cst_idx.find(flattened_idx)->second;
    }
    else {
      assert(is_nonconstant(i, j) && error_msg.c_str());
      return dense_to_sparse_noncst_idx.find(flattened_idx)->second;
    }
  }

  bool is_marked_nonzero_cst(size_t i, size_t j) const {
    size_t flattened_idx = get_flattened_index(i, j);

    if (dense_to_sparse_nonzero_cst_idx.find(flattened_idx) != dense_to_sparse_nonzero_cst_idx.end())
      return true;
    return false;
  }

  bool is_marked_noncst(size_t i, size_t j) const {
    size_t flattened_idx = get_flattened_index(i, j);

    if (dense_to_sparse_noncst_idx.find(flattened_idx) != dense_to_sparse_noncst_idx.end())
      return true;
    return false;
  }

  void mark_constant(size_t i, size_t j, bool is_nonzero) override {
    size_t flattened_idx = get_flattened_index(i, j);

    if (is_marked_noncst(i, j)) {
      // remove the nonconstant index mapping
      // we do not remove the physical entry though
      // we simply mark it as dirty then reallocate the dirty location
      // elsewhere
      size_t dirty_noncst_idx = dense_to_sparse_noncst_idx[flattened_idx];
      dense_to_sparse_noncst_idx.erase(flattened_idx);
      dirty_rt_indices_noncsts.push_back(dirty_noncst_idx);
    }

    if (!is_nonzero) {
      // want to mark entry as zero
      if (is_marked_nonzero_cst(i, j)) {
        // remove the nonzero cst index mapping
        // we do not remove the physical entry though
        // we simply mark it as dirty then reallocate the dirty location
        // elsewhere
        size_t dirty_nonzero_cst_idx = dense_to_sparse_nonzero_cst_idx[flattened_idx];
        dense_to_sparse_nonzero_cst_idx.erase(flattened_idx);
        dirty_rt_indices_nonzero_csts.push_back(dirty_nonzero_cst_idx);
      }
      // zeros are not added to either nonzero_csts or noncsts
      return;
    }

    // nonzero and already marked nonzero_cst then do nothing
    if (is_marked_nonzero_cst(i, j))
      return;

    // is_nonzero cst and (not previously marked nonzero cst)

    // now we add an index mapping going from (i,j) to some new sparse index
    // inside some list of nonzero_csts
    if (!dirty_rt_indices_nonzero_csts.empty()) {
      // some index in nonzero_csts is dirty
      // let's assign (i,j) to this dirty index
      dense_to_sparse_nonzero_cst_idx[flattened_idx] = dirty_rt_indices_nonzero_csts[0];
      // removing 0th element
      dirty_rt_indices_nonzero_csts.erase(dirty_rt_indices_nonzero_csts.begin());
    }
    else {
      // if no dirty indices available then nonzero_cst must be packed
      // just pick largest available index
      dense_to_sparse_nonzero_cst_idx[flattened_idx] = dense_to_sparse_nonzero_cst_idx.size();
    }
  }

  void mark_nonconstant(size_t i, size_t j) override {
    size_t flattened_idx = get_flattened_index(i, j);

    // if already marked noncst then do nothing
    if (is_marked_noncst(i, j))
      return;

    if (is_marked_nonzero_cst(i, j)) {
      // remove the nonzero cst index mapping
      // we do not remove the physical entry though
      // we simply mark it as dirty then reallocate the dirty location
      // elsewhere
      size_t dirty_nonzero_cst_idx = dense_to_sparse_nonzero_cst_idx[flattened_idx];
      dense_to_sparse_nonzero_cst_idx.erase(flattened_idx);
      dirty_rt_indices_nonzero_csts.push_back(dirty_nonzero_cst_idx);
    }

    // now we add an index mapping going from (i,j) to some new sparse index
    // inside some list of noncsts
    if (!dirty_rt_indices_noncsts.empty()) {
      // some index in noncsts is dirty
      // let's assign (i,j) to this dirty index
      dense_to_sparse_noncst_idx[flattened_idx] = dirty_rt_indices_noncsts[0];
      // removing 0th element
      dirty_rt_indices_noncsts.erase(dirty_rt_indices_noncsts.begin());
    }
    else {
      // if no dirty indices available then noncst must be packed
      // just pick largest available index
      dense_to_sparse_noncst_idx[flattened_idx] = dense_to_sparse_noncst_idx.size();
    }
  }

  bool is_nonzero(size_t i, size_t j) const override {
    if (is_marked_noncst(i, j))
      return true;
    else if (is_marked_nonzero_cst(i, j))
      return true;
    else
      return false;
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    if (is_marked_noncst(i, j))
      return true;
    else
      return false;
  }
};

struct singleton_repr : public in_memory_sparsity_repr {
  static_var<size_t> singleton_idx;
  static_var<size_t> singleton_status;

  singleton_repr(size_t r, size_t c) : in_memory_sparsity_repr(r, c) {
    singleton_status = ZERO;
  }

  size_t get_dense_to_sparse_idx(size_t i, size_t j) const override {
    assert(singleton_idx == get_flattened_index(i, j) && "non singleton element is never stored");
    // only single element stored so returns index 0
    return 0;
  }

  bool is_nonzero(size_t i, size_t j) const override {
    if (singleton_idx != get_flattened_index(i, j))
      return false;
    else
      return !(singleton_status == ZERO);
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    if (singleton_idx != get_flattened_index(i, j))
      return false;
    else
      return singleton_status == NONCST;
  }

  // mark_constant and mark_nonconstant only treat the last (i,j) they were run for,
  // as the singleton element.

  void mark_constant(size_t i, size_t j, bool is_nonzero) override {
    singleton_idx = get_flattened_index(i, j);
    if (is_nonzero)
      singleton_status = NONZERO_CST;
    else
      singleton_status = ZERO;
  }

  void mark_nonconstant(size_t i, size_t j) override {
    singleton_idx = get_flattened_index(i, j);
    singleton_status = NONCST;
  }
};

template <typename Scalar>
struct matrix_layout_expr : public zero_cst_status_checkable {
  void trigger_abstract_method_assert() const {
    assert(false && "abstract matrix_layout_expr method call");
  }

  // func returns builder::builder when T in dyn_var<T> differs at runtime
  // dyn_var<T> might be of type Prim or type Scalar, so we want to support both.
  // enables dynamic inheritance
  virtual const builder::builder gen_entry_at(size_t i, size_t j) const {
    trigger_abstract_method_assert();
    return dyn_var<Scalar>();
  }

  virtual Scalar gen_constant_entry_at(size_t i, size_t j) const {
    trigger_abstract_method_assert();
    return -1;
  }

  virtual const dyn_var<EigenMatrix<Scalar>> gen_dyn_matrix() const {
    trigger_abstract_method_assert();
    return dyn_var<EigenMatrix<Scalar>>();
  }

  virtual bool is_batched(size_t i, size_t j) const {
    trigger_abstract_method_assert();
    return false;
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
};

// fwd decl
template <typename Scalar>
struct matrix_layout_expr_leaf;

template <typename Scalar>
struct storage : public zero_cst_status_storable<Scalar> {
  typedef std::shared_ptr<storage<Scalar>> Ptr;

  size_t n_rows;
  size_t n_cols;

  storage(size_t r, size_t c) : n_rows(r), n_cols(c) {}

  std::string append_idx_error_msg(std::string error_msg, size_t i, size_t j) const {
    auto err_stream = std::stringstream{};
    err_stream << error_msg << ": "
               << "(" << i << "," << j << ")";
    return err_stream.str();
  }

  size_t get_flattened_index(size_t i, size_t j) const {
    size_t  flattened = i * n_cols + j;

    std::string error_msg = append_idx_error_msg("index out of bounds", i, j);
    assert(flattened < n_rows * n_cols && error_msg.c_str());

    return flattened;
  }

  void trigger_abstract_method_assert() const {
    assert(false && "abstract storage method call");
  }

  virtual dyn_var<EigenMatrix<Scalar>> denseify() const override { 
    trigger_abstract_method_assert();
    return dyn_var<EigenMatrix<Scalar>>();
  }

  virtual Scalar get_constant_entry(size_t i, size_t j) const override {
    trigger_abstract_method_assert();
    return -1;
  }

  virtual builder::builder get_entry(size_t i, size_t j) const override {
    trigger_abstract_method_assert();
    return -1;
  }

  virtual bool is_zero(size_t i, size_t j) const override {
    trigger_abstract_method_assert();
    return false;
  }

  virtual bool is_nonzero(size_t i, size_t j) const override {
    return !is_zero(i, j);
  }

  virtual bool is_constant(size_t i, size_t j) const override {
    trigger_abstract_method_assert();
    return false;
  }

  virtual bool is_nonconstant(size_t i, size_t j) const override {
    return !is_constant(i, j);
  }

  virtual void set_matrix(const dyn_var<EigenMatrix<Scalar>> &mat) override { trigger_abstract_method_assert(); }
  virtual void set_entry_to_constant(size_t i, size_t j, Scalar val) override { trigger_abstract_method_assert(); }
  virtual void set_entry_to_nonconstant(size_t i, size_t j, const builder::builder &entry) override { trigger_abstract_method_assert(); }

  virtual void set_zero() override {
    for (static_var<size_t> i = 0; i < n_rows; i = i+1) {
      for (static_var<size_t> j = 0; j < n_cols; j = j+1) {
        set_entry_to_constant(i, j, 0);
      }
    }
  }

  bool is_batching_enabled() const override {
    return false;
  }

  std::vector<size_t> get_expr_shape() const override {
    return {n_rows, n_cols};
  }
};

template <typename Prim>
struct unrolled_storage : public storage<inner_type_t<Prim>> {
  using Scalar = inner_type_t<Prim>;
  using storage<Scalar>::n_rows;
  using storage<Scalar>::n_cols;

  std::vector<sparse_entry<Prim>> soup;
  uncompressed_repr sparsity_tracker;

  unrolled_storage(size_t r, size_t c) : 
      storage<Scalar>(r, c), sparsity_tracker(r, c) {
    soup.resize(r * c);
  }

  dyn_var<EigenMatrix<Scalar>> denseify() const override {
    // needs to change for batched impl
    dyn_var<EigenMatrix<Scalar>> mat;
    mat.set_matrix_fixed_size(n_rows, n_cols);
    for (static_var<size_t> i = 0; i < n_rows; i = i+1) {
      for (static_var<size_t> j = 0; j < n_cols; j = j+1) {
        mat.coeffRef(i, j) = get_entry(i, j);
      }
    }

    return mat;
  }

  Scalar get_constant_entry(size_t i, size_t j) const override {
    size_t flattened = storage<Scalar>::get_flattened_index(i, j);

    std::string error_msg = storage<Scalar>::append_idx_error_msg("is not constant", i, j);
    assert(is_constant(i, j) && error_msg.c_str());

    return soup[flattened].static_entry;
  }

  builder::builder get_entry(size_t i, size_t j) const override {
    size_t flattened = storage<Scalar>::get_flattened_index(i, j);
    if (is_constant(i, j))
      return soup[flattened].static_entry;
    else
      return soup[flattened].dyn_entry;
  }

  bool is_zero(size_t i, size_t j) const override {
    return sparsity_tracker.is_zero(i, j);
  }

  bool is_constant(size_t i, size_t j) const override {
    return sparsity_tracker.is_constant(i, j);
  }

  void set_entry_to_constant(size_t i, size_t j, Scalar val) override { 
    size_t flattened = storage<Scalar>::get_flattened_index(i, j);

    // later add simplify_zero flag
    if (std::abs(val) < 1e-5) {
      soup[flattened].static_entry = 0;
      sparsity_tracker.mark_constant(i, j, 0);
    }
    else {
      soup[flattened].static_entry = val;
      sparsity_tracker.mark_constant(i, j, 1);
    }
  }

  void set_entry_to_nonconstant(size_t i, size_t j, const builder::builder &entry) override {
    size_t flattened = storage<Scalar>::get_flattened_index(i, j);
    soup[flattened].dyn_entry = entry;
    sparsity_tracker.mark_nonconstant(i, j);
  }

  void set_matrix(const dyn_var<EigenMatrix<Scalar>> &mat) override {
    for (static_var<size_t> i = 0; i < n_rows; i = i+1) {
      for (static_var<size_t> j = 0; j < n_cols; j = j+1) {
        set_entry_to_nonconstant(i, j, const_cast<dyn_var<EigenMatrix<Scalar>>&>(mat).coeffRef(i, j));
      }
    }
  }
};

template <typename Prim>
struct flattened_storage : public storage<inner_type_t<Prim>> {
  using Scalar = inner_type_t<Prim>;
  using storage<Scalar>::n_rows;
  using storage<Scalar>::n_cols;

  compression compress;
  in_memory_sparsity_repr::Ptr sparsity_tracker;

  // nonzero constants tracked within compiler
  static_var<Scalar[]> nonzero_csts;
  // backend emission of NONCST ops on m_array
  dyn_var<Prim[]> m_array;

  flattened_storage(compression comp, size_t r, size_t c) : 
      storage<Scalar>(r, c), compress(comp) {
    if (compress == UNCOMPRESSED)
      sparsity_tracker = std::make_shared<uncompressed_repr>(r, c);
    else if (compress == COMPRESSED)
      sparsity_tracker = std::make_shared<coo_repr>(r, c);
    else if (compress == SINGLETON)
      sparsity_tracker = std::make_shared<singleton_repr>(r, c);

    if (compress == SINGLETON) {
      // todo allow unrolled storage to support SINGLETON
      nonzero_csts.resize(1);
      resize_arr(m_array, 1);
    }
    else {
      nonzero_csts.resize(r * c);
      resize_arr(m_array, r * c);
    }
  }

  dyn_var<EigenMatrix<Scalar>> denseify() const override {
    // needs to change for batched impl
    dyn_var<EigenMatrix<Scalar>> mat;
    mat.set_matrix_fixed_size(n_rows, n_cols);
    for (static_var<size_t> i = 0; i < n_rows; i = i+1) {
      for (static_var<size_t> j = 0; j < n_cols; j = j+1) {
        mat.coeffRef(i, j) = get_entry(i, j);
      }
    }

    return mat;
  }

  Scalar get_constant_entry(size_t i, size_t j) const override {
    std::string error_msg = storage<Scalar>::append_idx_error_msg("is not constant", i, j);
    assert(is_constant(i, j) && error_msg.c_str());

    if (is_zero(i, j))
      return 0;

    size_t sparse_idx = sparsity_tracker->get_dense_to_sparse_idx(i, j);
    return nonzero_csts[sparse_idx];
  }

  builder::builder get_entry(size_t i, size_t j) const override {
    if (is_zero(i, j))
      return 0;

    size_t sparse_idx = sparsity_tracker->get_dense_to_sparse_idx(i, j);
    if (is_constant(i, j))
      return nonzero_csts[sparse_idx];
    else
      return const_cast<dyn_var<Prim[]>&>(m_array)[sparse_idx];
  }

  bool is_zero(size_t i, size_t j) const override {
    return sparsity_tracker->is_zero(i, j);
  }

  bool is_constant(size_t i, size_t j) const override {
    return sparsity_tracker->is_constant(i, j);
  }

  void set_entry_to_constant(size_t i, size_t j, Scalar val) override { 
    if (std::abs(val) < 1e-5) {
      sparsity_tracker->mark_constant(i, j, 0);
    }
    else {
      sparsity_tracker->mark_constant(i, j, 1);
      static_var<size_t> sparse_idx = sparsity_tracker->get_dense_to_sparse_idx(i, j);
      nonzero_csts[sparse_idx] = val;
    }
  }

  void set_entry_to_nonconstant(size_t i, size_t j, const builder::builder &entry) override {
    sparsity_tracker->mark_nonconstant(i, j);
    static_var<size_t> sparse_idx = sparsity_tracker->get_dense_to_sparse_idx(i, j);
    m_array[sparse_idx] = entry;
  }

  void set_matrix(const dyn_var<EigenMatrix<Scalar>> &mat) override {
    for (static_var<size_t> i = 0; i < n_rows; i = i+1) {
      for (static_var<size_t> j = 0; j < n_cols; j = j+1) {
        set_entry_to_nonconstant(i, j, const_cast<dyn_var<EigenMatrix<Scalar>>&>(mat).coeffRef(i, j));
      }
    }
  }
};

template <typename Scalar>
struct eigen_matrix_storage : public storage<Scalar> {
  dyn_var<EigenMatrix<Scalar>> m_matrix;
  // no sparsity tracking for eigen_matrix_storage for now

  eigen_matrix_storage(size_t r, size_t c) : 
      storage<Scalar>(r, c) {
    m_matrix.set_matrix_fixed_size(r, c);
  }

  eigen_matrix_storage(const dyn_var<EigenMatrix<Scalar>> &v, size_t r, size_t c) : 
      storage<Scalar>(r, c), m_matrix(v) {
  }

  dyn_var<EigenMatrix<Scalar>> denseify() const override {
    return m_matrix;
  }

  Scalar get_constant_entry(size_t i, size_t j) const override {
    assert(false && "no sparsity tracking for eigen_matrix_storage");
  }

  builder::builder get_entry(size_t i, size_t j) const override {
    return const_cast<dyn_var<EigenMatrix<Scalar>>&>(m_matrix).coeffRef(i, j);
  }

  bool is_zero(size_t i, size_t j) const override {
    return false;
  }

  bool is_constant(size_t i, size_t j) const override {
    return false;
  }

  void set_matrix(const dyn_var<EigenMatrix<Scalar>> &mat) override {
    m_matrix = mat;
  }

  void set_entry_to_constant(size_t i, size_t j, Scalar val) override { 
    m_matrix.coeffRef(i, j) = val;
  }

  void set_entry_to_nonconstant(size_t i, size_t j, const builder::builder &entry) override {
    m_matrix.coeffRef(i, j) = entry;
  }

  void set_zero() override {
    m_matrix.setZero();
  }
};

template <typename Scalar>
struct matrix_layout : public zero_cst_status_storable<Scalar> {
  typedef std::shared_ptr<matrix_layout<Scalar>> Ptr;

  typename storage<Scalar>::Ptr m_storage = nullptr;

  const bool is_batched;
  const iteration_order iter_order;
  const backend_mem_representation backend_mem_repr;
  const compression compress;
  const size_t shape[2];

  matrix_layout(size_t n_rows, size_t n_cols, iteration_order _iter, 
          backend_mem_representation _bmr, compression _comp, bool _ib=false) :
      is_batched(_ib), iter_order(_iter), backend_mem_repr(_bmr), compress(_comp), shape{n_rows, n_cols}
  {
    if (!is_batched) {
      switch(backend_mem_repr) {
        case UNROLLED:
          //std::cout << "warn: compression ignored for UNROLLED, defacto compressed\n";
          m_storage = std::make_shared<unrolled_storage<Scalar>>(n_rows, n_cols);
          break;
        case FLATTENED:
          m_storage = std::make_shared<flattened_storage<Scalar>>(compress, n_rows, n_cols);
          break;
        case EIGENMATRIX:
          //std::cout << "warn: compression ignored for EIGENMATRIX, default no sparsity tracking\n";
          m_storage = std::make_shared<eigen_matrix_storage<Scalar>>(n_rows, n_cols);
          break;
      }
    }
    else {
      switch(backend_mem_repr) {
        case UNROLLED:
          m_storage = std::make_shared<unrolled_storage<EigenMatrix<Scalar, SIMD_WIDTH, 1>>>(n_rows, n_cols);
          break;
        case FLATTENED:
          m_storage = std::make_shared<flattened_storage<EigenMatrix<Scalar, SIMD_WIDTH, 1>>>(compress, n_rows, n_cols);
          break;
        case EIGENMATRIX:
          assert(false && "simultaneous EigenMatrix and batched repr illegal, only Matrix of scalar value is defined");
          break;
      }
    }
    assert(m_storage != nullptr && "messed up shouldn't happen");
  }

  matrix_layout(const dyn_var<EigenMatrix<Scalar>> &raw_mat) :
      is_batched(false), iter_order(DENSE), backend_mem_repr(EIGENMATRIX), compress(UNCOMPRESSED), shape{raw_mat.n_rows, raw_mat.n_cols}
  {
    m_storage = std::make_shared<eigen_matrix_storage<Scalar>>(raw_mat, shape[0], shape[1]);
  }

  void operator=(const matrix_layout_expr<Scalar> &rhs) {
    if (iter_order == TILE) {
      m_storage->set_matrix(rhs.gen_dyn_matrix());
    }
    else {
      // this unrolls the full for-loop
      for (static_var<size_t> i = 0; i < shape[0]; i = i+1) {
        for (static_var<size_t> j = 0; j < shape[1]; j = j+1) {
          if (iter_order == DENSE) {
            m_storage->set_entry_to_nonconstant(i, j, rhs.gen_entry_at(i, j));
          }
          else {
            if (rhs.is_constant(i, j))
              m_storage->set_entry_to_constant(i, j, rhs.gen_constant_entry_at(i, j));
            else
              m_storage->set_entry_to_nonconstant(i, j, rhs.gen_entry_at(i, j));
          }
        }
      }
    }
  }

  void operator=(const zero_cst_status_storable<Scalar> &mat) {
    *this = matrix_layout_expr_leaf<Scalar>(mat);
  }

  size_t get_n_rows() const {
    return shape[0];
  }

  size_t get_n_cols() const {
    return shape[1];
  }

  dyn_var<EigenMatrix<Scalar>> denseify() const override {
    return m_storage->denseify();
  }

  bool is_zero(size_t i, size_t j) const override {
    return m_storage->is_zero(i, j);
  }

  bool is_constant(size_t i, size_t j) const override {
    return m_storage->is_constant(i, j);
  }

  bool is_nonzero(size_t i, size_t j) const override {
    return m_storage->is_nonzero(i, j);
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    return m_storage->is_nonconstant(i, j);
  }

  builder::builder get_entry(size_t i, size_t j) const override {
    return m_storage->get_entry(i, j);
  }

  Scalar get_constant_entry(size_t i, size_t j) const override {
    return m_storage->get_constant_entry(i, j);
  }

  void set_entry_to_constant(size_t i, size_t j, Scalar val) override { 
    m_storage->set_entry_to_constant(i, j, val);
  }

  void set_entry_to_nonconstant(size_t i, size_t j, const builder::builder &entry) override {
    m_storage->set_entry_to_nonconstant(i, j, entry);
  }

  void set_identity() {
    for (static_var<size_t> i = 0; i < shape[0]; i = i+1) {
      for (static_var<size_t> j = 0; j < shape[1]; j = j+1) {
        if (i == j)
          set_entry_to_constant(i, j, 1);
        else
          set_entry_to_constant(i, j, 0);
      }
    }
  }

  void set_zero() override {
    m_storage->set_zero();
  }

  void set_matrix(const dyn_var<EigenMatrix<Scalar>> &mat) override {
    m_storage->set_matrix(mat);
  }

  bool is_batching_enabled() const override {
    return is_batched;
  }

  std::vector<size_t> get_expr_shape() const override {
    return {shape[0], shape[1]};
  }
};

template <typename Scalar>
struct matrix_layout_expr_leaf : public matrix_layout_expr<Scalar> {
  const struct zero_cst_status_storable<Scalar> &m_mat;
  std::vector<size_t> expr_shape;

  matrix_layout_expr_leaf(const struct zero_cst_status_storable<Scalar> &mat) : m_mat(mat) {
    expr_shape.push_back(m_mat.get_expr_shape()[0]);
    expr_shape.push_back(m_mat.get_expr_shape()[1]);
  }

  const dyn_var<EigenMatrix<Scalar>> gen_dyn_matrix() const override {
    return m_mat.denseify();
  }

  const builder::builder gen_entry_at(size_t i, size_t j) const override {
    return m_mat.get_entry(i, j);
  }

  Scalar gen_constant_entry_at(size_t i, size_t j) const override {
    return m_mat.get_constant_entry(i, j);
  }
  
  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  bool is_batched(size_t i, size_t j) const override {
    return m_mat.is_batching_enabled();
  }

  bool is_nonzero(size_t i, size_t j) const override {
    return m_mat.is_nonzero(i, j);
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    return m_mat.is_nonconstant(i, j);
  }
};

template <typename Scalar>
struct matrix_layout_expr_scalar : public matrix_layout_expr<Scalar> {
  const struct sparse_entry<Scalar> m_se;
  static_var<int> is_nonzero_status;
  static_var<int> is_nonconstant_status;
  std::vector<size_t> expr_shape;

  matrix_layout_expr_scalar(const dyn_var<Scalar> &_scalar, size_t broadcast_rows, size_t broadcast_cols) : m_se(_scalar), is_nonzero_status(true), is_nonconstant_status(true) {
    expr_shape.push_back(broadcast_rows);
    expr_shape.push_back(broadcast_cols);
  }

  matrix_layout_expr_scalar(const Scalar &_scalar, size_t broadcast_rows, size_t broadcast_cols) : m_se(_scalar) {
    is_nonconstant_status = false;
    if (_scalar == 0)
      is_nonzero_status = false;
    else
      is_nonzero_status = true;
    expr_shape.push_back(broadcast_rows);
    expr_shape.push_back(broadcast_cols);
  }

  const dyn_var<EigenMatrix<Scalar>> gen_dyn_matrix() const override {
    dyn_var<EigenMatrix<Scalar>> mat;
    mat.set_matrix_fixed_size(expr_shape[0], expr_shape[1]);
    for (static_var<size_t> i = 0; i < expr_shape[0]; i = i+1) {
      for (static_var<size_t> j = 0; j < expr_shape[1]; j = j+1) {
        if (!is_nonconstant_status)
          mat.coeffRef(i, j) = m_se.static_entry;
        else
          mat.coeffRef(i, j) = m_se.dyn_entry;
      }
    }
    return mat;
  }

  const builder::builder gen_entry_at(size_t i, size_t j) const override {
    if (is_nonconstant(i, j))
      return m_se.dyn_entry;
    return m_se.static_entry;
  }

  Scalar gen_constant_entry_at(size_t i, size_t j) const override {
    return m_se.static_entry;
  }
  
  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  bool is_batched(size_t i, size_t j) const override {
    return false;
  }

  bool is_nonzero(size_t i, size_t j) const override {
    return is_nonzero_status;
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    return is_nonconstant_status;
  }
};

template <typename Scalar>
struct matrix_layout_expr_transpose : public matrix_layout_expr<Scalar> {
  const struct matrix_layout_expr<Scalar> &expr1;
  std::vector<size_t> expr_shape;

  matrix_layout_expr_transpose(const struct matrix_layout_expr<Scalar> &expr1) : expr1(expr1) {
    expr_shape.push_back(expr1.get_expr_shape()[1]);
    expr_shape.push_back(expr1.get_expr_shape()[0]);
  }

  const dyn_var<EigenMatrix<Scalar>> gen_dyn_matrix() const override {
    dyn_var<EigenMatrix<Scalar>> mat;
    mat.set_matrix_fixed_size(expr_shape[0], expr_shape[1]);
    for (static_var<size_t> i = 0; i < expr_shape[0]; i = i+1) {
      for (static_var<size_t> j = 0; j < expr_shape[1]; j = j+1) {
        mat.coeffRef(i, j) = gen_entry_at(i, j);
      }
    }
    return mat;
  }

  const builder::builder gen_entry_at(size_t i, size_t j) const override {
    return expr1.gen_entry_at(j, i);
  }

  Scalar gen_constant_entry_at(size_t i, size_t j) const override {
    return expr1.gen_constant_entry_at(j, i);
  }
  
  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  bool is_batched(size_t i, size_t j) const override {
    return expr1.is_batched(j, i);
  }

  bool is_nonzero(size_t i, size_t j) const override {
    return expr1.is_nonzero(j, i);
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    return expr1.is_nonconstant(j, i);
  }
};

template <typename Scalar>
struct matrix_layout_expr_mul : public matrix_layout_expr<Scalar> {
  const struct matrix_layout_expr<Scalar> &expr1;
  const struct matrix_layout_expr<Scalar> &expr2;

  using matrix_layout_expr<Scalar>::is_constant;

  std::vector<size_t> expr_shape;

  matrix_layout_expr_mul(const struct matrix_layout_expr<Scalar> &expr1, const struct matrix_layout_expr<Scalar> &expr2)
      : expr1(expr1), expr2(expr2) {
    std::vector<size_t> shape1, shape2;
    shape1 = expr1.get_expr_shape();
    shape2 = expr2.get_expr_shape();

    assert(shape1[1] == shape2[0] && "inner dim of matmul expr must match");

    expr_shape.push_back(shape1[0]);
    expr_shape.push_back(shape2[1]);
  }

  const dyn_var<EigenMatrix<Scalar>> gen_dyn_matrix() const override {
    return expr1.gen_dyn_matrix() * expr2.gen_dyn_matrix();
  }

  const builder::builder gen_entry_at(size_t i, size_t j) const override {
    if (!is_batched(i, j)) {
      const size_t inner_dim = expr1.get_expr_shape()[1];
      dyn_var<Scalar> sum = 0;
      // k is inner_dim for matmul
      for (static_var<size_t> k = 0; k < inner_dim; k = k + 1) {
        sum += expr1.gen_entry_at(i, k) * expr2.gen_entry_at(k, j);
      }
      return sum;
    }
    else {
      // will contain eigen specific calls for handling constants in a special way
      // if (expr1.is_batched() && !expr2.is_batched()) ... is the X_J * X_T case
      assert(false && "todo");
    }
  }

  Scalar gen_constant_entry_at(size_t i, size_t j) const override {
    assert(is_constant(i, j) && "entry isn't constant");
    if (!is_batched(i, j)) {
      const size_t inner_dim = expr1.get_expr_shape()[1];
      static_var<Scalar> sum = 0;
      // k is inner_dim for matmul
      for (static_var<size_t> k = 0; k < inner_dim; k = k + 1) {
        assert(!(expr1.is_nonconstant(i, k) && expr2.is_nonconstant(k, j)) && "constant output impossible if both exprs are nonconstant");
        if (expr1.is_zero(i, k) || expr2.is_zero(k, j))
          sum += 0;
        else {
          assert(expr1.is_constant(i, k) && expr2.is_constant(k, j) && "if expr1 and expr2 are not both dyn, and neither are zero, they must be nonzero constants");
          sum += expr1.gen_constant_entry_at(i, k) * expr2.gen_constant_entry_at(k, j);
        }
      }
      return sum;
    }
    else {
      // will contain eigen specific calls for handling constants in a special way
      assert(false && "todo");
    }
  }
  
  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  bool is_batched(size_t i, size_t j) const override {
    // todo
    return expr1.is_batched(i, j) || expr2.is_batched(i, j);
  }

  bool is_nonzero(size_t i, size_t j) const override {
    const size_t inner_dim = expr1.get_expr_shape()[1];
    for (static_var<size_t> k = 0; k < inner_dim; k = k + 1) {
      // when summing up products of inner_dim, if any one product is nonzero
      // then (i, j) is guaranteed to be nonzero.
      if (expr1.is_nonzero(i, k) && expr2.is_nonzero(k, j))
        return true;
    }
    return false;
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    const size_t inner_dim = expr1.get_expr_shape()[1];
    for (static_var<size_t> k = 0; k < inner_dim; k = k + 1) {
      if (expr1.is_zero(i, k) || expr2.is_zero(k, j))
        continue;
      else if (expr1.is_constant(i, k) && expr2.is_constant(k, j))
        continue;
      else
        // inner product is constant only when both exprs are constant
        // or either is zero
        // any other combination results in a nonconstant output.
        return true;
    }
    // if we didn't break out of loop earlier, means all inner products were
    // constant so their sum will also be constant
    return false;
  }
};

template <typename Scalar>
struct matrix_layout_expr_add : public matrix_layout_expr<Scalar> {
  const struct matrix_layout_expr<Scalar> &expr1;
  const struct matrix_layout_expr<Scalar> &expr2;

  std::vector<size_t> expr_shape;

  matrix_layout_expr_add(const struct matrix_layout_expr<Scalar> &expr1, const struct matrix_layout_expr<Scalar> &expr2)
      : expr1(expr1), expr2(expr2) {
    std::vector<size_t> shape1, shape2;
    shape1 = expr1.get_expr_shape();
    shape2 = expr2.get_expr_shape();

    assert(shape1[0] == shape2[0] && shape1[1] == shape2[1] && "shapes must match");

    expr_shape.push_back(shape1[0]);
    expr_shape.push_back(shape1[1]);
  }

  const dyn_var<EigenMatrix<Scalar>> gen_dyn_matrix() const override {
    return expr1.gen_dyn_matrix() + expr2.gen_dyn_matrix();
  }

  const builder::builder gen_entry_at(size_t i, size_t j) const override {
    if (!is_batched(i, j))
      return expr1.gen_entry_at(i, j) + expr2.gen_entry_at(i, j);
    else
      assert(false && "todo");
  }

  Scalar gen_constant_entry_at(size_t i, size_t j) const override {
    if (!is_batched(i, j)) {
      assert(expr1.is_constant(i, j) && expr2.is_constant(i, j) && "both exprs must be constant");
      return expr1.gen_constant_entry_at(i, j) + expr2.gen_constant_entry_at(i, j);
    }
    else {
      // will contain eigen specific calls for handling constants in a special way
      assert(false && "todo");
    }
  }
  
  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  bool is_batched(size_t i, size_t j) const override {
    // todo
    return expr1.is_batched(i, j) || expr2.is_batched(i, j);
  }

  bool is_nonzero(size_t i, size_t j) const override {
    return expr1.is_nonzero(i, j) || expr2.is_nonzero(i, j);
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    return expr1.is_nonconstant(i, j) || expr2.is_nonconstant(i, j);
  }
};

template <typename Scalar>
struct matrix_layout_expr_unary_minus : public matrix_layout_expr<Scalar> {
  const struct matrix_layout_expr<Scalar> &expr1;

  std::vector<size_t> expr_shape;

  matrix_layout_expr_unary_minus(const struct matrix_layout_expr<Scalar> &expr1)
      : expr1(expr1) {
    expr_shape = expr1.get_expr_shape();
  }

  const dyn_var<EigenMatrix<Scalar>> gen_dyn_matrix() const override {
    return -expr1.gen_dyn_matrix();
  }

  const builder::builder gen_entry_at(size_t i, size_t j) const override {
    return -expr1.gen_entry_at(i, j);
  }

  Scalar gen_constant_entry_at(size_t i, size_t j) const override {
    if (!is_batched(i, j)) {
      assert(expr1.is_constant(i, j) && "must be constant");
      return -expr1.gen_constant_entry_at(i, j);
    }
    else {
      // will contain eigen specific calls for handling constants in a special way
      assert(false && "todo");
    }
  }
  
  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  bool is_batched(size_t i, size_t j) const override {
    // todo
    return expr1.is_batched(i, j);
  }

  bool is_nonzero(size_t i, size_t j) const override {
    return expr1.is_nonzero(i, j);
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    return expr1.is_nonconstant(i, j);
  }
};

template <typename Scalar>
struct matrix_layout_expr_cwise_mul : public matrix_layout_expr<Scalar> {
  const struct matrix_layout_expr<Scalar> &expr1;
  const struct matrix_layout_expr<Scalar> &expr2;

  using matrix_layout_expr<Scalar>::is_constant;

  std::vector<size_t> expr_shape;

  matrix_layout_expr_cwise_mul(const struct matrix_layout_expr<Scalar> &expr1, const struct matrix_layout_expr<Scalar> &expr2)
      : expr1(expr1), expr2(expr2) {
    std::vector<size_t> shape1, shape2;
    shape1 = expr1.get_expr_shape();
    shape2 = expr2.get_expr_shape();

    assert(shape1[0] == shape2[0] && shape1[1] == shape2[1] && "shapes must match");

    expr_shape.push_back(shape1[0]);
    expr_shape.push_back(shape1[1]);
  }

  const dyn_var<EigenMatrix<Scalar>> gen_dyn_matrix() const override {
    // todo change to cwise mul
    return expr1.gen_dyn_matrix() * expr2.gen_dyn_matrix();
  }

  const builder::builder gen_entry_at(size_t i, size_t j) const override {
    if (!is_batched(i, j)) {
      return expr1.gen_entry_at(i, j) * expr2.gen_entry_at(i, j);
    }
    else {
      // will contain eigen specific calls for handling constants in a special way
      // if (expr1.is_batched() && !expr2.is_batched()) ... is the X_J * X_T case
      assert(false && "todo");
    }
  }

  Scalar gen_constant_entry_at(size_t i, size_t j) const override {
    assert(is_constant(i, j) && "entry isn't constant");
    if (!is_batched(i, j)) {
      assert(expr1.is_constant(i, j) && expr2.is_constant(i, j) && "both exprs must be constant");
      return expr1.gen_constant_entry_at(i, j) * expr2.gen_constant_entry_at(i, j);
    }
    else {
      // will contain eigen specific calls for handling constants in a special way
      assert(false && "todo");
    }
  }
  
  std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  bool is_batched(size_t i, size_t j) const override {
    // todo
    return expr1.is_batched(i, j) || expr2.is_batched(i, j);
  }

  bool is_nonzero(size_t i, size_t j) const override {
    return expr1.is_nonzero(i, j) && expr2.is_nonzero(i, j);
  }

  bool is_nonconstant(size_t i, size_t j) const override {
    return expr1.is_nonconstant(i, j) || expr2.is_nonconstant(i, j);
  }
};

}

#endif
