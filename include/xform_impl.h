#ifndef XFORM_IMPL_H
#define XFORM_IMPL_H
#include "forward_declarations.h"
#include "builder/static_var.h"
#include "builder/dyn_var.h"
#include <unordered_map>
#include <vector>
#include "backend.h"

#include <sstream>
#include <cassert>
// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

using builder::dyn_var;
using builder::static_var;

namespace ctup {

// todo: conditionally generate backend statements using hw features
struct HwFeatures {
  bool is_vectorized;
  bool is_threaded;
  enum {
    AVX2,
    AVX512,
    NEON
  } simd_type_id;
  size_t n_threads;
};

// Design intentions: 
// * Abstract away matrix representation from matrix types (Rotation, Translation,
// Xform, etc.)
// * Users only construct expressions of matrices, assign matrix entries, and
// the Storage class lowers them to whatever matrix representation has been
// chosen (DENSE, SPARSE_UNROLLED, SPARSE_BLOCKED...)
// * It shouldn't be the user's job to handle whether the matrix is being
// representated as a soup of single variables (SPARSE_UNROLLED) or a generic
// DENSE Eigen matrix.
// * Storage should be the only place we directly generate
// backend/runtime matrix/unrolled variable statements from.
// todo: Storage class needs much more work.
template<typename Scalar>
struct Storage {
  static_var<size_t> n_rows;
  static_var<size_t> n_cols;

  // for dense
  //dyn_var<builder::eigen_Xmat_t> m_matrix;

  // for sparse unrolled
  std::vector<dyn_var<Scalar>*> sparse_vars;

  // for sparse matrix
  dyn_var<Scalar[]> m_buffer;

  // for all sparse matrices, turns a dense index to its
  // sparse index to index into either m_buffer or sparse_vars
  std::unordered_map<size_t, size_t> dense_to_sparse_idx;

  enum Sparsity_type_id {
    DENSE, // default
    SPARSE_UNROLLED,
    SPARSE_BLOCKED,
    SPARSE_MATRIX,
    SKEW_SYMM,
  } sparsity_type_id;

  enum Storage_order_id {
    COL_MAJ, // default
    ROW_MAJ,
  } storage_order_id;

  Storage() : n_rows(0), n_cols(0), sparsity_type_id(DENSE), storage_order_id(COL_MAJ) {}

  Storage(size_t _n_rows, size_t _n_cols, Sparsity_type_id _sti=DENSE, Storage_order_id _soi=COL_MAJ) : 
      n_rows(_n_rows), n_cols(_n_cols), sparsity_type_id(_sti), storage_order_id(_soi) {}


  std::string append_idx_error_msg(std::string error_msg, size_t i, size_t j) {
    auto err_stream = std::stringstream{};
    err_stream << error_msg << ": " << "(" << i << "," << j << ")";
    return err_stream.str();
  }

  size_t get_flattened_index(size_t i, size_t j) {
    size_t flattened;
    if (storage_order_id == COL_MAJ)
      flattened = i * n_rows + j;
    else if (storage_order_id == ROW_MAJ)
      flattened = i * n_cols + j;

    const char * error_msg = append_idx_error_msg("index out of bounds", i, j).c_str();
    assertm(flattened < n_rows * n_cols, error_msg);

    return flattened;
  }

  size_t get_dense_to_sparse_idx(size_t i, size_t j) {
    size_t flattened_idx = get_flattened_index(i, j);

    const char * error_msg = append_idx_error_msg("no sparse entry found for", i, j).c_str();
    assertm(is_nonzero(i, j), error_msg);

    return dense_to_sparse_idx[flattened_idx];
  }

  dyn_var<Scalar> operator()(size_t i, size_t j) {
    size_t flattened_idx = get_flattened_index(i, j);

    if (sparsity_type_id == DENSE) {
      //return m_matrix(i, j);
      return m_buffer[flattened_idx];
    }
    if (sparsity_type_id == SPARSE_UNROLLED) {
      return *sparse_vars[get_dense_to_sparse_idx(i, j)];
    }
    if (sparsity_type_id == SPARSE_MATRIX) {
      return m_buffer[get_dense_to_sparse_idx(i, j)];
    }
  }
  
  int is_zero(size_t i, size_t j) {
    // couldn't find entry that maps dense idx to sparse idx
    return (dense_to_sparse_idx.find(get_flattened_index(i, j)) == dense_to_sparse_idx.end());
  }

  int is_nonzero(size_t i, size_t j) {
    return !is_zero(i, j);
  }

  void add_sparse_entry(dyn_var<Scalar>* e, size_t i, size_t j) {
    const char * error_msg = append_idx_error_msg("sparse entry already exists", i, j).c_str();
    assertm(is_zero(i, j), error_msg);

    sparse_vars.push_back(e);
    dense_to_sparse_idx[get_flattened_index(i, j)] = sparse_vars.size() - 1;
  }
};

template<typename Scalar>
struct Translation_expr {
  virtual const builder::builder get_x();
  virtual const builder::builder get_y();
  virtual const builder::builder get_z();

  virtual const int has_x();
  virtual const int has_y();
  virtual const int has_z();
};

template<typename Scalar>
struct Rotation_expr {
  virtual const builder::builder get_value_at(size_t r, size_t c);
  virtual int is_nonzero(size_t r, size_t c);
};

template<typename Scalar>
struct Xform_expr {
  virtual const builder::builder get_value();
  virtual const Rotation_expr<Scalar> get_rotation_expr();
  virtual const Translation_expr<Scalar> get_translation_expr();

  virtual const int has_rotation();
  virtual const int has_translation();
};


template<typename Scalar>
struct Translation {
private:
  // marked private because we want to make sure we track sparsity whenever
  // we assign some value to x, y, or z with an explicit setter
  dyn_var<Scalar> x;
  dyn_var<Scalar> y;
  dyn_var<Scalar> z;

public:
  static_var<int> has_x;
  static_var<int> has_y;
  static_var<int> has_z;

  Translation() : has_x(false), has_y(false), has_z(false) {}

  Translation(Scalar _x, Scalar _y, Scalar _z) {
    set_x(_x);
    set_y(_y);
    set_z(_z);
  }

  void set_x(Scalar val) {
    if (val == 0) {
      has_x = false;
      x = 0;
      return;
    }
    has_x = true;
    x = val;
  }
  void set_y(Scalar val) {
    if (val == 0) {
      has_y = false;
      y = 0;
      return;
    }
    has_y = true;
    y = val;
  }
  void set_z(Scalar val) {
    if (val == 0) {
      has_y = false;
      z = 0;
      return;
    }
    has_z = true;
    z = val;
  }

  void set_prismatic_axis(char axis) {
    if (axis == 'X')
      has_x = true;
    else if (axis == 'Y')
      has_y = true;
    else if (axis == 'Z')
      has_z = true;
  }

  void jcalc(dyn_var<Scalar> &q_i) {
    if (has_x)
      x = q_i;
    if (has_y)
      y = q_i;
    if (has_z)
      z = q_i;
  }

  void operator= (Translation_expr<Scalar> &rhs) {
    if (rhs.has_x()) {
      x = rhs.get_x();
      has_x = true;
    }
    if (rhs.has_y()) {
      y = rhs.get_y();
      has_y = true;
    }
    if (rhs.has_z()) {
      z = rhs.get_z();
      has_z = true;
    }
  }
};

template<typename Scalar>
struct Rotation {
  Storage<Scalar> storage;

  dyn_var<Scalar> s;
  dyn_var<Scalar> c;
  dyn_var<Scalar> minus_s;
  dyn_var<Scalar> one;

  static_var<int> has_x;
  static_var<int> has_y;
  static_var<int> has_z;

  Rotation() : has_x(false), has_y(false), has_z(false) {}

  void set_revolute_axis(char axis) {
    one = 1;
    if (axis == 'X') {
      has_x = true;
      storage.add_sparse_entry(one.addr(), 0, 0);
      storage.add_sparse_entry(c.addr(), 1, 1);
      storage.add_sparse_entry(minus_s.addr(), 1, 2);
      storage.add_sparse_entry(s.addr(), 2, 1);
      storage.add_sparse_entry(c.addr(), 2, 2);
    }
    else if (axis == 'Y') {
      has_y = true;
      storage.add_sparse_entry(one.addr(), 1, 1);
      storage.add_sparse_entry(c.addr(), 0, 0);
      storage.add_sparse_entry(s.addr(), 0, 2);
      storage.add_sparse_entry(minus_s.addr(), 2, 0);
      storage.add_sparse_entry(c.addr(), 2, 2);
    }
    else if (axis == 'Z') {
      has_z = true;
      storage.add_sparse_entry(one.addr(), 2, 2);
      storage.add_sparse_entry(c.addr(), 0, 0);
      storage.add_sparse_entry(minus_s.addr(), 0, 1);
      storage.add_sparse_entry(s.addr(), 1, 0);
      storage.add_sparse_entry(c.addr(), 1, 1);
    }
  }

  void jcalc(dyn_var<Scalar> &q_i) {
    s = backend::sin(q_i);
    minus_s = -s;
    c = backend::cos(q_i);
  }

  void operator= (Rotation_expr<Scalar> &rhs) {
    for (static_var<size_t> i = 0; i < 3; i = i + 1) {
      for (static_var<size_t> j = 0; j < 3; j = j + 1) {
        if (rhs.is_nonzero(i, j)) {
          // todo: propagate has_x,y,z here
          storage(i, j) = rhs.get_value_at(i, j);
        }
      }
    }
  }
};

template<typename Scalar>
struct Xform {
  Rotation<Scalar> rot;
  Translation<Scalar> trans;

  static_var<int> has_rotation;
  static_var<int> has_translation;

  Xform() : has_rotation(0), has_translation(0) {}

    // if set_revolute/prismatic funcs are being called, it means this Xform is associated
    // with a joint.
    // A joint cannot be both prismatic and revolute.
    // We enforce this condition using an assert.
  void set_revolute_axis(char axis) {
    assertm(has_translation == false, "joint xform cannot be both revolute and prismatic");
    has_rotation = true;
    rot.set_revolute_axis(axis);
  }
  void set_prismatic_axis(char axis) {
    assertm(has_rotation == false, "joint xform cannot be both revolute and prismatic");
    has_translation = true;
    trans.set_prismatic_axis(axis);
  }

  void jcalc(dyn_var<Scalar> &q_i) {
    if (has_rotation) {
      rot.jcalc(q_i);
    }
    else if (has_translation) {
      trans.jcalc(q_i);
    }
  }

  void operator= (Xform_expr<Scalar> &rhs) {
    if (rhs.has_rotation()) {
      has_rotation = true;
      rot = rhs.get_rotation_expr();
    }
    if (rhs.has_translation()) {
      has_translation = true;
      trans = rhs.get_translation_expr();
    }
  }
};

// Expressions

template<typename Scalar>
struct Translation_expr_leaf : public Translation_expr<Scalar> {
  const struct Translation<Scalar>& m_trans;

  Translation_expr_leaf(const struct Translation<Scalar>& trans) : m_trans(trans) {}

  const builder::builder get_x() {
    return m_trans.x;
  }
  const builder::builder get_y() {
    return m_trans.y;
  }
  const builder::builder get_z() {
    return m_trans.z;
  }
};

template<typename Scalar>
struct Translation_expr_add : public Translation_expr<Scalar> {
  const struct Translation_expr<Scalar>& expr1;
  const struct Translation_expr<Scalar>& expr2;

  Translation_expr_add(const struct Translation_expr<Scalar>& expr1, const struct Translation_expr<Scalar>& expr2) :
    expr1(expr1), expr2(expr2) {}

  const builder::builder get_x() {
    return expr1.get_x() + expr2.get_x();
  }
  const builder::builder get_y() {
    return expr1.get_y() + expr2.get_y();
  }
  const builder::builder get_z() {
    return expr1.get_z() + expr2.get_z();
  }
};

template<typename Scalar>
struct Rotation_expr_leaf : public Rotation_expr<Scalar> {
  const struct Rotation<Scalar>& m_rot;

  Rotation_expr_leaf(const struct Rotation<Scalar>& rot) : m_rot(rot) {}

  const builder::builder get_value_at(size_t i, size_t j) {
    return m_rot.storage(i, j);
  }

  int is_nonzero(size_t i, size_t j) {
    return m_rot.storage.is_nonzero(i, j);
  }
};

template<typename Scalar>
struct Rotation_expr_mul : public Rotation_expr<Scalar> {
  const struct Rotation_expr<Scalar>& expr1;
  const struct Rotation_expr<Scalar>& expr2;

  Rotation_expr_mul(const struct Rotation_expr<Scalar>& expr1, const struct Rotation_expr<Scalar>& expr2) :
    expr1(expr1), expr2(expr2) {}

  const builder::builder get_value_at(size_t i, size_t j) {
    dyn_var<Scalar> sum = 0;
    // k is inner_dim for matmul
    for (static_var<size_t> k = 0; k < 3; k = k + 1) {
      sum += expr1.get_value_at(i, k) * expr2.get_value_at(k, j); 
    }
    return sum;
  }

  int is_nonzero(size_t i, size_t j) {
    for (static_var<size_t> k = 0; k < 3; k = k + 1) {
      if (expr1.is_nonzero(i, k) && expr2.is_nonzero(k, j))
        return true;
    }
  }
};

template<typename Scalar>
struct Xform_expr_leaf : public Xform_expr<Scalar> {
  const struct Xform<Scalar>& m_xform;

  Xform_expr_leaf(const struct Xform<Scalar>& xform) : m_xform(xform) {}

  const Rotation_expr<Scalar> get_rotation_expr() {
    return m_xform.rot; // todo: cast to Rot_expr_leaf
  }
  const Translation_expr<Scalar> get_translation_expr() {
    return m_xform.trans; // todo: cast to Translation_expr_leaf
  }
};

template<typename Scalar>
struct Xform_expr_mul : public Xform_expr<Scalar> {
  const struct Xform_expr<Scalar>& expr1;
  const struct Xform_expr<Scalar>& expr2;

  Xform_expr_mul(const struct Xform_expr<Scalar>& expr1, const struct Xform_expr<Scalar>& expr2) :
    expr1(expr1), expr2(expr2) {}

  const Rotation_expr<Scalar> get_rotation_expr() {
    return expr1.get_rotation_expr() * expr2.get_rotation_expr();
  }
  const Translation_expr<Scalar> get_translation_expr() {
    return expr1.get_translation_expr() * expr2.get_translation_expr();
  }
};


}

#endif
