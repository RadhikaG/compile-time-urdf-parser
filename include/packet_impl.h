#ifndef PACKET_IMPL_H
#define PACKET_IMPL_H
#include "backend.h"
#include "builder/builder_base.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "builder/static_var.h"
#include <unordered_map>
#include <vector>

#include <cassert>
#include <sstream>
// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

using builder::dyn_var;
using builder::static_var;

namespace ctup {

enum Sparsity_type_id {
  DENSE,
  SPARSE_UNROLLED,
  //SPARSE_BLOCKED,
  //SPARSE_MATRIX,
  //SKEW_SYMM,
};

// todo: conditionally generate backend statements using hw features later
struct Flags {
  bool simplify_zero;
  Sparsity_type_id sparsity_type_id;
  bool is_vectorized;
  bool is_threaded;
  size_t n_threads;
  size_t size_packet;
  size_t size_subpacket;
  enum {
    AVX2,
    AVX512,
    NEON,
  } simd_type_id;

  Flags(Sparsity_type_id _sparsity_type_id = DENSE, bool _simplify_zero = true)
      : simplify_zero(_simplify_zero), sparsity_type_id(_sparsity_type_id,size_t size_packet = 36,size_t size_subpacket=16) {}
};
// only one of this through ctup for now
static Flags flags(DENSE, // sparsity_type_id
                   true   // simplify_zero
);

template <typename Scalar>
struct Matrix_expr {
  virtual const builder::builder get_value() const {
    return dyn_var<EigenMatrix<Scalar>>();
  }
  virtual const builder::builder get_value_at(size_t i, size_t j) const {
    return dyn_var<int>();
  }
  virtual int is_nonzero(size_t i, size_t j) const {
    return false;
  }
  virtual const std::vector<size_t> get_expr_shape(void) const {
    return std::vector<size_t>{};
  }
};

// Design intentions:
// * Abstract away matrix representation from matrix types (Rotation, Translation,
// Packet, etc.)
// * Users only construct expressions of matrices, assign matrix entries, and
// the Storage class lowers them to whatever matrix representation has been
// chosen (DENSE, SPARSE_UNROLLED, SPARSE_BLOCKED...)
// * It shouldn't be the user's job to handle whether the matrix is being
// representated as a soup of single variables (SPARSE_UNROLLED) or a generic
// DENSE Eigen matrix.
// * Storage should be the only place we directly generate
// backend/runtime matrix/unrolled variable statements from.
// todo: Storage class needs much more work.
template <typename Scalar>
struct Storage {
private:
  // SparseEntry should be inaccessible from outside Storage
  struct SparseEntry {
    dyn_var<Scalar> dyn_entry; // = builder::defer_init();
    static_var<double> static_entry; //CAMBIO
    static_var<int> is_constant;

    SparseEntry() : static_entry(0), is_constant(true) {}

    void operator=(Scalar val) {
      is_constant = true;
      static_entry = val;
    }

    void operator=(const dyn_var<Scalar>& val) {
      is_constant = false;
      dyn_entry = val;
    }
  };

public:
  const size_t n_rows;
  const size_t n_cols;

  // for dense
  dyn_var<EigenMatrix<Scalar>> m_matrix; // = builder::defer_init();

  // for sparse unrolled
  std::vector<SparseEntry> sparse_vars;

  // for sparse matrix
  dyn_var<Scalar[]> m_buffer = builder::defer_init();

  // for all sparse matrices, turns a dense index to its
  // sparse index to index into either m_buffer or sparse_vars
  std::unordered_map<size_t, size_t> dense_to_sparse_idx;

  Sparsity_type_id sparsity_type_id;

  enum Storage_order_id {
    COL_MAJ, // default
    ROW_MAJ,
  } storage_order_id;

  enum Sparse_entry_type_id {
    ZERO,
    NONZERO,
    CONSTANT,
    VARIABLE,
  } sparse_entry_type_id;

  Storage(size_t _n_rows, size_t _n_cols, Sparsity_type_id _sti = flags.sparsity_type_id,
          Storage_order_id _soi = COL_MAJ)
      : n_rows(_n_rows), n_cols(_n_cols), m_matrix(n_rows, n_cols), sparsity_type_id(_sti),
        storage_order_id(_soi) {
    if (sparsity_type_id == SPARSE_UNROLLED) {
      // set all matrix entries as non constant in the beginning
      sparse_vars.resize(n_rows * n_cols);
    }
    else if (sparsity_type_id == DENSE) {
      //m_matrix.deferred_init();
    }
  }

  std::string append_idx_error_msg(std::string error_msg, size_t i, size_t j) const {
    auto err_stream = std::stringstream{};
    err_stream << error_msg << ": "
               << "(" << i << "," << j << ")";
    return err_stream.str();
  }

  size_t get_flattened_index(size_t i, size_t j) const {
    size_t flattened = -1;
    if (storage_order_id == COL_MAJ)
      // i is col-idx, j is row-idx
      flattened = i * n_rows + j;
    else if (storage_order_id == ROW_MAJ)
      // i is row-idx, j is col-idx
      flattened = i * n_cols + j;

    const char *error_msg = append_idx_error_msg("index out of bounds", i, j).c_str();
    assertm(flattened < n_rows * n_cols, error_msg);

    return flattened;
  }

  size_t get_dense_to_sparse_idx(size_t i, size_t j) const {
    size_t flattened_idx = get_flattened_index(i, j);

    const char *error_msg = append_idx_error_msg("no sparse entry found for", i, j).c_str();
    assertm(is_nonzero(i, j), error_msg);

    return dense_to_sparse_idx.find(flattened_idx)->second;
  }

  dyn_var<Scalar> _get(size_t i, size_t j) const {
    size_t flattened_idx = get_flattened_index(i, j);

    if (sparsity_type_id == DENSE) {
      return const_cast<dyn_var<EigenMatrix<Scalar>> &>(m_matrix).coeffRef(i, j);
    }
    if (sparsity_type_id == SPARSE_UNROLLED) {
      const SparseEntry &e = sparse_vars[flattened_idx];
      if (e.is_constant) {
        const char *error_msg =
            append_idx_error_msg(
                "can't retrieve constant entries using (i, j), use get_constant_entry", i, j)
                .c_str();
        assertm(false, error_msg);
      }
      return e.dyn_entry;
    }
    //if (sparsity_type_id == SPARSE_MATRIX) {
    //  if (is_zero(i, j)) {
    //    const char * error_msg = append_idx_error_msg("can't retrieve constant (zero) value using (i, j), use get_constant_entry", i, j).c_str();
    //    assertm(false, error_msg);
    //  }
    //  return const_cast<dyn_var<Scalar[]>&>(m_buffer)[get_dense_to_sparse_idx(i, j)];
    //}

    assertm(false, "no impl for sparsity type");
    return sparse_vars[0].dyn_entry; // return bs value
  }

  // meant to be used on RHS of an assignment statement
  dyn_var<Scalar> get_dyn_entry(size_t i, size_t j) {
    return _get(i, j);
  }
  // meant to be used on LHS of an assignment statement, with const storage
  const dyn_var<Scalar> get_dyn_entry(size_t i, size_t j) const {
    return _get(i, j);
  }

  builder::builder get_entry(size_t i, size_t j) const {
    if (sparsity_type_id == DENSE) {
      return get_dyn_entry(i, j);
    }
    else if (sparsity_type_id == SPARSE_UNROLLED) {
      if (is_constant(i, j))
        return get_constant_entry(i, j);
      else
        return get_dyn_entry(i, j);
    }

    assertm(false, "type not implemented");
    return sparse_vars[0].dyn_entry; // return bs value
  }

  dyn_var<EigenMatrix<Scalar>> denseify() const {
    if (sparsity_type_id == SPARSE_UNROLLED) {
      dyn_var<EigenMatrix<Scalar>> converted_mat(n_rows, n_cols);

      converted_mat.setZero();
      for (static_var<size_t> i = 0; i < n_rows; i = i + 1) {
        for (static_var<size_t> j = 0; j < n_cols; j = j + 1) {
          converted_mat.coeffRef(i, j) = get_entry(i, j);
        }
      }

      return converted_mat;
    }
    else if (sparsity_type_id == DENSE) {
      return m_matrix;
    }

    assertm(false, "type not implemented");
    return false;
  }

  Scalar get_constant_entry(size_t i, size_t j) const {
    size_t flattened_idx = get_flattened_index(i, j);

    if (sparsity_type_id == DENSE)
      assertm(false, "no constant tracking for DENSE matrices");
    else if (sparsity_type_id == SPARSE_UNROLLED) {
      const SparseEntry &e = sparse_vars[flattened_idx];
      if (!e.is_constant)
        assertm(false, "entry is not constant");
      return e.static_entry;
    }

    assertm(false, "constants are untracked for this sparsity type");
    return false;
  }

  void set_entry_to_constant(size_t i, size_t j, Scalar val) {
    size_t flattened_idx = get_flattened_index(i, j);

    if (flags.simplify_zero) {
      if (std::abs(val) < 1e-5)
        val = 0;
    }

    if (sparsity_type_id == DENSE) {
      m_matrix.coeffRef(i, j) = val;
    }
    else if (sparsity_type_id == SPARSE_UNROLLED) {
      SparseEntry &e = sparse_vars[flattened_idx];
      e = val;
    }
    else
      assertm(false, "todo unsupported");
  }

  void set_entry_to_dyn(size_t i, size_t j, const dyn_var<Scalar> &val) {
    size_t flattened_idx = get_flattened_index(i, j);

    if (sparsity_type_id == DENSE) {
      m_matrix.coeffRef(i, j) = val;
    }
    else if (sparsity_type_id == SPARSE_UNROLLED) {
      SparseEntry &e = sparse_vars[flattened_idx];
      e = val;
    }
    else
      assertm(false, "todo unsupported");
  }

  void set_matrix(const dyn_var<EigenMatrix<Scalar>> &mat) {
    if (sparsity_type_id == DENSE) {
      m_matrix = mat;
    }
    else
      assertm(false, "todo unsupported");
  }

  int is_constant(size_t i, size_t j) const {
    size_t flattened_idx = get_flattened_index(i, j);
    if (sparsity_type_id == SPARSE_UNROLLED) {
      if (sparse_vars[flattened_idx].is_constant)
        return true;
      else
        return false;
    }

    assertm(false, "constants are untracked for this sparsity type");
    return false;
  }

  int is_zero(size_t i, size_t j) const {
    size_t flattened_idx = get_flattened_index(i, j);

    if (sparsity_type_id == SPARSE_UNROLLED) {
      if (!sparse_vars[flattened_idx].is_constant)
        return false;
      else if (sparse_vars[flattened_idx].static_entry == 0)
        return true;
      else
        return false;
    }

    assertm(false, "zero are untracked for this sparsity type");
    return false;
  }

  int is_nonzero(size_t i, size_t j) const {
    return !is_zero(i, j);
  }
};

template <typename Scalar>
struct Translation_expr : public Matrix_expr<Scalar> {
  virtual const builder::builder get_value() const override {
    dyn_var<EigenMatrix<Scalar>> translation_mat(3, 3);
    translation_mat.setZero();

    translation_mat.coeffRef(1, 0) = get_z();
    translation_mat.coeffRef(0, 1) = -get_z();
    translation_mat.coeffRef(0, 2) = get_y();
    translation_mat.coeffRef(2, 0) = -get_y();
    translation_mat.coeffRef(2, 1) = get_x();
    translation_mat.coeffRef(1, 2) = -get_x();

    return translation_mat;
  }

  virtual const builder::builder get_value_at(size_t i, size_t j) const override {
    // get skew symmetric entry
    // [x]         [ 0 -z   y]
    // [y] cross = [ z  0  -x]
    // [z]         [-y  x   0]
    // zero along diagonal
    if (i == j)
      return 0;
    if (i == 1 && j == 0)
      return get_z();
    if (i == 0 && j == 1)
      return -get_z();
    if (i == 2 && j == 0)
      return -get_y();
    if (i == 0 && j == 2)
      return get_y();
    if (i == 2 && j == 1)
      return get_x();
    if (i == 1 && j == 2)
      return -get_x();

    assertm(false, "index out of bounds");
    return get_x(); // return bs value
  }

  virtual int is_nonzero(size_t i, size_t j) const override {
    //if (i == j)
    //  return false;
    //return true;
    return false;
  }

  virtual const builder::builder get_x() const {
    return dyn_var<EigenMatrix<int,flags.size_subpacket,1>>(); //CAMBIO
  }
  virtual const builder::builder get_y() const {
    return dyn_var<EigenMatrix<int,flags.size_subpacket,1>>(); //CAMBIO
  }
  virtual const builder::builder get_z() const {
    return dyn_var<EigenMatrix<int,flags.size_subpacket,1>>(); //CAMBIO
  }

  virtual const std::vector<size_t> get_expr_shape(void) const override {
    return std::vector<size_t>({3, 3}); //FALTA
  }

  virtual int has_x() const {
    return false;
  }
  virtual int has_y() const {
    return false;
  }
  virtual int has_z() const {
    return false;
  }
};

//template<typename Scalar>
//struct Rotation_expr : public Matrix_expr<Scalar> {
//  virtual const builder::builder get_value_at(size_t i, size_t j) const override { return dyn_var<int>(); }
//  virtual int is_nonzero(size_t i, size_t j) const override { return false; }
//
//  virtual int has_x() const { return false; }
//  virtual int has_y() const { return false; }
//  virtual int has_z() const { return false; }
//};

template <typename Scalar>
struct Packet_expr : public Matrix_expr<Scalar> {
  virtual const builder::builder get_value() const override {
    dyn_var<EigenMatrix<Scalar>> X_mat(6, 6);
    X_mat.setZero();

    X_mat.block(0, 0) = get_rotation_expr().get_value();
    X_mat.block(3, 3) = get_rotation_expr().get_value();
    X_mat.block(3, 0) = get_minus_E_rcross_expr().get_value();

    return X_mat;
  }

  virtual const builder::builder get_value_at(size_t i, size_t j) const override {
    // X matrix dense repr:
    // -Erx = -rot * cross(trans)
    // [ rot  0 ]
    // [-Erx rot]
    if (i < 3 && j < 3) {
      // rotation part
      return get_rotation_expr().get_value_at(i, j);
    }
    else if (i >= 3 && j >= 3) {
      // rotation part
      return get_rotation_expr().get_value_at(i - 3, j - 3);
    }
    else if (i >= 3 && j < 3) {
      // -Erx
      // we need to compute matmul entry for -Erx
      return get_minus_E_rcross_expr().get_value_at(i - 3, j);
    }
    else {
      return 0;
    }
  }

  virtual int is_nonzero(size_t i, size_t j) const override {
    if (i < 3 && j < 3) {
      // rotation part
      return get_rotation_expr().is_nonzero(i, j);
    }
    else if (i >= 3 && j >= 3) {
      // rotation part
      return get_rotation_expr().is_nonzero(i - 3, j - 3);
    }
    else if (i >= 3 && j < 3) {
      // -Erx
      return get_minus_E_rcross_expr().is_nonzero(i - 3, j);
    }
    else {
      return false;
    }
  }

  virtual const std::vector<size_t> get_expr_shape(void) const override {
    return std::vector<size_t>({6, 6});
  }

  virtual const Matrix_expr<Scalar> &get_rotation_expr() const {
    return *new Matrix_expr<Scalar>();
  }
  virtual const Translation_expr<Scalar> &get_translation_expr() const {
    return *new Translation_expr<Scalar>();
  }
  virtual const Matrix_expr<Scalar> &get_minus_E_rcross_expr() const {
    return *new Matrix_expr<Scalar>();
  }

  virtual int has_rotation() const {
    return 0;
  }
  virtual int has_translation() const {
    return 0;
  }
};

// fwd decl
template <typename Scalar>
struct Packet_expr_leaf;

template <typename Scalar>
struct Translation {
private:
  // marked private because we want to make sure we track sparsity whenever
  // we assign some value to x, y, or z with an explicit setter
  dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>> x;//CAMBIO
  dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>> y;//CAMBIO
  dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>> z;//CAMBIO

public:
  static_var<int> has_x;
  static_var<int> has_y;
  static_var<int> has_z;

  Translation() : has_x(true), has_y(true), has_z(true) {
    x.setZero();;
    y.setZero();;
    z.setZero();;
  }

  Translation(Scalar _x, Scalar _y, Scalar _z) {
    set_x(_x);
    set_y(_y);
    set_z(_z);
  }

  void set_x(Scalar val) {
    if (std::abs(val) < 1e-5) {
      has_x = false;
      x.setZero();//CAMBIO
      return;
    }
    has_x = true;
    x.setConstant(val);//CAMBIO
  }

  void set_x(EigenMatrix<Scalar,flags.size_subpacket,1>& val) {//NUEVO
    has_x = true;
    x=val;
  }

  void set_y(Scalar val) {
    if (std::abs(val) < 1e-5) {
      has_y = false;
      y.setZero();//CAMBIO
      return;
    }
    has_y = true;
    y.setConstant(val);//CAMBIO
  }

  void set_y(EigenMatrix<Scalar,flags.size_subpacket,1>& val) {//NUEVO
    has_y = true;
    y=val;
  }

  void set_z(Scalar val) {
    if (std::abs(val) < 1e-5) {
      has_z = false;
      z.setZero();//CAMBIO
      return;
    }
    has_z = true;
    z.setConstant(val);//CAMBIO
  }

  void set_z(EigenMatrix<Scalar,flags.size_subpacket,1>& val) {//NUEVO
    has_z = true;
    z=val;
  }

  // the variable returned here is marked const so no one can modify the dyn_var directly
  const dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>> get_x() const { //CAMBIO
    if (has_x)
      return x;
    else
      dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>> a.setZero();//CAMBIO
      return a;
  }
  const dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>> get_y() const {
    if (has_y)
      return y;
    else
      dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>> a.setZero();//CAMBIO
      return a;
  }
  const dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>> get_z() const {
    if (has_z)
      return z;
    else
      dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>> a.setZero();//CAMBIO
      return a;
  }

  void set_prismatic_axis(char axis) {
    has_x = false;
    has_y = false;
    has_z = false;

    if (axis == 'X')
      has_x = true;
    else if (axis == 'Y')
      has_y = true;
    else if (axis == 'Z') {
      has_z = true;
    }
  }

  void jcalc(const dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>>& q_i) {//CAMBIO
    if (has_x)
      x = q_i;
    if (has_y)
      y = q_i;
    if (has_z)
      z = q_i;
  }

  void operator=(const Translation_expr<Scalar>& rhs) {//CAMBIO
    if (rhs.has_x()) {
      has_x = true;
      x = rhs.get_x();
    }
    if (rhs.has_y()) {
      has_y = true;
      y = rhs.get_y();
    }
    if (rhs.has_z()) {
      has_z = true;
      z = rhs.get_z();
    }
  }
};

template <typename Scalar>
struct Matrix {
  Storage<Scalar> storage;
  const size_t n_rows;
  const size_t n_cols;

  Matrix(size_t _n_rows, size_t _n_cols, Sparsity_type_id _sti = flags.sparsity_type_id,
         typename Storage<Scalar>::Storage_order_id _soi = Storage<Scalar>::COL_MAJ)
      : storage(_n_rows, _n_cols, _sti, _soi), n_rows(_n_rows), n_cols(_n_cols) {}

  void set_entry_to_constant(size_t i, size_t j, Scalar val) {
    storage.set_entry_to_constant(i, j, val);
  }

  void set_identity() {
    for (static_var<size_t> i = 0; i < n_rows; i = i + 1) {
      for (static_var<size_t> j = 0; j < n_cols; j = j + 1) {
        if (i == j)
          set_entry_to_constant(i, j, 1);
        else
          set_entry_to_constant(i, j, 0);
      }
    }
  }

  void operator=(const Matrix_expr<Scalar> &rhs) {
    if (storage.sparsity_type_id == SPARSE_UNROLLED) {
      for (static_var<size_t> i = 0; i < n_rows; i = i + 1) {
        for (static_var<size_t> j = 0; j < n_cols; j = j + 1) {
          if (rhs.is_nonzero(i, j)) {
            // todo: we currently mark the entry as dyn even when the assigned value
            // is a non-zero constant.
            // We don't propagate any non-zero constants.
            // Change later.
            storage.set_entry_to_dyn(i, j, rhs.get_value_at(i, j));
          }
          else {
            storage.set_entry_to_constant(i, j, 0);
          }
        }
      }
    }
    else if (storage.sparsity_type_id == DENSE) {
      storage.set_matrix(rhs.get_value());
    }
  }
};

template <typename Scalar> //CAMBIO
struct Rotation : public Matrix<EigenMatrix<Scalar>> { //CAMBIO
  using Matrix<EigenMatrix<Scalar,flags.size_subpacket,1>>::storage; //CAMBIO

  dyn_var<EigenMatrix<double,flags.size_subpacket,1>> sinq;
  dyn_var<EigenMatrix<double,flags.size_subpacket,1>> cosq;

  static_var<int> is_joint_packet;
  static_var<int> has_x;
  static_var<int> has_y;
  static_var<int> has_z;

  Rotation()
      : Matrix<EigenMatrix<Scalar,flags.size_subpacket,1>>(3, 3), is_joint_packet(false), has_x(false), has_y(false), has_z(false) {//CAMBIO
    Matrix<EigenMatrix<Scalar,flags.size_subpacket,1>>::set_identity();//CAMBIO
  }

  void set_revolute_axis(char axis) {
    is_joint_packet = true;
    if (axis == 'X') {
      has_x = true;
    }
    else if (axis == 'Y') {
      has_y = true;
    }
    else if (axis == 'Z') {
      has_z = true;
    }
  }

  void jcalc(const dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>> &q_i) { //CAMBIO
    // Featherstone, Table 2.2
    sinq = backend::sin(q_i);
    cosq = backend::cos(q_i);

    assertm(is_joint_packet == true, "can't jcalc on a non-joint packet");

    if (has_x) {
      storage.set_entry_to_dyn(1, 1, cosq);
      storage.set_entry_to_dyn(1, 2, sinq);
      storage.set_entry_to_dyn(2, 1, -sinq);
      storage.set_entry_to_dyn(2, 2, cosq);
    }
    else if (has_y) {
      storage.set_entry_to_dyn(0, 0, cosq);
      storage.set_entry_to_dyn(0, 2, -sinq);
      storage.set_entry_to_dyn(2, 0, sinq);
      storage.set_entry_to_dyn(2, 2, cosq);
    }
    else if (has_z) {
      storage.set_entry_to_dyn(0, 0, cosq);
      storage.set_entry_to_dyn(0, 1, sinq);
      storage.set_entry_to_dyn(1, 0, -sinq);
      storage.set_entry_to_dyn(1, 1, cosq);
    }
  }

  //void operator= (const Rotation_expr<Scalar> &rhs) {
  //  if (rhs.has_x()) {
  //    has_x = true;
  //  }
  //  if (rhs.has_y()) {
  //    has_y = true;
  //  }
  //  if (rhs.has_z()) {
  //    has_z = true;
  //  }

  //  Matrix<Scalar>::operator=(rhs);
  //}
  using Matrix<Scalar>::operator=;
};

template <typename Scalar>
struct Packet {
  Rotation<Scalar> rot;
  Translation<Scalar> trans;

  Matrix<EigenMatrix<Scalar,flags.size_subpacket,1>> minus_E_rcross;

  static_var<int> is_joint_packet;
  static_var<int> has_rotation;
  static_var<int> has_translation;

  Packet() : minus_E_rcross(3, 3), has_rotation(true), has_translation(true) {}//CAMBIO

  // if set_revolute/prismatic funcs are being called, it means this Packet is associated
  // with a joint.
  // A joint cannot be both prismatic and revolute.
  // We enforce this condition using an assert.
  void set_revolute_axis(char axis) {
    is_joint_packet = true;
    has_translation = false;
    has_rotation = true;
    rot.set_revolute_axis(axis);
  }
  void set_prismatic_axis(char axis) {
    is_joint_packet = true;
    has_rotation = false;
    has_translation = true;
    trans.set_prismatic_axis(axis);
  }
  void jcalc(const dyn_var<EigenMatrix<Scalar,flags.size_subpacket,1>>& q_i) {//CAMBIO
    if (has_rotation) {
      rot.jcalc(q_i);
    }
    else if (has_translation) {
      trans.jcalc(q_i);
    }
    // todo: trans should be trans-cross, do later
    minus_E_rcross = -rot * trans; //FALTA DEFINIR EL OPERADOR
  }

  void operator=(const Packet_expr<Scalar> &rhs) {
    if (rhs.has_rotation()) {
      has_rotation = true;
      rot = rhs.get_rotation_expr();
    }
    if (rhs.has_translation()) {
      has_translation = true;
      minus_E_rcross = rhs.get_minus_E_rcross_expr();
    }
  }

  void operator=(const Packet<Scalar> &packet) {
    *this = Packet_expr_leaf<Scalar>(packet);
  }
};

// Expressions

template <typename Scalar>
struct Matrix_expr_leaf : public Matrix_expr<Scalar> {
  const struct Matrix<Scalar> &m_mat;
  std::vector<size_t> expr_shape;

  Matrix_expr_leaf(const struct Matrix<Scalar> &mat) : m_mat(mat) {
    expr_shape.push_back(m_mat.n_rows);
    expr_shape.push_back(m_mat.n_cols);
  }

  const builder::builder get_value() const override {
    return m_mat.storage.denseify();
  }

  const builder::builder get_value_at(size_t i, size_t j) const override {
    return m_mat.storage.get_entry(i, j);
  }

  const std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  int is_nonzero(size_t i, size_t j) const override {
    return m_mat.storage.is_nonzero(i, j);
  }
};

template <typename Scalar>
struct Matrix_expr_unary_minus : public Matrix_expr<Scalar> {
  const struct Matrix_expr<Scalar> &expr1;

  std::vector<size_t> expr_shape;

  Matrix_expr_unary_minus(const struct Matrix_expr<Scalar> &expr1) : expr1(expr1) {
    expr_shape = expr1.get_expr_shape();
  }

  const std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  const builder::builder get_value() const override {
    return -expr1.get_value();
  }

  const builder::builder get_value_at(size_t i, size_t j) const override {
    return -expr1.get_value_at(i, j);
  }

  int is_nonzero(size_t i, size_t j) const override {
    return expr1.is_nonzero(i, j);
  }
};

template <typename Scalar>
struct Matrix_expr_add : public Matrix_expr<Scalar> {
  const struct Matrix_expr<Scalar> &expr1;
  const struct Matrix_expr<Scalar> &expr2;

  std::vector<size_t> expr_shape;

  Matrix_expr_add(const struct Matrix_expr<Scalar> &expr1, const struct Matrix_expr<Scalar> &expr2)
      : expr1(expr1), expr2(expr2) {
    std::vector<size_t> shape1 = expr1.get_expr_shape();
    std::vector<size_t> shape2 = expr2.get_expr_shape();
    assertm(shape1[0] == shape2[0] && shape1[1] == shape2[1], "shapes must match");
    expr_shape.push_back(shape1[0]);
    expr_shape.push_back(shape1[1]);
  }

  const std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  const builder::builder get_value() const override {
    return expr1.get_value() + expr2.get_value();
  }

  const builder::builder get_value_at(size_t i, size_t j) const override {
    return expr1.get_value_at(i, j) + expr2.get_value_at(i, j);
  }

  int is_nonzero(size_t i, size_t j) const override {
    return expr1.is_nonzero(i, j) || expr2.is_nonzero(i, j);
  }
};

template <typename Scalar>
struct Matrix_expr_mul : public Matrix_expr<Scalar> {
  const struct Matrix_expr<Scalar> &expr1;
  const struct Matrix_expr<Scalar> &expr2;

  std::vector<size_t> expr_shape;

  Matrix_expr_mul(const struct Matrix_expr<Scalar> &expr1, const struct Matrix_expr<Scalar> &expr2)
      : expr1(expr1), expr2(expr2) {
    std::vector<size_t> shape1 = expr1.get_expr_shape();
    std::vector<size_t> shape2 = expr2.get_expr_shape();
    assertm(shape1[1] == shape2[0], "inner dim of matmul expr must match");
    expr_shape.push_back(shape1[0]);
    expr_shape.push_back(shape2[1]);
  }

  const std::vector<size_t> get_expr_shape() const override {
    return expr_shape;
  }

  const builder::builder get_value() const override {
    return expr1.get_value() * expr2.get_value();
  }

  const builder::builder get_value_at(size_t i, size_t j) const override {
    const size_t inner_dim = expr1.get_expr_shape()[1];
    dyn_var<Scalar> sum = 0;
    // k is inner_dim for matmul
    for (static_var<size_t> k = 0; k < inner_dim; k = k + 1) {
      sum += expr1.get_value_at(i, k) * expr2.get_value_at(k, j);
    }
    return sum;
  }

  int is_nonzero(size_t i, size_t j) const override {
    const size_t inner_dim = expr1.get_expr_shape()[1];
    for (static_var<size_t> k = 0; k < inner_dim; k = k + 1) {
      // when summing up products of inner_dim, if any one product is nonzero
      // then (i, j) is guaranteed to be nonzero.
      if (expr1.is_nonzero(i, k) && expr2.is_nonzero(k, j))
        return true;
    }
    return false;
  }
};

template <typename Scalar>
struct Translation_expr_leaf : public Translation_expr<Scalar> {
  const struct Translation<Scalar> &m_trans;

  Translation_expr_leaf(const struct Translation<Scalar> &trans) : m_trans(trans) {}

  const builder::builder get_x() const override {
    return m_trans.get_x();
  }
  const builder::builder get_y() const override {
    return m_trans.get_y();
  }
  const builder::builder get_z() const override {
    return m_trans.get_z();
  }

  int has_x() const override {
    return m_trans.has_x;
  }
  int has_y() const override {
    return m_trans.has_y;
  }
  int has_z() const override {
    return m_trans.has_z;
  }
};

template <typename Scalar>
struct Translation_expr_add : public Translation_expr<Scalar> {
  const struct Translation_expr<Scalar> &expr1;
  const struct Translation_expr<Scalar> &expr2;

  Translation_expr_add(const struct Translation_expr<Scalar> &expr1,
                       const struct Translation_expr<Scalar> &expr2)
      : expr1(expr1), expr2(expr2) {}

  const builder::builder get_x() const override {
    return expr1.get_x() + expr2.get_x();
  }
  const builder::builder get_y() const override {
    return expr1.get_y() + expr2.get_y();
  }
  const builder::builder get_z() const override {
    return expr1.get_z() + expr2.get_z();
  }

  int has_x() const override {
    return expr1.has_x() || expr2.has_x();
  }
  int has_y() const override {
    return expr1.has_y() || expr2.has_y();
  }
  int has_z() const override {
    return expr1.has_z() || expr2.has_z();
  }
};

//template<typename Scalar>
//struct Rotation_expr_leaf : public Rotation_expr<Scalar> {
//  const struct Rotation<Scalar>& m_rot;
//
//  Rotation_expr_leaf(const struct Rotation<Scalar>& rot) : m_rot(rot) {}
//
//  const builder::builder get_value_at(size_t i, size_t j) const override {
//    if (m_rot.storage.is_constant(i, j))
//      return m_rot.storage.get_constant_entry(i, j);
//    else
//      return m_rot.storage.get_dyn_entry(i, j);
//  }
//
//  int is_nonzero(size_t i, size_t j) const override {
//    return m_rot.storage.is_nonzero(i, j);
//  }
//
//  int has_x() const override {
//    return m_rot.has_x;
//  }
//  int has_y() const override {
//    return m_rot.has_y;
//  }
//  int has_z() const override {
//    return m_rot.has_z;
//  }
//};
//
//template<typename Scalar>
//struct Rotation_expr_mul : public Rotation_expr<Scalar> {
//  const struct Rotation_expr<Scalar>& expr1;
//  const struct Rotation_expr<Scalar>& expr2;
//
//  Rotation_expr_mul(const struct Rotation_expr<Scalar>& expr1, const struct Rotation_expr<Scalar>& expr2) :
//    expr1(expr1), expr2(expr2) {}
//
//  const builder::builder get_value_at(size_t i, size_t j) const override {
//    dyn_var<Scalar> sum = 0;
//    // k is inner_dim for matmul
//    for (static_var<size_t> k = 0; k < 3; k = k + 1) {
//      sum += expr1.get_value_at(i, k) * expr2.get_value_at(k, j);
//    }
//    return sum;
//  }
//
//  int is_nonzero(size_t i, size_t j) const override {
//    for (static_var<size_t> k = 0; k < 3; k = k + 1) {
//      // when summing up products of inner_dim, if any one product is nonzero
//      // then (i, j) is guaranteed to be nonzero.
//      if (expr1.is_nonzero(i, k) && expr2.is_nonzero(k, j))
//        return true;
//    }
//    return false;
//  }
//
//  int has_x() const override {
//    return expr1.has_x() || expr2.has_x();
//  }
//  int has_y() const override {
//    return expr1.has_y() || expr2.has_y();
//  }
//  int has_z() const override {
//    return expr1.has_z() || expr2.has_z();
//  }
//};

template <typename Scalar>
struct Packet_expr_leaf : public Packet_expr<Scalar> {
  const struct Packet<Scalar> &m_packet;

  Packet_expr_leaf(const struct Packet<Scalar> &packet) : m_packet(packet) {}

  const Matrix_expr<Scalar> &get_rotation_expr() const override {
    return *new Matrix_expr_leaf<Scalar>(m_packet.rot);
  }
  const Translation_expr<Scalar> &get_translation_expr() const override {
    return *new Translation_expr_leaf<Scalar>(m_packet.trans);
  }
  const Matrix_expr<Scalar> &get_minus_E_rcross_expr() const override {
    return *new Matrix_expr_leaf<Scalar>(m_packet.minus_E_rcross);
  }

  int has_rotation() const override {
    return m_packet.has_rotation;
  }
  int has_translation() const override {
    return m_packet.has_translation;
  }
};

template <typename Scalar>
struct Packet_expr_mul : public Packet_expr<Scalar> {
  const struct Packet_expr<Scalar> &expr1;
  const struct Packet_expr<Scalar> &expr2;

  Packet_expr_mul(const struct Packet_expr<Scalar> &expr1, const struct Packet_expr<Scalar> &expr2)
      : expr1(expr1), expr2(expr2) {}

  const Matrix_expr<Scalar> &get_rotation_expr() const override {
    return expr1.get_rotation_expr() * expr2.get_rotation_expr();
  }
  const Matrix_expr<Scalar> &get_minus_E_rcross_expr() const override {
    return expr1.get_minus_E_rcross_expr() * expr2.get_rotation_expr() +
           expr1.get_rotation_expr() * expr2.get_minus_E_rcross_expr();
  }

  int has_rotation() const override {
    return expr1.has_rotation() || expr2.has_rotation();
  }
  int has_translation() const override {
    return expr1.has_translation() || expr2.has_translation();
  }
};

} // namespace ctup

#endif
