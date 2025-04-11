// Include the headers
#include "blocks/c_code_generator.h"
#include "builder/forward_declarations.h"
#include "builder/static_var.h"
#include "builder/builder_base.h"
#include "builder/dyn_var.h"
#include <iostream>

// Include the BuildIt types
using builder::dyn_var;
using builder::static_var;
using builder::as_member;

/***** Eigen backend *****/

namespace builder {
static const char eigen_matrix_t_name[] = "Eigen::Matrix";

template <typename Scalar>
using EigenMatrix = name<eigen_matrix_t_name, Scalar>;

template <typename Scalar>
class dyn_var<EigenMatrix<Scalar>> : public dyn_var_impl<EigenMatrix<Scalar>> {
public:
  typedef dyn_var_impl<EigenMatrix<Scalar>> super;
  using super::super;
  using super::operator=;
  builder operator=(const dyn_var<EigenMatrix<Scalar>> &t) {
    return (*this) = (builder)t;
  }
  
  dyn_var() : dyn_var_impl<EigenMatrix<Scalar>>() {}

  // convenience getter variables
  size_t n_rows;
  size_t n_cols;

  void set_matrix_fixed_size(size_t _n_rows, size_t _n_cols) {
    auto type = block::to<block::named_type>(this->block_var->var_type);
    auto d1 = block::to<block::named_type>(type->template_args[1]);
    auto d2 = block::to<block::named_type>(type->template_args[2]);
    d1->type_name = std::to_string(_n_rows);
    d2->type_name = std::to_string(_n_cols);

    // setting convenience getter variables
    n_rows = _n_rows;
    n_cols = _n_cols;
  }
  
  dyn_var(size_t n_rows, size_t n_cols) : dyn_var_impl<EigenMatrix<Scalar>>() {
    // sets stage 2 compile-time n_rows, n_cols in emitted Eigen::Matrix<Scalar, n_rows, n_cols> type 
    set_matrix_fixed_size(n_rows, n_cols);
  }
  
  dyn_var(const dyn_var &t) : dyn_var_impl<EigenMatrix<Scalar>>((builder)t) {
    // want n_rows, n_cols template args to be copied correctly
    this->block_var->var_type = block::clone(t.block_var->var_type);
  }
   
  dyn_var<double (size_t, size_t)> coeffRef = as_member(this, "coeffRef");
  dyn_var<void (void)> setZero = as_member(this, "setZero");
};
}

/***** Data structures for emitting Eigen backend code *****/

struct Matrix {
  size_t n;
  dyn_var<builder::EigenMatrix<double>> m_matrix = builder::defer_init();
  std::vector<std::vector<size_t>> sparsity_pattern;

  Matrix(bool as_global=false) {
    if (as_global)
      m_matrix = builder::as_global("mat");
  }

  void set_size(size_t _n) {
    n = _n;
    m_matrix.set_matrix_fixed_size(n, n);
    for (static_var<size_t> i = 0; i < n; i = i+1) {
      sparsity_pattern.push_back({});
      sparsity_pattern[i].resize(n);
    }
  }

  Matrix(size_t _n) {
    set_size(_n);
  }

  void set_diagonal_to_nonzero() {
    for (static_var<size_t> i = 0; i < n; i = i+1) {
      for (static_var<size_t> j = 0; j < n; j = j+1) {
        if (i == j)
          sparsity_pattern[i][j] = 1;
        else
          sparsity_pattern[i][j] = 0;
      }
    }
  }

  void set_var_along_diagonal(dyn_var<double> &diag_var) {
    // sets all diagonal entries of matrix to `diag_var`
    for (static_var<size_t> i = 0; i < n; i = i+1) {
      for (static_var<size_t> j = 0; j < n; j = j+1) {
        if (i == j)
          m_matrix.coeffRef(i, j) = diag_var;
      }
    }
  }
  
  builder::builder get_entry(size_t i, size_t j) const {
    if (sparsity_pattern[i][j] == 0)
      return 0;
    else
      return const_cast<dyn_var<builder::EigenMatrix<double>>&>(m_matrix).coeffRef(i, j);
  }
};

//static builder::builder some_matrix_op(dyn_var<builder::EigenMatrix<double>> &mat) {
//  mat.coeffRef(0, 0) += 1;
//  return mat;
//}

/***** "DSL" frontend code ****/
// Contrived function that:
// 1. Takes a `diag_var` as input, along with some static_var 2D matrix index {i, j}
// 2. Sets all the diagonal elements in the matrix to `diag_var`
// 3. Returns entry {i, j} of matrix

static dyn_var<double> set_matrix_diag_var(dyn_var<double> diag_var, static_var<size_t> n) {
  Matrix mat(n);
  mat.set_diagonal_to_nonzero();
  mat.set_var_along_diagonal(diag_var);

  for (static_var<size_t> i = 0; i < n; i = i+1) {
    for (static_var<size_t> j = 0; j < n; j = j+1) {
      if (mat.get_entry(i, j) == 1)
        return 1;
    }
  }
  return 0;
}

/**** ======= PERSISTENT STRUCT ========= ****/

struct persistent_data {
  Matrix mat;

  persistent_data(bool as_global) : mat(as_global) {}

  void set_size(size_t n) {
    mat.set_size(n);
  }
};
persistent_data pd(true);

static dyn_var<double> bar(dyn_var<double> diag_var, static_var<size_t> n) {
  pd.set_size(n);

  pd.mat.set_diagonal_to_nonzero();
  pd.mat.set_var_along_diagonal(diag_var);

  for (static_var<size_t> i = 0; i < n; i = i+1) {
    for (static_var<size_t> j = 0; j < n; j = j+1) {
      if (pd.mat.get_entry(i, j) == 1)
        return 1;
    }
  }
  return 0;
}

/**** ======= MAIN ========= ****/

int main(int argc, char* argv[]) {
  builder::builder_context context;
  //auto ast = context.extract_function_ast(set_matrix_diag_var, "set_matrix_diag_var", 4);
  block::c_code_generator::generate_struct_decl<dyn_var<persistent_data>>(std::cout);
  auto ast = context.extract_function_ast(bar, "bar", 4);
  block::c_code_generator::generate_code(ast, std::cout, 0);
  return 0;
}


