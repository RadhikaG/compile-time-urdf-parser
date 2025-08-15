#include "backend.h"
#include "matrix_layout.h"
#include "matrix_operators.h"

#include "blocks/c_code_generator.h"
#include "builder/builder_context.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "builder/static_var.h"

#include <iostream>
#include <fstream>

const size_t N_X_T = 1;

const double data[N_X_T][16] = {
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.42, 0.69, 0.33, 1}
};
using builder::dyn_var;
using builder::static_var;

namespace ctup {

template <typename Prim>
struct Xform : public matrix_layout<Prim> {
  using matrix_layout<Prim>::set_entry_to_constant;
  using matrix_layout<Prim>::set_entry_to_nonconstant;
  dyn_var<Prim> sinq;
  dyn_var<Prim> cosq;

  static_var<int> joint_type;
  static_var<int> joint_xform_axis;

  Xform() : matrix_layout<Prim>(4, 4, SPARSE, UNROLLED, UNCOMPRESSED) {
    matrix_layout<Prim>::set_identity();
  }

  void set_revolute_axis(char axis) {
    assert((axis == 'X' || axis == 'Y' || axis == 'Z') && "axis must be X,Y,Z");
    joint_xform_axis = axis;
    joint_type = 'R';
  }

  void set_prismatic_axis(char axis) {
    assert((axis == 'X' || axis == 'Y' || axis == 'Z') && "axis must be X,Y,Z");
    joint_xform_axis = axis;
    joint_type = 'P';
  }

  void jcalc(dyn_var<Prim> &q_i) {
    if constexpr (ctup::is_vamp_float_vector_type<Prim>::value) {
      q_i.sin();
      q_i.cos();
    }
    else {
      sinq = ctup::backend::sin<Prim>(q_i);
      cosq = ctup::backend::cos<Prim>(q_i);
    }

    if (joint_type == 'R') {
      // switching to pinocchio transpose repr because
      // regular featherstone notation isn't working with
      // homogeneous transforms for some godforesaken reason.
      // suspect it has sth to do with the X_T that pinocchio
      // is spitting out.
      // reverse the signs of the sines for feath (hah)
      if (joint_xform_axis == 'X') {
        set_entry_to_nonconstant(1, 1, cosq);
        set_entry_to_nonconstant(1, 2, -sinq);
        set_entry_to_nonconstant(2, 1, sinq);
        set_entry_to_nonconstant(2, 2, cosq);
      }
      else if (joint_xform_axis == 'Y') {
        set_entry_to_nonconstant(0, 0, cosq);
        set_entry_to_nonconstant(0, 2, sinq);
        set_entry_to_nonconstant(2, 0, -sinq);
        set_entry_to_nonconstant(2, 2, cosq);
      }
      else if (joint_xform_axis == 'Z') {
        set_entry_to_nonconstant(0, 0, cosq);
        set_entry_to_nonconstant(0, 1, -sinq);
        set_entry_to_nonconstant(1, 0, sinq);
        set_entry_to_nonconstant(1, 1, cosq);
      }
    }
    else if (joint_type == 'P') {
      if (joint_xform_axis == 'X') {
        set_entry_to_nonconstant(0, 3, q_i);
      }
      else if (joint_xform_axis == 'Y') {
        set_entry_to_nonconstant(1, 3, q_i);
      }
      else if (joint_xform_axis == 'Z') {
        set_entry_to_nonconstant(2, 3, q_i);
      }
    }
    else {
      assert(false && "jcalc called on non joint xform or joint unset");
    }
  }

  using matrix_layout<Prim>::operator=;
};

}

using ctup::backend::blaze_avx256f;
using ctup::backend::vamp_avx256f;
using ctup::Xform;

template <typename Prim>
static void set_X_T(Xform<Prim> &X_T, bool const_prop) {
  static_var<int> r;
  static_var<int> c;

  Eigen::Matrix<double, 4, 4> pin_X_T(data[0]);

  for (r = 0; r < 4; r = r + 1) {
    for (c = 0; c < 4; c = c + 1) {
      double entry = pin_X_T.coeffRef(r, c);
      if (const_prop)
        X_T.set_entry_to_constant(r, c, entry);
      else
        X_T.set_entry_to_nonconstant(r, c, entry);
    }
  }
}

static dyn_var<ctup::EigenMatrix<float, 4, 4>> scalar_mm(dyn_var<float> q, static_var<int> const_prop) {
  Xform<float> X_T, X_J, X_0;

  set_X_T(X_T, const_prop);
  X_J.set_revolute_axis('Z');
  X_J.jcalc(q);

  X_0 = X_T * X_J;

  return X_0.denseify();
}

static dyn_var<ctup::BlazeStaticMatrix<blaze_avx256f, 4, 4>> batched_mm_blaze(dyn_var<blaze_avx256f> q, static_var<int> const_prop) {
  Xform<blaze_avx256f> X_T, X_J, X_0;

  set_X_T(X_T, const_prop);
  X_J.set_revolute_axis('Z');
  X_J.jcalc(q);

  X_0 = X_T * X_J;

  dyn_var<ctup::BlazeStaticMatrix<blaze_avx256f>> final_ans;
  final_ans.set_matrix_fixed_size(4, 4);

  for (static_var<size_t> i = 0; i < 4; i = i+1) {
    for (static_var<size_t> j = 0; j < 4; j = j+1) {
      final_ans[i][j] = X_0.get_entry(i, j);
    }
  }

  return final_ans;
}

static dyn_var<ctup::VampFloatVector<8, 16>> batched_mm_intrin(dyn_var<vamp_avx256f> q, static_var<int> const_prop) {
  Xform<vamp_avx256f> X_T, X_J, X_0;

  set_X_T(X_T, const_prop);
  X_J.set_revolute_axis('Z');
  X_J.jcalc(q);

  X_0 = X_T * X_J;

  dyn_var<ctup::VampFloatVector<8, 16>> final_ans;

  for (static_var<size_t> i = 0; i < 4; i = i+1) {
    for (static_var<size_t> j = 0; j < 4; j = j+1) {
      final_ans[i*4 + j] = X_0.get_entry(i, j);
    }
  }

  return final_ans;
}


int main(int argc, char* argv[]) {
  const std::string header_filename = (argc <= 1) ? "./fk_gen.h" : argv[1];
  std::cout << header_filename << "\n";

  std::ofstream of(header_filename);
  block::c_code_generator codegen(of);

  of << "// clang-format off\n\n";
  of << "#include \"Eigen/Dense\"\n\n";
  of << "#include \"blaze/Math.h\"\n\n";
  of << "#include \"vamp/vector.hh\"\n\n";
  of << "#include <iostream>\n\n";
  of << "namespace ctup_gen {\n\n";

  of << "static void print_string(const char* str) {\n";
  of << "  std::cout << str << \"\\n\";\n";
  of << "}\n\n";

  //----- todo fix print funcs

  of << "static void print_matrix(const blaze::StaticMatrix<blaze::StaticVector<double, 8>, 6, 6>& matrix) {\n";
  of << "  for (int i = 0; i < 6; i++) {\n";
  of << "    for (int j = 0; j < 6; j++) {\n";
  of << "      blaze::StaticVector<double, 8> colVec(matrix(i, j));\n";
  of << "      blaze::StaticVector<double, 8, blaze::rowVector> rowVec;\n";
  of << "      for (int k = 0; k < 8; k++)\n";
  of << "        rowVec[k] = colVec[k];\n";
  of << "      std::cout << rowVec;\n";
  of << "    }\n";
  of << "  }\n";
  of << "}\n";

  of << "template<typename MT>\n";
  of << "static void print_matrix(const blaze::DenseMatrix<MT, blaze::rowMajor>& matrix) {\n";
  of << "  std::cout << matrix << \"\\n\";\n";
  of << "}\n\n";

  of << "template<typename Derived>\n";
  of << "static void print_matrix(const Eigen::MatrixBase<Derived>& matrix) {\n";
  of << "  std::cout << matrix << \"\\n\";\n";
  of << "}\n\n";

  //-------------------------

  builder::builder_context context;

  //// without const prop

  auto ast = context.extract_function_ast(scalar_mm, "scalar_mm", false);
  of << "static ";
  block::c_code_generator::generate_code(ast, of, 0);

  ast = context.extract_function_ast(batched_mm_blaze, "batched_mm_blaze", false);
  of << "static ";
  block::c_code_generator::generate_code(ast, of, 0);

  ast = context.extract_function_ast(batched_mm_intrin, "batched_mm_intrin", false);
  of << "static ";
  block::c_code_generator::generate_code(ast, of, 0);

  //// with const prop

  ast = context.extract_function_ast(scalar_mm, "scalar_mm_constprop", true);
  of << "static ";
  block::c_code_generator::generate_code(ast, of, 0);

  ast = context.extract_function_ast(batched_mm_blaze, "batched_mm_blaze_constprop", true);
  of << "static ";
  block::c_code_generator::generate_code(ast, of, 0);

  ast = context.extract_function_ast(batched_mm_intrin, "batched_mm_intrin_constprop", true);
  of << "static ";
  block::c_code_generator::generate_code(ast, of, 0);

  of << "}\n";
}
