#include "backend.h"
#include "matrix_layout.h"
#include "matrix_operators.h"

#include "blocks/c_code_generator.h"
#include "builder/builder_context.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "builder/static_var.h"
#include "builder/array.h"

#include <fstream>

const size_t N_X_T = 3;

const double data[N_X_T][36] = {
  {1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0.686,0,1,0,0,-0.686,0,0.06,0,1,0,0,-0.06,0,0,0,1},
  {0.707105,0.707108,0,0,0,0,-0.707108,0.707105,0,0,0,0,0,0,1,0,0,0,-0.0916596,0.0916593,-0.137886,0.707105,0.707108,0,-0.0916593,-0.0916596,0.228434,-0.707108,0.707105,0,0.259027,-0.0640272,0,0,0,1},
  {1,0,0,0,0,0,0,4.89664e-12,-1,0,0,0,0,1,4.89664e-12,0,0,0,0,0.27035,0,1,0,0,-1.32381e-12,0.069,3.37868e-13,0,4.89664e-12,-1,-0.27035,-3.37868e-13,0.069,0,1,4.89664e-12}
};
using builder::dyn_var;
using builder::static_var;

namespace ctup {

template <typename Prim>
struct Translation : public matrix_layout<Prim> {
  typedef std::shared_ptr<Translation<Prim>> Ptr;

  using matrix_layout<Prim>::set_entry_to_constant;
  using matrix_layout<Prim>::set_entry_to_nonconstant;

  Translation() : matrix_layout<Prim>(3, 3, SPARSE, FLATTENED, COMPRESSED) {
    matrix_layout<Prim>::set_zero();
  }

  dyn_var<Prim> x;
  dyn_var<Prim> y;
  dyn_var<Prim> z;

  static_var<int> joint_xform_axis;

  void jcalc(const dyn_var<Prim> &q_i) {
    if (joint_xform_axis == 'X') {
      set_entry_to_nonconstant(1, 2, q_i);
      set_entry_to_nonconstant(2, 1, -q_i);
    }
    else if (joint_xform_axis == 'Y') {
      set_entry_to_nonconstant(2, 0, q_i);
      set_entry_to_nonconstant(0, 2, -q_i);
    }
    else if (joint_xform_axis == 'Z') {
      set_entry_to_nonconstant(0, 1, q_i);
      set_entry_to_nonconstant(1, 0, -q_i);
    }
  }
};

template <typename Prim>
struct Rotation : public matrix_layout<Prim> {
  typedef std::shared_ptr<Rotation<Prim>> Ptr;

  using matrix_layout<Prim>::set_entry_to_constant;
  using matrix_layout<Prim>::set_entry_to_nonconstant;
  dyn_var<Prim> sinq;
  dyn_var<Prim> cosq;

  static_var<int> joint_xform_axis;

  Rotation() : matrix_layout<Prim>(3, 3, SPARSE, FLATTENED, COMPRESSED) {
    matrix_layout<Prim>::set_identity();
  }

  void jcalc(const dyn_var<Prim> &q_i) {
    sinq = backend::sin<Prim>(q_i);
    cosq = backend::cos<Prim>(q_i);

    if (joint_xform_axis == 'X') {
      set_entry_to_nonconstant(1, 1, cosq);
      set_entry_to_nonconstant(1, 2, sinq);
      set_entry_to_nonconstant(2, 1, -sinq);
      set_entry_to_nonconstant(2, 2, cosq);
    }
    else if (joint_xform_axis == 'Y') {
      set_entry_to_nonconstant(0, 0, cosq);
      set_entry_to_nonconstant(0, 2, -sinq);
      set_entry_to_nonconstant(2, 0, sinq);
      set_entry_to_nonconstant(2, 2, cosq);
    }
    else if (joint_xform_axis == 'Z') {
      set_entry_to_nonconstant(0, 0, cosq);
      set_entry_to_nonconstant(0, 1, sinq);
      set_entry_to_nonconstant(1, 0, -sinq);
      set_entry_to_nonconstant(1, 1, cosq);
    }
    else {
      assert(false && "jcalc called on non joint xform or joint unset");
    }
  }
};

template <typename Prim>
struct Xform : public blocked_layout<Prim> {
  using blocked_layout<Prim>::set_partitions;
  using blocked_layout<Prim>::set_new_block;
  using blocked_layout<Prim>::operator=;

  typename Rotation<Prim>::Ptr rot;
  typename Translation<Prim>::Ptr trans;
  typename matrix_layout<Prim>::Ptr minus_E_rcross;

  static_var<int> joint_type;

  Xform() : blocked_layout<Prim>(6, 6), 
    rot(new Rotation<Prim>()), trans(new Translation<Prim>()), 
    minus_E_rcross(new matrix_layout<Prim>(3, 3, SPARSE, FLATTENED, COMPRESSED)) {

    minus_E_rcross->set_zero();
    set_partitions({0, 3}, {0, 3});
    set_new_block(0, 0, rot);
    set_new_block(1, 1, rot);
    set_new_block(1, 0, minus_E_rcross);
  }

  void set_revolute_axis(char axis) {
    assert((axis == 'X' || axis == 'Y' || axis == 'Z') && "axis must be X,Y,Z");
    rot->joint_xform_axis = axis;
    joint_type = 'R';
  }

  void set_prismatic_axis(char axis) {
    assert((axis == 'X' || axis == 'Y' || axis == 'Z') && "axis must be X,Y,Z");
    trans->joint_xform_axis = axis;
    joint_type = 'P';
  }

  void jcalc(const dyn_var<Prim> &q_i) {
    if (joint_type == 'R')
      rot->jcalc(q_i);
    else
      trans->jcalc(q_i);

    *minus_E_rcross = -(*rot) * (*trans);
  }
};

}

/** helpers **/

using ctup::Xform;
using ctup::BlazeStaticVector;
using ctup::BlazeStaticMatrix;
using ctup::backend::blaze_avx512d;

builder::dyn_var<void(BlazeStaticMatrix<double> &)> print_matrix = builder::as_global("print_matrix");
builder::dyn_var<void(char *)> print_string = builder::as_global("print_string");

template <typename Scalar>
static void print_Xmat(std::string prefix, Xform<Scalar> &xform) {
  print_string(prefix.c_str());
  print_matrix(xform.denseify());
}

/** helpers end **/

template <typename Prim>
static void set_X_T(builder::array<Xform<Prim>> &X_T) {
  static_var<int> r;
  static_var<int> c;

  for (static_var<size_t> i = 1; i < N_X_T; i = i+1) {
    Eigen::Matrix<double, 6, 6> pin_X_T(data[i]);

    for (r = 0; r < 6; r = r + 1) {
      for (c = 0; c < 6; c = c + 1) {
        double entry = pin_X_T.coeffRef(c, r);
        X_T[i].set_entry_to_constant(r, c, entry);
      }
    }
  }
}

static void toRawMatrix(dyn_var<BlazeStaticMatrix<blaze_avx512d>> &raw_mat, Xform<blaze_avx512d> &xform) {
  static_var<int> r, c;

  for (r = 0; r < 6; r = r + 1)
    for (c = 0; c < 6; c = c + 1)
      raw_mat(r, c) = xform.get_entry(r, c);
}

static dyn_var<BlazeStaticMatrix<blaze_avx512d>> fk(dyn_var<BlazeStaticVector<blaze_avx512d> &> q) {

  builder::array<Xform<blaze_avx512d>> X_T;
  X_T.set_size(N_X_T);

  set_X_T(X_T);

  Xform<blaze_avx512d> X1, X2;

  X1.set_revolute_axis('Z');
  X1.jcalc(q[1]);

  X2 = X1 * X_T[1];

  print_Xmat("us X1:", X1);
  print_Xmat("us X_T[1]:", X_T[1]);
  print_Xmat("us X2:", X2);

  dyn_var<BlazeStaticMatrix<blaze_avx512d>> final_ans;
  final_ans.set_matrix_fixed_size(6, 6);

  toRawMatrix(final_ans, X2);

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

  auto ast = context.extract_function_ast(fk, "fk");
  of << "static ";
  block::c_code_generator::generate_code(ast, of, 0);

  of << "}\n";
}
