#include "backend.h"

#include "blocks/block_visitor.h"
#include "blocks/c_code_generator.h"
#include "builder/builder_base.h"
#include "builder/builder_context.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "builder/static_var.h"
#include "builder/lib/utils.h"
// ignore unused header warning in IDE, this is needed
#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "assert.h"
#include <memory>

#define CAST_TO_VECTOR_XD (dyn_var<builder::eigen_vectorXd_t>)(builder::cast)
#define CAST_TO_EIGEN_MATRIX (dyn_var<EigenMatrix<double>>)(builder::cast)

using builder::dyn_var;
using builder::static_var;
using ctup::EigenMatrix;

using namespace pinocchio;

static const char eigen_Xmat_t_name[] = "Eigen::Matrix<double, 6, 6>";
static const char eigen_vec_t_name[] = "Eigen::VectorXd";

using eigen_Xmat_t = builder::name<eigen_Xmat_t_name>;
using eigen_vectorXd_t = builder::name<eigen_vec_t_name>;

namespace runtime {
  builder::dyn_var<double (double)> sin = builder::as_global("sin");
  builder::dyn_var<double (double)> cos = builder::as_global("cos");
}

namespace builder {
template <>
class dyn_var<eigen_Xmat_t> : public dyn_var_impl<eigen_Xmat_t> {
public:
  typedef dyn_var_impl<eigen_Xmat_t> super;
  using super::super;
  using super::operator=;
  builder operator=(const dyn_var<eigen_Xmat_t> &t) {
    return (*this) = (builder)t;
  }
  dyn_var(const dyn_var &t) : dyn_var_impl((builder)t) {}
  dyn_var() : dyn_var_impl<eigen_Xmat_t>() {} 

  //dyn_var<double& (Eigen::MatrixXd::*)(Eigen::Index, Eigen::Index)> coeffRef = as_member(this, "coeffRef");
  dyn_var<double& (Eigen::Index, Eigen::Index)> coeffRef = as_member(this, "coeffRef");
  dyn_var<double& (Eigen::Index, Eigen::Index)> block = as_member(this, "block<3,3>"); // remove template params later
  dyn_var<void (void)> setZero = as_member(this, "setZero");
};

template <>
class dyn_var<eigen_Xmat_t[]> : public dyn_var_impl<eigen_Xmat_t[]> {
public:
  typedef dyn_var_impl<eigen_Xmat_t[]> super;
  using super::super;
  using super::operator=;
  builder operator=(const dyn_var<eigen_Xmat_t> &t) {
    return (*this) = (builder)t;
  }
  dyn_var(const dyn_var &t) : dyn_var_impl((builder)t) {}
  dyn_var() : dyn_var_impl<eigen_Xmat_t[]>() {} 

  // for indexing into MatrixXd[] arrays and using MatrixXd class methods
  dyn_var_mimic<eigen_Xmat_t> operator[](const builder &bt) {
    return (dyn_var_mimic<eigen_Xmat_t>)(cast)this->dyn_var_impl<eigen_Xmat_t[]>::operator[](bt);
  }
};
}

// We do our math the same way pinocchio does it, indexing is transposed from
// featherstone, and matmul order is reversed.
// This is because Eigen is col major.

dyn_var<eigen_Xmat_t[]> X_T = builder::as_global("X_T");

void set_X_T(const Model &model) {

  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i = 1;

  static_var<int> r = 0;
  static_var<int> c = 0;

  for (; i < (JointIndex)model.njoints; i = i+1) {
    Eigen::Matrix<double, 6, 6> tmp_cst_mat = model.jointPlacements[i];

    builder::annotate(std::string(model.names[i]));
    for (c = 0; c < 6; c = c + 1) {
      for (r = 0; r < 6; r = r + 1) {
        //((dyn_var<eigen_Xmat_t>)(builder::cast)X_T[i]).coeffRef(r, c) = blah.coeffRef(r, c);
        X_T[i].coeffRef(c, r) = tmp_cst_mat.coeffRef(c, r);
      }
    }
  }
}

static int get_jtype(const Model &model, Model::JointIndex i) {
  std::string joint_name = model.joints[i].shortname();

  bool is_revolute = joint_name.find("JointModelR") != std::string::npos;
  bool is_prismatic = joint_name.find("JointModelP") != std::string::npos;

  if (is_revolute)
    return 'R';
  if (is_prismatic)
    return 'P';
  else
    return 'N';
}

static int get_joint_axis(const Model &model, Model::JointIndex i) {
  std::string joint_name = model.joints[i].shortname();
  char axis = joint_name.back();

  switch(axis) {
    case 'X': return 'X';
    case 'Y': return 'Y';
    case 'Z': return 'Z';
    default: assert(false && "should never happen");
  }
}

dyn_var<EigenMatrix<double>> fk(const Model &model, dyn_var<EigenMatrix<double>> q) {
  dyn_var<EigenMatrix<double>[]> X_J;
  dyn_var<EigenMatrix<double>[]> X_0;
  
  resize_arr(X_J, model.njoints);
  resize_arr(X_0, model.njoints);

  static_var<JointIndex> j = 1;
  for (; j <(JointIndex)model.njoints; j = j+1) {
    X_0[j] = dyn_var<EigenMatrix<double>>(16, 36);
    X_J[j] = dyn_var<EigenMatrix<double>>(16, 36);
  }

  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i = 1;

  static_var<int> jtype;
  static_var<int> axis;

  dyn_var<EigenMatrix<double>> a(16,1);
  dyn_var<EigenMatrix<double>> b(16,1);
  dyn_var<EigenMatrix<double>> c(16,1);
  c.setConstant(1);

  for (; i < (JointIndex)model.njoints; i = i+1) {

    jtype = get_jtype(model, i);
    axis = get_joint_axis(model, i);
    (CAST_TO_EIGEN_MATRIX X_J[i]).setZero();

    builder::annotate(std::string(model.names[i]));

    a=(CAST_TO_VECTOR_XD(CAST_TO_VECTOR_XD q.col(i-1)).array()).cos();
    b=(CAST_TO_VECTOR_XD(CAST_TO_VECTOR_XD q.col(i-1)).array()).sin();

    if (jtype == 'R') {
      if (axis == 'X') {
        (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_J[i]).col(0))=c;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(7) = a;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(8) = -b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(13) = b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(14) = a;
        // not smart but doing nonetheless for symmetric E
        (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_J[i]).col(21))=c;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(28) = a;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(29) = -b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(34) = b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(35) = a;
      } 
      else if (axis == 'Y') {
        (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_J[i]).col(7))=c;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(0) = a;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(2) = b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(12) = -b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(14) = a;
        // not smart but doing nonetheless for symm E
        (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX  X_J[i]).col(28))=c;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(21) = a;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(23) = b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(33) = -b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(35) = a;
      } 
      else if (axis == 'Z') {
        (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_J[i]).col(14))=c;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(0) = a;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(1) = -b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(6) = b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(7) = a;
        // not smart but doing nonetheless for symm E
        (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX  X_J[i]).col(35))=c;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(21) = a;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(22) = -b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(27) = b;
        (CAST_TO_EIGEN_MATRIX X_J[i]).col(28) = a;
      } 
      // r = 0
    }
    else if (jtype == 'P') {
      // negative r-cross, opp signs of featherstone 2.23
      if (axis == 'X') {
        (CAST_TO_EIGEN_MATRIX  X_J[i]).col(8) = -(q.col(i-1));
        (CAST_TO_EIGEN_MATRIX  X_J[i]).col(13) = (q.col(i-1));
      }
      else if (axis == 'Y') {
        (CAST_TO_EIGEN_MATRIX  X_J[i]).col(2) = (q.col(i-1));
        (CAST_TO_EIGEN_MATRIX  X_J[i]).col(12) = -(q.col(i-1));
      }
      else if (axis == 'Z') {
        (CAST_TO_EIGEN_MATRIX  X_J[i]).col(1) = -(q.col(i-1));
        (CAST_TO_EIGEN_MATRIX  X_J[i]).col(6) = (q.col(i-1));
      }
      // E = Identity 
      (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_J[i]).col(0)).setConstant(1);
      (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_J[i]).col(7)).setConstant(1);
      (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_J[i]).col(14)).setConstant(1);
      // symm E
      (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_J[i]).col(21)).setConstant(1);
      (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_J[i]).col(28)).setConstant(1);
      (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_J[i]).col(35)).setConstant(1);   
    }
    else {
      assert(false && "joint type unsupported");
    }
  }

  static_var<JointIndex> parent;

  for (i = 1; i < (JointIndex)model.njoints; i = i+1) {
    dyn_var<EigenMatrix<double>> X_pi(16,36);
    builder::annotate(std::string(model.names[i]));

    dyn_var<int> l1;
    l1 = 0;
    for (; l1 < 36; l1 = l1+1) {
        X_pi.col(l1)  =  X_T[i].coeffRef(l1/6,0)*(CAST_TO_EIGEN_MATRIX X_J[i]).col(0*6+l1%6)
        + X_T[i].coeffRef(l1/6,1)*(CAST_TO_EIGEN_MATRIX X_J[i]).col(6+l1%6)
        + X_T[i].coeffRef(l1/6,2)*(CAST_TO_EIGEN_MATRIX X_J[i]).col(12+l1%6)
        + X_T[i].coeffRef(l1/6,3)*(CAST_TO_EIGEN_MATRIX X_J[i]).col(18+l1%6)
        + X_T[i].coeffRef(l1/6,4)*(CAST_TO_EIGEN_MATRIX X_J[i]).col(24+l1%6)
        + X_T[i].coeffRef(l1/6,5)*(CAST_TO_EIGEN_MATRIX X_J[i]).col(30+l1%6);
    }

    parent = model.parents[i];
    if (parent > 0) {
      dyn_var<int> l2;
        l2 = 0;
        for (; l2 < 36; l2 = l2+1) {
            ((CAST_TO_EIGEN_MATRIX X_0[i])).col(l2)  =  (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_0[parent]).col(0+6*(l2/6))).array()*(CAST_TO_VECTOR_XD X_pi.col(0+l2%6)).array()
            + (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_0[parent]).col(1+6*(l2/6))).array()*(CAST_TO_VECTOR_XD X_pi.col(6+l2%6)).array()
            + (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_0[parent]).col(2+6*(l2/6))).array()*(CAST_TO_VECTOR_XD X_pi.col(12+l2%6)).array()
            + (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_0[parent]).col(3+6*(l2/6))).array()*(CAST_TO_VECTOR_XD X_pi.col(18+l2%6)).array()
            + (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_0[parent]).col(4+6*(l2/6))).array()*(CAST_TO_VECTOR_XD X_pi.col(24+l2%6)).array()
            + (CAST_TO_VECTOR_XD(CAST_TO_EIGEN_MATRIX X_0[parent]).col(5+6*(l2/6))).array()*(CAST_TO_VECTOR_XD X_pi.col(30+l2%6)).array();
        }
    }
    else {
      X_0[i] = X_pi;
    }
  }

  return X_0[model.njoints-1];
}

int main(int argc, char* argv[]) {
  const std::string urdf_filename = argv[1];
  std::cout << urdf_filename << "\n";

  const std::string header_filename = (argc <= 2) ? "./fk_gen.h" : argv[2];
  std::cout << header_filename << "\n";

  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  std::ofstream of(header_filename);
  block::c_code_generator codegen(of);

  of << "#include \"Eigen/Dense\"\n\n";
  of << "namespace ctup_gen {\n\n";

  resize_arr(X_T, model.njoints);
  auto X_T_decl = std::make_shared<block::decl_stmt>();
  X_T_decl->decl_var = X_T.block_var;
  X_T_decl->accept(&codegen);
  of << "\n\n";

  builder::builder_context context;
  auto ast = context.extract_function_ast(set_X_T, "set_X_T", model);
  block::c_code_generator::generate_code(ast, of, 0);

  ast = context.extract_function_ast(fk, "fk", model);
  ast->dump(std::cout, 0);
  block::c_code_generator::generate_code(ast, of, 0);

  of << "}\n";
}

