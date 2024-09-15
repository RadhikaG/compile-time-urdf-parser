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

using builder::dyn_var;
using builder::static_var;

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

dyn_var<eigen_Xmat_t> fk(const Model &model, dyn_var<eigen_vectorXd_t &> q) {
  dyn_var<eigen_Xmat_t[]> X_J, X_0;
  resize_arr(X_J, model.njoints);
  resize_arr(X_0, model.njoints);
  
  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i = 1;

  static_var<int> jtype;
  static_var<int> axis;

  for (; i < (JointIndex)model.njoints; i = i+1) {
    dyn_var<double> sinq = runtime::sin((dyn_var<double>)(builder::cast)q(i-1));
    dyn_var<double> cosq = runtime::cos((dyn_var<double>)(builder::cast)q(i-1));

    jtype = get_jtype(model, i);
    axis = get_joint_axis(model, i);

    X_J[i].setZero();

    builder::annotate(std::string(model.names[i]));
    if (jtype == 'R') {
      if (axis == 'X') {
        X_J[i].coeffRef(0, 0) = 1;
        X_J[i].coeffRef(1, 1) = cosq;
        X_J[i].coeffRef(1, 2) = -sinq;
        X_J[i].coeffRef(2, 1) = sinq;
        X_J[i].coeffRef(2, 2) = cosq;
        // not smart but doing nonetheless for symmetric E
        X_J[i].coeffRef(3+0, 3+0) = 1;
        X_J[i].coeffRef(3+1, 3+1) = cosq;
        X_J[i].coeffRef(3+1, 3+2) = -sinq;
        X_J[i].coeffRef(3+2, 3+1) = sinq;
        X_J[i].coeffRef(3+2, 3+2) = cosq;
      } 
      else if (axis == 'Y') {
        X_J[i].coeffRef(1, 1) = 1;
        X_J[i].coeffRef(0, 0) = cosq;
        X_J[i].coeffRef(0, 2) = sinq;
        X_J[i].coeffRef(2, 0) = -sinq;
        X_J[i].coeffRef(2, 2) = cosq;
        // not smart but doing nonetheless for symm E
        X_J[i].coeffRef(3+1, 3+1) = 1;
        X_J[i].coeffRef(3+0, 3+0) = cosq;
        X_J[i].coeffRef(3+0, 3+2) = sinq;
        X_J[i].coeffRef(3+2, 3+0) = -sinq;
        X_J[i].coeffRef(3+2, 3+2) = cosq;
      } 
      else if (axis == 'Z') {
        X_J[i].coeffRef(2, 2) = 1;
        X_J[i].coeffRef(0, 0) = cosq;
        X_J[i].coeffRef(0, 1) = -sinq;
        X_J[i].coeffRef(1, 0) = sinq;
        X_J[i].coeffRef(1, 1) = cosq;
        // not smart but doing nonetheless for symm E
        X_J[i].coeffRef(3+2, 3+2) = 1;
        X_J[i].coeffRef(3+0, 3+0) = cosq;
        X_J[i].coeffRef(3+0, 3+1) = -sinq;
        X_J[i].coeffRef(3+1, 3+0) = sinq;
        X_J[i].coeffRef(3+1, 3+1) = cosq;
      } 
      // r = 0
    }
    else if (jtype == 'P') {
      // negative r-cross, opp signs of featherstone 2.23
      if (axis == 'X') {
        X_J[i].coeffRef(1, 2) = -q(i-1);
        X_J[i].coeffRef(2, 1) = q(i-1);
      }
      else if (axis == 'Y') {
        X_J[i].coeffRef(0, 2) = q(i-1);
        X_J[i].coeffRef(2, 0) = -q(i-1);
      }
      else if (axis == 'Z') {
        X_J[i].coeffRef(0, 1) = -q(i-1);
        X_J[i].coeffRef(1, 0) = q(i-1);
      }
      // E = Identity 
      X_J[i].coeffRef(0, 0) = 1;
      X_J[i].coeffRef(1, 1) = 1;
      X_J[i].coeffRef(2, 2) = 1;
      // symm E
      X_J[i].coeffRef(3+0, 3+0) = 1;
      X_J[i].coeffRef(3+1, 3+1) = 1;
      X_J[i].coeffRef(3+2, 3+2) = 1;
    }
    else {
      assert(false && "joint type unsupported");
    }
  }

  static_var<JointIndex> parent;
  dyn_var<eigen_Xmat_t> X_pi;

  i = 1;
  for (; i < (JointIndex)model.njoints; i = i+1) {
    builder::annotate(std::string(model.names[i]));
    X_pi = X_T[i] * X_J[i];

    parent = model.parents[i];
    if (parent > 0) {
      X_0[i] = X_0[parent] * X_pi;
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

