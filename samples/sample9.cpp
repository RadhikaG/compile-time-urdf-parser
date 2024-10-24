#include "include/xform_impl.h"

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

using pinocchio::Model;

std::vector<ctup::Xform<double>> X_T, X_J, X_0;

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

void set_X_T(const Model &model) {
  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i = 1;

  static_var<int> r = 0;
  static_var<int> c = 0;

  for (; i < (JointIndex)model.njoints; i = i+1) {
    Eigen::Matrix<double, 6, 6> tmp_cst_mat = model.jointPlacements[i];

    // reverse engineer rot and trans from tmp_cst_mat

    builder::annotate(std::string(model.names[i]));
    for (c = 0; c < 6; c = c + 1) {
      for (r = 0; r < 6; r = r + 1) {
        //((dyn_var<eigen_Xmat_t>)(builder::cast)X_T[i]).coeffRef(r, c) = blah.coeffRef(r, c);
        X_T[i].coeffRef(c, r) = tmp_cst_mat.coeffRef(c, r);
      }
    }
  }
}

void fk(const Model &model, dyn_var<eigen_vectorXd_t &> q) {
  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i = 1;

  static_var<int> jtype;
  static_var<int> axis;

  for (; i < (JointIndex)model.njoints; i = i+1) {
    jtype = get_jtype(model, i);
    axis = get_joint_axis(model, i);

    if (jtype == 'R') {
      X_J[i].set_revolute_axis(axis);
    }
    else if (jtype == 'P') {
      X_J[i].set_prismatic_axis(axis);
    }
  }

  static_var<JointIndex> parent;
  Xform<Scalar> X_pi;

  for (i = 1; i < (JointIndex)model.njoints; i = i+1) {
    X_J[i].jcalc(q[i]);

    X_pi = X_T[i] * X_J[i];
    parent = model.parents[i];
    if (parent > 0) {
      X_0[i] = X_0[parent] * X_pi;
    }
    else {
      X_0[i] = X_pi;
    }
  }
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
