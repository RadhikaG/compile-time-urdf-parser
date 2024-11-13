// ./build/sample9 ./deps/pinocchio/models/baxter_simple.urdf ../ctup-experiments/gen/fk_gen_unrolled.h
#include "backend.h"
#include "spatial_algebra.h"
#include "utils.h"

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
#include <string>

using builder::dyn_var;
using builder::static_var;

using pinocchio::Model;

using ctup::Xform;

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

void set_X_T(Xform<double> X_T[], const Model &model) {
  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i;

  static_var<int> r;
  static_var<int> c;

  for (i = 1; i < (JointIndex)model.njoints; i = i+1) {
    Eigen::Matrix<double, 6, 6> pin_X_T = model.jointPlacements[i];

    // setting E
    for (r = 0; r < 3; r = r + 1) {
      for (c = 0; c < 3; c = c + 1) {
        double entry = pin_X_T.coeffRef(c, r);
        if (std::abs(entry) < 1e-5)
          X_T[i].rot.set_entry_to_constant(r, c, 0);
        else
          X_T[i].rot.set_entry_to_constant(r, c, entry);
      }
    }

    // setting -E*r-cross terms

    for (r = 0; r < 3; r = r + 1) {
      for (c = 0; c < 3; c = c + 1) {
        double entry = pin_X_T.coeffRef(c, r+3);
        X_T[i].minus_E_rcross.set_entry_to_constant(r, c, entry);
      }
    }
  }
}

dyn_var<builder::eigen_Xmat_t> fk(const Model &model, dyn_var<builder::eigen_vectorXd_t &> q) {
  Xform<double> X_T[model.njoints];
  Xform<double> X_J[model.njoints];
  Xform<double> X_0[model.njoints];

  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i;

  set_X_T(X_T, model);

  static_var<int> jtype;
  static_var<int> axis;

  for (i = 1; i < (JointIndex)model.njoints; i = i+1) {
    jtype = get_jtype(model, i);
    axis = get_joint_axis(model, i);

    if (jtype == 'R') {
      X_J[i].set_revolute_axis(axis);
    }
    if (jtype == 'P') {
      X_J[i].set_prismatic_axis(axis);
    }
  }

  static_var<JointIndex> parent;
  Xform<double> X_pi;

  for (i = 1; i < (JointIndex)model.njoints; i = i+1) {
    X_J[i].jcalc(q(i-1));

    X_pi = X_J[i] * X_T[i];
    parent = model.parents[i];
    if (parent > 0) {
      X_0[i] = X_pi * X_0[parent];
    }
    else {
      X_0[i] = X_pi;
    }
  }

  //for (i = 1; i < (JointIndex)model.njoints; i = i+1) {
  //  if (i <= 4) {
  //    ctup::print_Xmat_pin_order("XT ", X_T[i]);
  //    ctup::print_Xmat_pin_order("XJ ", X_J[i]);
  //    X_pi = X_J[i] * X_T[i];
  //    ctup::print_Xmat_pin_order("Xpi ", X_pi);
  //    ctup::print_Xmat_pin_order("Xpar ", X_0[model.parents[i]]);
  //  }

  //  std::string prefix("X ");
  //  ctup::print_Xmat_pin_order(prefix + std::to_string(i), X_0[i]);
  //}

  dyn_var<builder::eigen_Xmat_t> final_ans;
  toPinEigen(final_ans, X_0[model.njoints-1]);

  return final_ans;
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
  of << "#include <iostream>\n\n";
  of << "namespace ctup_gen {\n\n";

  of << "void print_string(const char* str) {\n";
  of << "  std::cout << str << \"\\n\";\n";
  of << "}\n\n";

  of << "template<typename Derived>\n";
  of << "void print_matrix(const Eigen::MatrixBase<Derived>& matrix) {\n";
  of << "  std::cout << matrix << \"\\n\";\n";
  of << "}\n\n";

  builder::builder_context context;

  auto ast = context.extract_function_ast(fk, "fk", model);
  //ast->dump(std::cout, 0);
  block::c_code_generator::generate_code(ast, of, 0);

  of << "}\n";
}
