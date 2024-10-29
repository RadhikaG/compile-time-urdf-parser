#include "spatial_algebra.h"

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

std::vector<Xform<double>> X_T, X_J, X_0;

void set_X_T(const Model &model) {
  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i;

  static_var<int> r;
  static_var<int> c;

  for (i = 1; i < (JointIndex)model.njoints; i = i+1) {
    Eigen::Matrix3d pin_rot = model.jointPlacements[i].rotation();
    Eigen::Vector3d pin_trans = model.jointPlacements[i].translation();

    for (c = 0; c < 3; c = c + 1) {
      for (r = 0; r < 3; r = r + 1) {
        double entry = pin_rot.coeffRef(c, r);
        if (std::abs(entry) < 1e-5)
          X_T[i].rot.set_entry_to_constant(c, r, 0);
        else
          X_T[i].rot.set_entry_to_constant(c, r, entry);
      }
    }

    X_T[i].trans.set_x(pin_trans.coeffRef(0));
    X_T[i].trans.set_y(pin_trans.coeffRef(1));
    X_T[i].trans.set_z(pin_trans.coeffRef(2));
    //for (r = 0; r < 3; r = r + 1) {
    //  X_T_i.trans(r) = model.jointPlacements[i].translation().coeffRef(r);
    //}
  }
}

void fk(const Model &model, dyn_var<builder::eigen_vectorXd_t &> q) {
  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i;

  for (i = 0; i < (JointIndex)model.njoints; i = i+1) {
    X_T.push_back(Xform<double>());
    X_J.push_back(Xform<double>());
    X_0.push_back(Xform<double>());
  }

  set_X_T(model);

  static_var<int> jtype;
  static_var<int> axis;

  for (i = 1; i < (JointIndex)model.njoints; i = i+1) {
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
  Xform<double> X_pi;

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

  builder::builder_context context;

  //auto ast = context.extract_function_ast(set_X_T, "set_X_T", model);
  //block::c_code_generator::generate_code(ast, of, 0);

  auto ast = context.extract_function_ast(fk, "fk", model);
  ast->dump(std::cout, 0);
  block::c_code_generator::generate_code(ast, of, 0);

  of << "}\n";
}
