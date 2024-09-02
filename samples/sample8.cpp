#include "blocks/c_code_generator.h"
#include "builder/builder_context.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "builder/static_var.h"
#include "builder/array.h"
#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "assert.h"

using builder::dyn_var;
using builder::static_var;
using builder::dyn_arr;

using namespace pinocchio;

static const char eigen_Xmat_t_name[] = "Eigen::MatrixXd";
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

};
}

dyn_var<eigen_Xmat_t*> X_T = builder::as_global("X_T");

void set_X_T(const Model &model) {
  //X_T.set_size(model.njoints);

  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i = 1;

  static_var<int> r = 0;
  static_var<int> c = 0;

  for (; i < (JointIndex)model.njoints; i = i+1) {
    Eigen::Matrix<double, 6, 6> blah = model.jointPlacements[i];

    for (r = 0; r < 6; r = r + 1) {
      for (c = 0; c < 6; c = c + 1) {
        //((dyn_var<eigen_Xmat_t>)(builder::cast)X_T[i]).coeffRef(r, c) = blah.coeffRef(r, c);
        X_T[i].coeffRef(r, c) = blah.coeffRef(r, c);
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

static int get_revolute_axis(const Model &model, Model::JointIndex i) {
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
  dyn_var<eigen_Xmat_t*> X_J, X_0;
  
  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i = 1;

  static_var<int> jtype;
  static_var<int> axis;

  for (; i < (JointIndex)model.njoints; i = i+1) {
    dyn_var<double> sinq = runtime::sin((dyn_var<double>)(builder::cast)q[i]);
    dyn_var<double> cosq = runtime::cos((dyn_var<double>)(builder::cast)q[i]);

    jtype = get_jtype(model, i);

    if (jtype == 'R') {
      axis = get_revolute_axis(model, i);
      if (axis == 'X') {
        //((dyn_var<eigen_Xmat_t>)(builder::cast)X_J[i]).coeffRef(1, 1) = cosq;
        X_J[i].coeffRef(1, 1) = cosq;
        X_J[i].coeffRef(1, 2) = -sinq;
        X_J[i].coeffRef(2, 1) = sinq;
        X_J[i].coeffRef(2, 2) = cosq;
      } 
      else if (axis == 'Y') {
        X_J[i].coeffRef(0, 0) = cosq;
        X_J[i].coeffRef(0, 2) = sinq;
        X_J[i].coeffRef(2, 0) = -sinq;
        X_J[i].coeffRef(2, 2) = cosq;
      } 
      else if (axis == 'Z') {
        X_J[i].coeffRef(0, 0) = cosq;
        X_J[i].coeffRef(0, 1) = -sinq;
        X_J[i].coeffRef(1, 0) = sinq;
        X_J[i].coeffRef(1, 1) = cosq;
      } 
    }
    //else if (is_prismatic) {
    //  //assert(false && "Not yet implemented");
    //}
    //else {
    //  //assert(false && "Joint type unsupported");
    //}
  }

  static_var<JointIndex> parent;
  dyn_var<eigen_Xmat_t> X_pi;

  i = 1;
  for (; i < (JointIndex)model.njoints; i = i+1) {
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

  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  builder::builder_context context;
  //auto ast = context.extract_function_ast(set_X_T, "set_X_T", model);
  auto ast = context.extract_function_ast(fk, "fk", model);
  ast->dump(std::cout, 0);
  block::c_code_generator::generate_code(ast, std::cout, 0);
}

