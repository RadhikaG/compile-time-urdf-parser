#include "blocks/c_code_generator.h"
#include "builder/builder_context.h"
#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "builder/static_var.h"
#include "builder/array.h"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "assert.h"

using builder::dyn_var;
using builder::static_var;
using builder::dyn_arr;

using namespace pinocchio;

static const char eigen_Xmat_t_name[] = "Eigen::Matrix6d";
static const char eigen_vec_t_name[] = "Eigen::Vector15d";

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


};
}

template<
  typename Scalar,
  int Options,
  template<typename, int>
  class JointCollectionTpl
>
struct forwardKinematicsCodeGen {
  typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;

  const Model &model;
  dyn_arr<eigen_Xmat_t> X_T;

  forwardKinematicsCodeGen(Model &model) : model(model) {}

  void set_X_T() {
    X_T.set_size(model.njoints);

    typedef typename Model::JointIndex JointIndex;
    static_var<JointIndex> i = 1;

    for (; i < (JointIndex)model.njoints; i = i+1) {
      X_T[i] << model.jointPlacements[i]; 
    }
  }

  void jcalc(dyn_var<eigen_Xmat_t> & Xmat, size_t i, double q_i) {
    dyn_var<double> sinq = runtime::sin(q_i);
    dyn_var<double> cosq = runtime::cos(q_i);

    std::string joint_name = model.joints[i].shortname();
    if (joint_name.find("JointModelR") != std::string::npos) {
      char axis = joint_name.back();
      if (axis == 'X') {
        Xmat.coeffRef(1, 1) = cosq;
        Xmat.coeffRef(1, 2) = -sinq;
        Xmat.coeffRef(2, 1) = sinq;
        Xmat.coeffRef(2, 2) = cosq;
      } 
      else if (axis == 'Y') {
        Xmat.coeffRef(0, 0) = cosq;
        Xmat.coeffRef(0, 2) = sinq;
        Xmat.coeffRef(2, 0) = -sinq;
        Xmat.coeffRef(2, 2) = cosq;
      } 
      else if (axis == 'Z') {
        Xmat.coeffRef(0, 0) = cosq;
        Xmat.coeffRef(0, 1) = -sinq;
        Xmat.coeffRef(1, 0) = sinq;
        Xmat.coeffRef(1, 1) = cosq;
      } 
    }
    else if (joint_name.find("JointModelP") != std::string::npos) {
      assert(false && "Not yet implemented");
    }
    else {
      assert(false && "Joint type unsupported");
    }
  }

  dyn_var<eigen_Xmat_t> algo(dyn_var<eigen_vectorXd_t &> q) {
    dyn_var<eigen_Xmat_t[model.njoints]> X_J, X_0;
    
    typedef typename Model::JointIndex JointIndex;
    static_var<JointIndex> i = 1;

    for (; i < (JointIndex)model.njoints; i = i+1) {
      jcalc(X_J[i], i, q[i]);
    }

    static_var<JointIndex> parent;
    dyn_var<eigen_Xmat_t> X_pi;

    i = 1;
    for (; i < (JointIndex)model.njoints; i = i+1) {
      X_pi = X_T[i] * X_J[i];

      parent = model.parents(i);
      if (parent > 0) {
        X_0[i] = X_0[parent] * X_pi;
      }
      else {
        X_0[i] = X_pi;
      }
    }

    return X_0[model.njoints-1];
  }
};

int main(int argc, char* argv[]) {
  const std::string urdf_filename = argv[1];
  std::cout << urdf_filename << "\n";

  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  forwardKinematicsCodeGen fkcg(model);

  builder::builder_context context;
  auto ast = context.extract_function_ast(fkcg.set_X_T, "fk");
  ast->dump(std::cout, 0);
  block::c_code_generator::generate_code(ast, std::cout, 0);
}

