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

int flattened_idx(int joint_id, int i, int j) {
  return joint_id * 36 + i*6 + j;
}

int flattened_joint_idx(int joint_id, int col) {
  return joint_id * 36 +  col;
}

void init_props(static_var<int> prop_X_T[], static_var<int> prop_X_J[], static_var<int> prop_X_pi[], static_var<int> prop_X_0[], size_t njoints) {
  static_var<int> i;
  for (i = 0; i < 36 * njoints; i = i + 1) {
    prop_X_T[i] = 2;
    prop_X_J[i] = 0;
    prop_X_pi[i] = 2;
    prop_X_0[i] = 2;
  }
}

void set_prop_X_T(const Model &model, static_var<int> prop_X_T[]) {

  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i = 1;

  static_var<int> r = 0;
  static_var<int> c = 0;

  for (; i < (JointIndex)model.njoints; i = i+1) {
    Eigen::Matrix<double, 6, 6> tmp_cst_mat = model.jointPlacements[i];

    for (c = 0; c < 6; c = c + 1) {
      for (r = 0; r < 6; r = r + 1) {
        //((dyn_var<eigen_Xmat_t>)(builder::cast)X_T[i]).coeffRef(r, c) = blah.coeffRef(r, c);
        if (abs(tmp_cst_mat.coeffRef(c, r)) <1e-5){
          prop_X_T[flattened_idx(i, c, r)] = 0;
        }
        else if(tmp_cst_mat.coeffRef(c, r) < 1+1e-5 && tmp_cst_mat.coeffRef(c, r) > 1-1e-5 ){
          prop_X_T[flattened_idx(i, c, r)] = 1;
        }
      }
    }
  }
}

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

void generate_output_sparsity(static_var<int> lhs[], static_var<int> rhs1[], static_var<int> rhs2[], int joint_id) {
  int offset = joint_id * 36;

  static_var<int> is_inner_sum_nonzero = false;
  static_var<int> i, j, k;

  for (i = 0; i < 6; i = i+1) {
    for (j = 0; j < 6; j = j+1) {
      is_inner_sum_nonzero = 0;
      for (k = 0; k < 6; k = k+1) {
        if (!(rhs1[flattened_idx(joint_id, i, k)] == 0 || rhs2[flattened_idx(joint_id, k, j)] == 0)) {
          if ((rhs1[flattened_idx(joint_id, i, k)] == 2 || rhs2[flattened_idx(joint_id, k, j)] == 2)) {
            is_inner_sum_nonzero = 2;
            break;
          }
          is_inner_sum_nonzero = 1;
        }
      }
      lhs[flattened_idx(joint_id, i, j)] = is_inner_sum_nonzero;
    }
  }
}

void generate_output_sparsity_2(static_var<int> lhs[], static_var<int> rhs1[], static_var<int> rhs2[], int joint_id,int joint_id2) {
  int offset = joint_id * 36;

  static_var<int> is_inner_sum_nonzero = false;
  static_var<int> i, j, k;

  for (i = 0; i < 6; i = i+1) {
    for (j = 0; j < 6; j = j+1) {
      is_inner_sum_nonzero = 0;
      for (k = 0; k < 6; k = k+1) {
        if (!(rhs1[flattened_idx(joint_id2, i, k)] == 0 || rhs2[flattened_idx(joint_id, k, j)] == 0)) {
          if ((rhs1[flattened_idx(joint_id2, i, k)] == 2 || rhs2[flattened_idx(joint_id, k, j)] == 2)) {
            is_inner_sum_nonzero = 2;
            break;
          }
          is_inner_sum_nonzero = 1;
        }
      }
      lhs[flattened_idx(joint_id, i, j)] = rhs1[flattened_idx(joint_id2, i, k)] *rhs2[flattened_idx(joint_id, k, j)];
    }
  }
}

void copy_sparsity(static_var<int> lhs[], static_var<int> rhs[], int joint_id) {

  static_var<int> i, j;

  for (i = 0; i < 6; i = i+1) {
    for (j = 0; j < 6; j = j+1) {
        lhs[flattened_idx(joint_id, i, j)]=rhs[flattened_idx(joint_id, i, j)];
    }
  }
}

dyn_var<EigenMatrix<double>> fk(const Model &model, dyn_var<EigenMatrix<double>> q) {
  // 0 for zero, 1 for one, 2 for everything else
  static_var<int> prop_X_T[36 * model.njoints];
  static_var<int> prop_X_J[36 * model.njoints];
  static_var<int> prop_X_pi[36 * model.njoints];
  static_var<int> prop_X_0[36 * model.njoints];

  init_props(prop_X_T, prop_X_J, prop_X_pi, prop_X_0, model.njoints);
  set_prop_X_T(model, prop_X_T);

  dyn_var<EigenMatrix<double>[]> X_J;
  dyn_var<EigenMatrix<double>[]> X_0;

  resize_arr(X_J, model.njoints);
  resize_arr(X_0, model.njoints);
  
  typedef typename Model::JointIndex JointIndex;
  static_var<JointIndex> i = 1;

  static_var<int> jtype;
  static_var<int> axis;

  static_var<JointIndex> j = 1;
  for (; j <(JointIndex)model.njoints; j = j+1) {
    X_0[j] = dyn_var<EigenMatrix<double>>(16, 36);
    X_J[j] = dyn_var<EigenMatrix<double>>(16, 36);
  }
  dyn_var<EigenMatrix<double>> a(16,1);
  dyn_var<EigenMatrix<double>> b(16,1);
  dyn_var<EigenMatrix<double>> c(16,1);
  c.setConstant(1);

  for (; i < (JointIndex)model.njoints; i = i+1) {

    jtype = get_jtype(model, i);
    axis = get_joint_axis(model, i);
    ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).setZero();

    builder::annotate(std::string(model.names[i]));

    a=(CAST_TO_VECTOR_XD(CAST_TO_VECTOR_XD q.col(i-1)).array()).cos();
    b=((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<builder::eigen_vectorXd_t>)(builder::cast) q.col(i-1)).array()).sin();

    if (jtype == 'R') {
      if (axis == 'X') {
        ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(0))=c;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(7) = a;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(8) = -b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(13) = b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(14) = a;
        // not smart but doing nonetheless for symmetric E
        ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(21))=c;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(28) = a;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(29) = -b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(34) = b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(35) = a;

        prop_X_J[flattened_joint_idx(i, 0)] = 1;
        prop_X_J[flattened_joint_idx(i, 21)] = 1;

        prop_X_J[flattened_joint_idx(i, 7)] = 2;
        prop_X_J[flattened_joint_idx(i, 8)] = 2;
        prop_X_J[flattened_joint_idx(i, 13)] = 2;
        prop_X_J[flattened_joint_idx(i, 14)] = 2;
        prop_X_J[flattened_joint_idx(i, 28)] = 2;
        prop_X_J[flattened_joint_idx(i, 29)] = 2;
        prop_X_J[flattened_joint_idx(i, 34)] = 2;
        prop_X_J[flattened_joint_idx(i, 35)] = 2;
      } 
      else if (axis == 'Y') {
        ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(7))=c;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(0) = a;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(2) = b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(12) = -b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(14) = a;
        // not smart but doing nonetheless for symm E
        ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(28))=c;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(21) = a;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(23) = b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(33) = -b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(35) = a;

        prop_X_J[flattened_joint_idx(i, 7)] = 1;
        prop_X_J[flattened_joint_idx(i, 28)] = 1;

        prop_X_J[flattened_joint_idx(i, 0)] = 2;
        prop_X_J[flattened_joint_idx(i, 2)] = 2;
        prop_X_J[flattened_joint_idx(i, 12)] = 2;
        prop_X_J[flattened_joint_idx(i, 14)] = 2;
        prop_X_J[flattened_joint_idx(i, 21)] = 2;
        prop_X_J[flattened_joint_idx(i, 23)] = 2;
        prop_X_J[flattened_joint_idx(i, 33)] = 2;
        prop_X_J[flattened_joint_idx(i, 35)] = 2;
      } 
      else if (axis == 'Z') {
        ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(14))=c;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(0) = a;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(1) = -b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(6) = b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(7) = a;
        // not smart but doing nonetheless for symm E
        ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(35))=c;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(21) = a;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(22) = -b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(27) = b;
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(28) = a;

        prop_X_J[flattened_joint_idx(i, 14)] = 1;
        prop_X_J[flattened_joint_idx(i, 35)] = 1;

        prop_X_J[flattened_joint_idx(i, 0)] = 2;
        prop_X_J[flattened_joint_idx(i, 1)] = 2;
        prop_X_J[flattened_joint_idx(i, 6)] = 2;
        prop_X_J[flattened_joint_idx(i, 7)] = 2;
        prop_X_J[flattened_joint_idx(i, 21)] = 2;
        prop_X_J[flattened_joint_idx(i, 22)] = 2;
        prop_X_J[flattened_joint_idx(i, 27)] = 2;
        prop_X_J[flattened_joint_idx(i, 28)] = 2;
      } 
      // r = 0
    }
    else if (jtype == 'P') {
      // negative r-cross, opp signs of featherstone 2.23
      if (axis == 'X') {
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(8) = -(q.col(i-1));
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(13) = (q.col(i-1));

        prop_X_J[flattened_joint_idx(i, 8)] = 2;
        prop_X_J[flattened_joint_idx(i, 13)] = 2;
      }
      else if (axis == 'Y') {
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(2) = (q.col(i-1));
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(12) = -(q.col(i-1));

        prop_X_J[flattened_joint_idx(i, 2)] = 2;
        prop_X_J[flattened_joint_idx(i, 12)] = 2;
      }
      else if (axis == 'Z') {
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(1) = -(q.col(i-1));
        ((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i]).col(6) = (q.col(i-1));

        prop_X_J[flattened_joint_idx(i, 1)] = 2;
        prop_X_J[flattened_joint_idx(i, 6)] = 2;
      }
      // E = Identity 
      ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(0)).setConstant(1);
      ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(7)).setConstant(1);
      ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(14)).setConstant(1);
      // symm E
      ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(21)).setConstant(1);
      ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(28)).setConstant(1);
      ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_J[i]).col(35)).setConstant(1);

      prop_X_J[flattened_joint_idx(i, 0)] = 1;
      prop_X_J[flattened_joint_idx(i, 7)] = 1;
      prop_X_J[flattened_joint_idx(i, 14)] = 1;
      prop_X_J[flattened_joint_idx(i, 21)] = 1;
      prop_X_J[flattened_joint_idx(i, 28)] = 1;
      prop_X_J[flattened_joint_idx(i, 35)] = 1;    
    }
    else {
      assert(false && "joint type unsupported");
    }
  }

  static_var<JointIndex> parent;
  dyn_var<EigenMatrix<double>> X_pi(16,36);

  i = 1;
  for (; i < (JointIndex)model.njoints; i = i+1) {
    //(CAST_TO_EIGEN_MATRIX X_0[i]).setZero();

    builder::annotate(std::string(model.names[i]));

    static_var<int> l;

    generate_output_sparsity(prop_X_pi,prop_X_T,prop_X_J,i);

    static_var<int> sum;
    sum=true;

    for(l=0;l<36;l=l+1){
        if(prop_X_J[flattened_joint_idx(i, l%6)]==2 && prop_X_T[flattened_idx(i, l/6, 0)]==2){
          X_pi.col(l)  =  X_T[i].coeffRef(l/6,0)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(0+l%6);
          sum=false;
        }
        else if(prop_X_J[flattened_joint_idx(i, 0+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6, 0)]==2 ){
          (CAST_TO_VECTOR_XD X_pi.col(l)).array()  = X_T[i].coeffRef(l/6,0);
          sum=false;
        }
        else if(prop_X_J[flattened_joint_idx(i, 0+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6, 0)]== 1 ){
          X_pi.col(l)  = (((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(0+l%6);
          sum=false;
        }

        if(sum==true){
          if(prop_X_J[flattened_joint_idx(i, 6+l%6)]==2 && prop_X_T[flattened_idx(i, l/6,1)]==2){
            X_pi.col(l)  =  X_T[i].coeffRef(l/6,1)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(6+l%6);
            sum=false;
          }
          else if(prop_X_J[flattened_joint_idx(i, 6+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6,1)]==2 ){
            (CAST_TO_VECTOR_XD X_pi.col(l)).array()  = X_T[i].coeffRef(l/6,1);
            sum=false;
          }
          else if(prop_X_J[flattened_joint_idx(i, 6+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6,1)]== 1 ){
            X_pi.col(l)  = (((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(6+l%6);
            sum=false;
          }
        }
        else{
          if(prop_X_J[flattened_joint_idx(i, 6+l%6)]==2 && prop_X_T[flattened_idx(i, l/6,1)]==2){
            X_pi.col(l) += X_T[i].coeffRef(l/6,1)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(6+l%6);
          }
          else if(prop_X_J[flattened_joint_idx(i, 6+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6,1)]==2 ){
            (CAST_TO_VECTOR_XD X_pi.col(l)).array() +=X_T[i].coeffRef(l/6,1);
          }
          else if(prop_X_J[flattened_joint_idx(i, 6+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6,1)]== 1 ){
            X_pi.col(l) +=(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(6+l%6);
          }
        }

        if(sum==true){
          if(prop_X_J[flattened_joint_idx(i, 12+l%6)]==2 && prop_X_T[flattened_idx(i, l/6,2)]==2){
            X_pi.col(l)  =  X_T[i].coeffRef(l/6,2)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(12+l%6);
            sum=false;
          }
          else if(prop_X_J[flattened_joint_idx(i, 12+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6,2)]==2 ){
            (CAST_TO_VECTOR_XD X_pi.col(l)).array()  = X_T[i].coeffRef(l/6,2);
            sum=false;
          }
          else if(prop_X_J[flattened_joint_idx(i, 12+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6,2)]== 1 ){
            X_pi.col(l)  = (((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(12+l%6);
            sum=false;
          }
        }
        else{
          if(prop_X_J[flattened_joint_idx(i, 12+l%6)]==2 && prop_X_T[flattened_idx(i, l/6,2)]==2){
            X_pi.col(l) += X_T[i].coeffRef(l/6,2)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(12+l%6);
          }
          else if(prop_X_J[flattened_joint_idx(i, 12+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6,2)]==2 ){
            (CAST_TO_VECTOR_XD X_pi.col(l)).array() +=X_T[i].coeffRef(l/6,2);
          }
          else if(prop_X_J[flattened_joint_idx(i, 12+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6,2)]== 1 ){
            X_pi.col(l) +=(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(12+l%6);
          }
        }

        if(sum==true){
          if(prop_X_J[flattened_joint_idx(i, 18+l%6)]==2 && prop_X_T[flattened_idx(i, l/6,3)]==2){
            X_pi.col(l)  =  X_T[i].coeffRef(l/6,3)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(18+l%6);
            sum=false;
          }
          else if(prop_X_J[flattened_joint_idx(i, 18+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6,3)]==2 ){
            (CAST_TO_VECTOR_XD X_pi.col(l)).array()  = X_T[i].coeffRef(l/6,3);
            sum=false;
          }
          else if(prop_X_J[flattened_joint_idx(i, 18+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6,3)]== 1 ){
            X_pi.col(l)  = (((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(18+l%6);
            sum=false;
          }
        }
        else{
          if(prop_X_J[flattened_joint_idx(i, 18+l%6)]==2 && prop_X_T[flattened_idx(i, l/6,3)]==2){
            X_pi.col(l) += X_T[i].coeffRef(l/6,3)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(18+l%6);
          }
          else if(prop_X_J[flattened_joint_idx(i, 18+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6,3)]==2 ){
            (CAST_TO_VECTOR_XD X_pi.col(l)).array() +=X_T[i].coeffRef(l/6,3);
          }
          else if(prop_X_J[flattened_joint_idx(i, 18+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6,3)]== 1 ){
            X_pi.col(l) +=(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(18+l%6);
          }
        }
        
        if(sum==true){
          if(prop_X_J[flattened_joint_idx(i, 24+l%6)]==2 && prop_X_T[flattened_idx(i, l/6,4)]==2){
            X_pi.col(l)  =  X_T[i].coeffRef(l/6,4)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(24+l%6);
            sum=false;
          }
          else if(prop_X_J[flattened_joint_idx(i, 24+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6,4)]==2 ){
            (CAST_TO_VECTOR_XD X_pi.col(l)).array()  = X_T[i].coeffRef(l/6,4);
            sum=false;
          }
          else if(prop_X_J[flattened_joint_idx(i, 24+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6,4)]== 1 ){
            X_pi.col(l)  = (((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(24+l%6);
            sum=false;
          }
        }
        else{
          if(prop_X_J[flattened_joint_idx(i, 24+l%6)]==2 && prop_X_T[flattened_idx(i, l/6,4)]==2){
            X_pi.col(l) += X_T[i].coeffRef(l/6,4)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(24+l%6);
          }
          else if(prop_X_J[flattened_joint_idx(i, 24+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6,4)]==2 ){
            (CAST_TO_VECTOR_XD  X_pi.col(l)).array() +=X_T[i].coeffRef(l/6,4);
          }
          else if(prop_X_J[flattened_joint_idx(i, 24+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6,4)]== 1 ){
            X_pi.col(l) +=(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(24+l%6);
          }
        }

        if(sum==true){
          if(prop_X_J[flattened_joint_idx(i, 30+l%6)]==2 && prop_X_T[flattened_idx(i, l/6,5)]==2){
            X_pi.col(l)  =  X_T[i].coeffRef(l/6,5)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(30+l%6);
            sum=false;
          }
          else if(prop_X_J[flattened_joint_idx(i, 30+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6,5)]==2 ){
            (CAST_TO_VECTOR_XD X_pi.col(l)).array()  = X_T[i].coeffRef(l/6,5);
            sum=false;
          }
          else if(prop_X_J[flattened_joint_idx(i, 30+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6,5)]== 1 ){
            X_pi.col(l)  = (((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(30+l%6);
            sum=false;
          }
        }
        else{
          if(prop_X_J[flattened_joint_idx(i, 30+l%6)]==2 && prop_X_T[flattened_idx(i, l/6,5)]==2){
            X_pi.col(l) += X_T[i].coeffRef(l/6,5)*(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(30+l%6);
          }
          else if(prop_X_J[flattened_joint_idx(i, 30+l%6)]== 1 && prop_X_T[flattened_idx(i, l/6,5)]==2 ){
            (CAST_TO_VECTOR_XD X_pi.col(l)).array() +=X_T[i].coeffRef(l/6,5);
          }
          else if(prop_X_J[flattened_joint_idx(i, 30+l%6)]== 2 && prop_X_T[flattened_idx(i, l/6,5)]== 1 ){
            X_pi.col(l) +=(((dyn_var<EigenMatrix<double>>)(builder::cast)X_J[i])).col(30+l%6);
          }
        }
        
    }

    parent = model.parents[i];
    if (parent > 0) {
      generate_output_sparsity_2(prop_X_0,prop_X_0,prop_X_pi,i,parent);
      sum=true;
      for(l=0;l<36;l=l+1){
        /*
        (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  =  
          ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(0+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(0+l%6)).array()
        + ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(1+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(6+l%6)).array()
        + ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(2+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(12+l%6)).array()
        + ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(3+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(18+l%6)).array()
        + ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(4+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(24+l%6)).array()
        + ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(5+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(30+l%6)).array();
        */
       
        if(prop_X_pi[flattened_joint_idx(i, 0+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 0+6*(l/6))]==2){
          (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  =  ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(0+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(0+l%6)).array();
          sum=false;
        }
        else if(prop_X_pi[flattened_joint_idx(i, 0+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 0+6*(l/6))]==2 ){
          (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(0+6*(l/6));
          sum=false;
        }
        else if(prop_X_pi[flattened_joint_idx(i, 0+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 0+6*(l/6))]== 1 ){
          (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = (X_pi).col(0+l%6);
          sum=false;
        }

        if(sum==true){
          if(prop_X_pi[flattened_joint_idx(i, 6+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 1+6*(l/6))]==2){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  =  ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(1+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(6+l%6)).array();
            sum=false;
          }
          else if(prop_X_pi[flattened_joint_idx(i, 6+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 1+6*(l/6))]==2 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(1+6*(l/6));
            sum=false;
          }
          else if(prop_X_pi[flattened_joint_idx(i, 6+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 1+6*(l/6))]== 1 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = (X_pi).col(6+l%6);
            sum=false;
          }
        }
        else{
          if(prop_X_pi[flattened_joint_idx(i, 6+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 1+6*(l/6))]==2){
            (CAST_TO_VECTOR_XD(((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)).array()  +=  ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(1+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(6+l%6)).array();
          }
          else if(prop_X_pi[flattened_joint_idx(i, 6+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 1+6*(l/6))]==2 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  += ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(1+6*(l/6));
          }
          else if(prop_X_pi[flattened_joint_idx(i, 6+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 1+6*(l/6))]== 1 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  += (X_pi).col(6+l%6);
          }
        }

        if(sum==true){
          if(prop_X_pi[flattened_joint_idx(i, 12+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 2+6*(l/6))]==2){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(2+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(12+l%6)).array();
            sum=false;
          }
          else if(prop_X_pi[flattened_joint_idx(i, 12+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 2+6*(l/6))]==2 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(2+6*(l/6));
            sum=false;
          }
          else if(prop_X_pi[flattened_joint_idx(i, 12+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 2+6*(l/6))]== 1 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = (X_pi).col(12+l%6);
            sum=false;
          }
        }
        else{
          if(prop_X_pi[flattened_joint_idx(i, 12+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 2+6*(l/6))]==2){
            (CAST_TO_VECTOR_XD(((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)).array()  += ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(2+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(12+l%6)).array();
          }
          else if(prop_X_pi[flattened_joint_idx(i, 12+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 2+6*(l/6))]==2 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  += ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(2+6*(l/6));
          }
          else if(prop_X_pi[flattened_joint_idx(i, 12+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 2+6*(l/6))]== 1 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  += (X_pi).col(12+l%6);
          }
        }

        if(sum==true){
          if(prop_X_pi[flattened_joint_idx(i, 18+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 3+6*(l/6))]==2){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(3+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(18+l%6)).array();
            sum=false;
          }
          else if(prop_X_pi[flattened_joint_idx(i, 18+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 3+6*(l/6))]==2 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(3+6*(l/6));
            sum=false;
          }
          else if(prop_X_pi[flattened_joint_idx(i, 18+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 3+6*(l/6))]== 1 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = (X_pi).col(18+l%6);
            sum=false;
          }
        }
        else{
          if(prop_X_pi[flattened_joint_idx(i, 18+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 3+6*(l/6))]==2){
            (CAST_TO_VECTOR_XD(((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)).array()  += ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(3+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(18+l%6)).array();
          }
          else if(prop_X_pi[flattened_joint_idx(i, 18+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 3+6*(l/6))]==2 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  += ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(3+6*(l/6));
          }
          else if(prop_X_pi[flattened_joint_idx(i, 18+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 3+6*(l/6))]== 1 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  += (X_pi).col(18+l%6);
          }
        }
        
        if(sum==true){
          if(prop_X_pi[flattened_joint_idx(i, 24+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 4+6*(l/6))]==2){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(4+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(24+l%6)).array();
            sum=false;
          }
          else if(prop_X_pi[flattened_joint_idx(i, 24+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 4+6*(l/6))]==2 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(4+6*(l/6));
            sum=false;
          }
          else if(prop_X_pi[flattened_joint_idx(i, 24+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 4+6*(l/6))]== 1 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = (X_pi).col(24+l%6);
            sum=false;
          }
        }
        else{
          if(prop_X_pi[flattened_joint_idx(i, 24+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 4+6*(l/6))]==2){
            (CAST_TO_VECTOR_XD(((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)).array()  += ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(4+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(24+l%6)).array();
          }
          else if(prop_X_pi[flattened_joint_idx(i, 24+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 4+6*(l/6))]==2 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  += ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(4+6*(l/6));
          }
          else if(prop_X_pi[flattened_joint_idx(i, 24+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 4+6*(l/6))]== 1 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  += (X_pi).col(24+l%6);
          }
        }

        if(sum==true){
          if(prop_X_pi[flattened_joint_idx(i, 30+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 5+6*(l/6))]==2){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  =  ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(5+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(30+l%6)).array();
            sum=false;
          }
          else if(prop_X_pi[flattened_joint_idx(i, 30+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 5+6*(l/6))]==2 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(5+6*(l/6));
            sum=false;
          }
          else if(prop_X_pi[flattened_joint_idx(i, 30+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 5+6*(l/6))]== 1 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  = (X_pi).col(30+l%6);
            sum=false;
          }
        }
        else{
          if(prop_X_pi[flattened_joint_idx(i, 30+l%6)]==2 && prop_X_0[flattened_joint_idx(parent, 5+6*(l/6))]==2){
            (CAST_TO_VECTOR_XD(((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)).array()  +=  ((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(5+6*(l/6))).array()*((dyn_var<builder::eigen_vectorXd_t>)(builder::cast)(X_pi).col(30+l%6)).array();
          }
          else if(prop_X_pi[flattened_joint_idx(i, 30+l%6)]== 1 && prop_X_0[flattened_joint_idx(parent, 5+6*(l/6))]==2 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  += ((dyn_var<EigenMatrix<double>>)(builder::cast) X_0[parent]).col(5+6*(l/6));
          }
          else if(prop_X_pi[flattened_joint_idx(i, 30+l%6)]== 2 && prop_X_0[flattened_joint_idx(parent, 5+6*(l/6))]== 1 ){
            (((dyn_var<EigenMatrix<double>>)(builder::cast)X_0[i])).col(l)  += (X_pi).col(30+l%6);
          }
        }
      
      }
    }
    else {
      copy_sparsity(prop_X_0,prop_X_pi,i);
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

