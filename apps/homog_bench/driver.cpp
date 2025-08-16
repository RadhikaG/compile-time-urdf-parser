#include "Eigen/Core"
#include "Eigen/Sparse"
#include "Eigen/Geometry"
#include "blaze/Math.h"
#include "vamp/vector.hh"
#include <type_traits>
#include "fk_gen.h"
#include <chrono>

#define N_IT 10000

#define START_MEASUREMENT() \
    auto __start = std::chrono::high_resolution_clock::now(); \
    for (int __i = 0; __i < N_IT; ++__i) {

#define STOP_MEASUREMENT() \
    } \
    auto __end = std::chrono::high_resolution_clock::now(); \
    auto __elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(__end - __start).count();

const size_t N_X_T = 1;
const size_t SIMD_WIDTH = 8UL;

// clang-format off
const double data[N_X_T][16] = {
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.42, 0.69, 0.33, 1}
};
// clang-format on

typedef Eigen::Matrix<float, 8, 1> eigen_avx256_intent;
typedef blaze::StaticVector<float, 8> blaze_avx256_intent;

int main(int argc, char *argv[]) {
  ///// INPUTS
  // scalar
  float q = 0.2;
  // eigen batched
  eigen_avx256_intent eigen_qb;
  eigen_qb.setConstant(q);
  // blaze batched
  blaze_avx256_intent blaze_qb;
  blaze_qb = q;
  // intrin batched
  vamp::FloatVector<8> intrin_qb;
  intrin_qb = q;

  ///// OUTPUTS
  // Eigen vanilla scalar
  Eigen::Matrix<float, 4, 4> X_0v;
  // Eigen isometry scalar
  Eigen::Transform<float, 3, Eigen::Isometry> X_0;
  // Eigen sparse
  Eigen::SparseMatrix<float> X_0sp(4, 4);
  // Blaze vanilla scalar
  blaze::StaticMatrix<float, 4, 4> X_0_bl;
  // Eigen batched
  Eigen::Matrix<eigen_avx256_intent, 4, 4> X_0b;
  // Blaze batched
  blaze::StaticMatrix<blaze_avx256_intent, 4, 4> X_0b_bl;
  // CTUP scalar
  Eigen::Matrix4f ctup_scalar_out;
  // CTUP blaze
  blaze::StaticMatrix<blaze_avx256_intent, 4, 4> ctup_blaze_out;
  // CTUP intrin, 8 is vector width, 16 is length
  vamp::FloatVector<8, 16> ctup_intrin_out;

  // Fixed transform
  Eigen::Matrix<double, 4, 4> pin_X_T(data[0]);

  ///// Eigen vanilla scalar

  Eigen::Matrix<float, 4, 4> X_Tv, X_Jv;
  X_Tv.setIdentity();
  X_Jv.setIdentity();

  {
    for (int r = 0; r < 4; r = r + 1) {
      for (int c = 0; c < 4; c = c + 1) {
        double entry = pin_X_T.coeffRef(r, c);
        X_Tv(r, c) = entry;
      }
    }
    START_MEASUREMENT()
    X_Jv(0, 0) = cos(q);
    X_Jv(0, 1) = -sin(q);
    X_Jv(1, 0) = sin(q);
    X_Jv(1, 1) = cos(q);
    X_0v = X_Tv * X_Jv;
    STOP_MEASUREMENT()
    std::cout << "eigen vanilla scalar avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
    //std::cout << X_0.matrix() << "\n";
  }

  ///// Eigen isometry scalar

  Eigen::Transform<float, 3, Eigen::Isometry> X_T, X_J;
  X_T.setIdentity();
  X_J.setIdentity();

  {
    for (int r = 0; r < 4; r = r + 1) {
      for (int c = 0; c < 4; c = c + 1) {
        double entry = pin_X_T.coeffRef(r, c);
        X_T(r, c) = entry;
      }
    }
    START_MEASUREMENT()
    X_J(0, 0) = cos(q);
    X_J(0, 1) = -sin(q);
    X_J(1, 0) = sin(q);
    X_J(1, 1) = cos(q);
    X_0 = X_T * X_J;
    STOP_MEASUREMENT()
    std::cout << "eigen isometry scalar avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
    //std::cout << X_0.matrix() << "\n";
  }

  ///// Eigen sparse

  Eigen::SparseMatrix<float> X_Tsp(4, 4), X_Jsp(4, 4);
  {
    std::vector<Eigen::Triplet<float>> T_triplets;
    for (int r = 0; r < 4; r = r + 1) {
      for (int c = 0; c < 4; c = c + 1) {
        double entry = pin_X_T.coeffRef(r, c);
        if (abs(entry) > 1e-5)
          T_triplets.emplace_back(r, c, entry);
      }
    }
    X_Tsp.setFromTriplets(T_triplets.begin(), T_triplets.end());

    START_MEASUREMENT()
    std::vector<Eigen::Triplet<float>> J_triplets;
    float c = std::cos(q);
    float s = std::sin(q);

    // Fill rotation about Z axis (top-left 3x3)
    J_triplets.emplace_back(0, 0, c);
    J_triplets.emplace_back(0, 1, -s);
    J_triplets.emplace_back(1, 0, s);
    J_triplets.emplace_back(1, 1, c);
    J_triplets.emplace_back(2, 2, 1);

    // Fill bottom row of homogeneous matrix
    J_triplets.emplace_back(3, 3, 1);

    // Set the triplets into the sparse matrix
    X_Jsp.setFromTriplets(J_triplets.begin(), J_triplets.end());

    X_0sp = X_Tsp * X_Jsp;

    STOP_MEASUREMENT()
    std::cout << "eigen sparse avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
    //for (int k = 0; k < X_0sp.outerSize(); ++k) {
    //  for (Eigen::SparseMatrix<float>::InnerIterator it(X_0sp, k); it; ++it) {
    //    std::cout << "(" << it.row() << ", " << it.col() << ") = " << it.value() << "\n";
    //  }
    //}
  }

  ///// Blaze scalar

  blaze::StaticMatrix<float, 4, 4> X_T_bl, X_J_bl;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      X_T_bl(i, j) = 1;
      X_J_bl(i, j) = 1;
    }
  }

  {
    for (int r = 0; r < 4; r = r + 1) {
      for (int c = 0; c < 4; c = c + 1) {
        double entry = pin_X_T.coeffRef(r, c);
        X_T_bl(r, c) = entry;
      }
    }
    START_MEASUREMENT()
    X_J_bl(0, 0) = cos(q);
    X_J_bl(0, 1) = -sin(q);
    X_J_bl(1, 0) = sin(q);
    X_J_bl(1, 1) = cos(q);
    X_0_bl = X_T_bl * X_J_bl;
    STOP_MEASUREMENT()
    std::cout << "blaze scalar avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
    //std::cout << X_0.matrix() << "\n";
  }


  ///// CTUP scalar

  {
    START_MEASUREMENT()
    ctup_scalar_out = ctup_gen::scalar_mm(q);
    STOP_MEASUREMENT()
    std::cout << "ctup scalar avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
    //std::cout << ctup_scalar_out << "\n";
  }

  ///// Eigen batched (no structure)

  // Eigen::Isometry3f does not have batched support

  Eigen::Matrix<eigen_avx256_intent, 4, 4> X_Tb, X_Jb;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      X_Tb(i, j).array() = 1;
      X_Jb(i, j).array() = 1;
    }
  }

  {
    for (int r = 0; r < 4; r = r + 1) {
      for (int c = 0; c < 4; c = c + 1) {
        double entry = pin_X_T.coeffRef(r, c);
        X_Tb(r, c).setConstant(entry);
      }
    }
    START_MEASUREMENT()
    X_Jb(0, 0).array() = cos(q);
    X_Jb(0, 1).array() = -sin(q);
    X_Jb(1, 0).array() = sin(q);
    X_Jb(1, 1).array() = cos(q);
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        for (int k = 0; k < 4; k++) {
          X_0b(i, j).array() += X_Tb(i, k).array() * X_Jb(k, j).array();
        }
      }
    }
    STOP_MEASUREMENT()
    std::cout << "batched mm eigen (no structure) avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
  }

  ///// Blaze batched (no structure)

  blaze::StaticMatrix<blaze_avx256_intent, 4, 4> X_Tb_bl, X_Jb_bl;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      X_Tb_bl(i, j) = 1;
      X_Jb_bl(i, j) = 1;
    }
  }

  {
    for (int r = 0; r < 4; r = r + 1) {
      for (int c = 0; c < 4; c = c + 1) {
        double entry = pin_X_T.coeffRef(r, c);
        X_Tb_bl(r, c) = entry;
      }
    }
    START_MEASUREMENT()
    X_Jb_bl(0, 0) = cos(q);
    X_Jb_bl(0, 1) = -sin(q);
    X_Jb_bl(1, 0) = sin(q);
    X_Jb_bl(1, 1) = cos(q);
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        for (int k = 0; k < 4; k++) {
          X_0b_bl(i, j) += X_Tb_bl(i, k) * X_Jb_bl(k, j);
        }
      }
    }
    STOP_MEASUREMENT()
    std::cout << "batched mm blaze (no structure) avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
  }

  ///// CTUP batched ablation

  {
    START_MEASUREMENT()
    ctup_blaze_out = ctup_gen::batched_mm_blaze(blaze_qb);
    STOP_MEASUREMENT()
    std::cout << "batched mm blaze avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
    //for (int i = 0; i < 4; i++)
    //  for (int j = 0; j < 4; j++)
    //    std::cout << ctup_blaze_out(i, j)[0] << ", ";
    //std::cout << "\n";
  }

  {
    START_MEASUREMENT()
    ctup_intrin_out = ctup_gen::batched_mm_intrin(intrin_qb);
    STOP_MEASUREMENT()
    std::cout << "batched mm intrin avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
  }

  {
    START_MEASUREMENT()
    ctup_scalar_out = ctup_gen::scalar_mm_constprop(q);
    STOP_MEASUREMENT()
    std::cout << "scalar mm constprop avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
  }

  {
    START_MEASUREMENT()
    ctup_blaze_out = ctup_gen::batched_mm_blaze_constprop(blaze_qb);
    STOP_MEASUREMENT()
    std::cout << "batched mm blaze constprop avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
  }

  {
    START_MEASUREMENT()
    ctup_intrin_out = ctup_gen::batched_mm_intrin_constprop(intrin_qb);
    STOP_MEASUREMENT()
    std::cout << "batched mm intrin constprop avg time taken (ns): " << (__elapsed / static_cast<double>(N_IT)) << "\n";
    //for (int i = 0; i < 16; i++)
    //  std::cout << ctup_intrin_out[i][0] << ", ";
    //std::cout << "\n";
  }
}
