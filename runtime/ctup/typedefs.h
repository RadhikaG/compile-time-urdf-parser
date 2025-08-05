#ifndef CTUP_TYPEDEFS_H
#define CTUP_TYPEDEFS_H
#include <type_traits>
#include <vector>
#include "blaze/Math.h"
#include <sstream>

namespace ctup_runtime {

template <typename Scalar>
struct inner_type { 
  using type = Scalar;
};

// need to include the third transpose flag param
// to match the template properly, even though we only ever 
// use the default value.
template <typename Scalar, size_t _Rows, bool TF>
struct inner_type<blaze::StaticVector<Scalar, _Rows, TF>> {
  using type = Scalar;
};

using avx256f = blaze::StaticVector<float, 8>;
using avx512f = blaze::StaticVector<float, 16>;

using avx256d = blaze::StaticVector<double, 4>;
using avx512d = blaze::StaticVector<double, 8>;

//using avx256f = blaze::StaticVector<float, 8, blaze::rowVector, blaze::aligned, blaze::padded>;
//using avx512f = blaze::StaticVector<float, 16, blaze::rowVector, blaze::aligned, blaze::padded>;
//
//using avx256d = blaze::StaticVector<double, 4, blaze::rowVector, blaze::aligned, blaze::padded>;
//using avx512d = blaze::StaticVector<double, 8, blaze::rowVector, blaze::aligned, blaze::padded>;

template<typename SV>
using Allocator = blaze::AlignedAllocator<SV>;

template <typename SV>
using AlignedVec = std::vector<SV, Allocator<SV>>;

using alignedv_avx256f = AlignedVec<avx256f>;
using alignedv_avx512f = AlignedVec<avx512f>;
using alignedv_avx256d = AlignedVec<avx256d>;
using alignedv_avx512d = AlignedVec<avx512d>;

template <typename VT, bool TF>
std::string pp(const blaze::Vector<VT, TF>& vec) {
  const VT& v = ~vec;
  std::ostringstream oss;
  oss << "[ ";
  for (size_t i = 0; i < v.size(); ++i) {
    oss << v[i];
    if (i + 1 < v.size()) oss << ", ";
  }
  oss << " ]";
  return oss.str();
}

template <typename VT, bool TF>
static inline blaze::RemoveAdaptor_t<VT> fast_sin(const blaze::Vector<VT, TF>& vec)
{
  const VT& x = ~vec;
  using VectorType = blaze::RemoveAdaptor_t<VT>;
  using Scalar = blaze::ElementType_t<VT>;
  
  static_assert(std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
    "fast_cos only supports float or double");
  
  VectorType result(x.size());
  
  for (size_t i = 0; i < x.size(); ++i) {
    Scalar xi = x[i];
    Scalar x2 = xi * xi;
    
    if constexpr (std::is_same_v<Scalar, float>) {
      // 7th-degree minimax for sin(x), float
      result[i] = xi * (1.0f +
                x2 * (-1.666665710e-1f +
                x2 * (8.333017292e-3f +
                x2 * (-1.980661520e-4f +
                x2 * (2.601903036e-6f)))));
    } 
    else {
      // 7th-degree minimax for sin(x), double
      result[i] = xi * (1.0 +
                x2 * (-1.66666666666666307295e-1 +
                x2 * (8.33333333332211858878e-3 +
                x2 * (-1.98412698295895385996e-4 +
                x2 * (2.75573136213857245213e-6)))));
    }
  }
  
  return result;
}

template <typename VT, bool TF>
static inline blaze::RemoveAdaptor_t<VT> fast_cos(const blaze::Vector<VT, TF>& vec)
{
  const VT& x = ~vec;
  using VectorType = blaze::RemoveAdaptor_t<VT>;
  using Scalar = blaze::ElementType_t<VT>;
  
  static_assert(std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
    "fast_cos only supports float or double");
  
  VectorType result(x.size());
  
  for (size_t i = 0; i < x.size(); ++i) {
    Scalar xi = x[i];
    Scalar x2 = xi * xi;
    
    if constexpr (std::is_same_v<Scalar, float>) {
      // 6th-degree minimax for cos(x), float
      result[i] = 1.0f +
              x2 * (-0.5f +
              x2 * (4.166664568298827e-2f +
              x2 * (-1.388731625493765e-3f +
              x2 * (2.443315711809948e-5f))));
    } 
    else {
      // 6th-degree minimax for cos(x), double
      result[i] = 1.0 +
              x2 * (-0.5 +
              x2 * (4.16666666666666019037e-2 +
              x2 * (-1.38888888888741095749e-3 +
              x2 * (2.48015872894767294178e-5))));
    }
  }
  
  return result;
}

}

#endif
