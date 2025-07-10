#ifndef CTUP_TYPEDEFS_H
#define CTUP_TYPEDEFS_H
#include <type_traits>
#include <vector>
#include "blaze/Math.h"

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

template<typename SV>
using Allocator = blaze::AlignedAllocator<SV>;

template <typename SV>
using AlignedVec = std::vector<SV, Allocator<SV>>;

using alignedv_avx256f = AlignedVec<avx256f>;
using alignedv_avx512f = AlignedVec<avx512f>;
using alignedv_avx256d = AlignedVec<avx256d>;
using alignedv_avx512d = AlignedVec<avx512d>;

}

#endif
