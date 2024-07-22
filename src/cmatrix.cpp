#include "spatial_algebra.h"

namespace SpatialAlgebra {

std::vector<int> match_expr_sizes(const std::vector<int> size1, const std::vector<int> size2) {
  if (size1.size() == 0)
    return size2;
  if (size2.size() == 0)
    return size1;
  assert(size1.size() == size2.size() && "Sizes of arrays do not agree for operation");
  
  return size1;
}

}
