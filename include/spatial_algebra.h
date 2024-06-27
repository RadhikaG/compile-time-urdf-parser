#ifndef SPATIALALGEBRA_H
#define SPATIALALGEBRA_H
#include "builder/forward_declarations.h"
#include "Eigen/Dense"

namespace SpatialAlgebra {



template<typename T>
struct Translation {
public:
  builder::dyn_var<T> x;
  builder::dyn_var<T> y;
  builder::dyn_var<T> z;

  cmatrix<double> skew() {
    cmatrix<double> rx({3,3});

    rx << 0, -z,  y,
          z,  0, -x,
         -y,  x,  0;

    return rx;
  }

  cmatrix<double> xlt(cmatrix<double> rx) {
    cmatrix<double> col1 = cmatrix::vstack(cmatrix::eye(3), -rx);
    cmatrix<double> col2 = cmatrix::vstack(cmatrix::zeros({3, 3}), cmatrix::eye(3));
    return cmatrix::hstack(col1, col2);
  }
};

template<typename T>
struct Rotation {

};

}

#endif
